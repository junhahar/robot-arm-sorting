# -*- coding: utf-8 -*-
"""
pick_runner.py — 잡기 안무(ik_server_pose.pick_sequence) 이식 초안.

역할: ik_server_pose.py의 "두뇌"(통위치·드롭자세·M5클리어·안무·클래스분기·IK)만 옮긴 것.
CAN 쓰기/S-curve 보간은 브리지 엔진(motion_worker/_move_to)이 담당 → 여기선 호출 안 함.
이 모듈은 "웨이포인트 + 순서 + 그리퍼/딜레이 + 단계명"만 담는다.

연동(계약 B · 의존성 주입): 브리지가 아래 api를 넘긴다. 이 모듈은 브리지를 import 안 함.
  api.submit_and_wait(seq, speed=20) -> "done" | "preempted" | "timeout"
  api.request_gripper(angle)
  api.get_gen() -> int            # 현재 motion gen (dwell 중 HALT 감지용)
IK 수학은 ik_pose에서 직접 import (junhahar 소유, 브리지와 동일 버전).

속도: 수동=20(브리지 기본), pick=30(=ik_server, M4 발열↓·동일 거동).
타이밍: 브리지 MIN_DURATION=0.8 = ik_server → min_dur 안 넘김(기본값 일치).
검증: dry_run=True로 절대 목표각만 출력 → ik_server pick_sequence와 대조(POSITION_ONLY 대체).

⚠ 그리퍼: 브리지 request_gripper는 1프레임 set(소프트 보간 없음).
   ik_server send_gripper는 cosine 램프였음 → 거동 다름. GRIP_SETTLE이 MG90 물리이동을
   덮는지 확인 필요. 부드러운 그립이 중요하면 브리지 그리퍼에 램프 추가(④ 검토).

⚠ 초안 — junhahar 검증/소유. 상수는 라파 ~/ik_server_pose.py(2026-06-14)에서 그대로 가져옴.
"""

import time

try:
    from ik_pose import solve_pick, check_limits
except Exception:                      # dry-run 단독 테스트 시 ik_pose 부재 허용
    solve_pick = None
    check_limits = None

# ── pick 속도 (수동 20과 분리) ──
PICK_SPEED = 30.0

# ── 자세 상수 (ik_server_pose.py에서 이식) ──
# ★ SCAN_POSE는 브리지에도 있음(중복) → 추후 poses.py로 단일화. 값은 브리지와 반드시 일치.
SCAN_POSE = {1: 180.0, 2: 104.8, 3: 248.2, 4: 112.7, 5: 140.8, 6: 180.0}
M5_CLEAR_NEAR = SCAN_POSE[5] + 55.0    # 195.8 — 가까운 물체(팔 더 접힘 → 충돌위험↑)일수록 더 올림
M5_CLEAR_FAR = SCAN_POSE[5] + 40.0     # 180.8

BIN_M1_PIPE = 60.0                      # 파이프 V홈
BIN_M1_BOLT = 270.0                     # 볼트 분류통
BIN_M1_NUT = 300.0                      # 너트 분류통
DROP_REST = {2: 79.5, 3: 273.4, 4: 123.2, 5: 181.1, 6: 180.0}        # 볼트·너트 드롭
DROP_REST_PIPE = {2: 76.5, 3: 276.4, 4: 103.5, 5: 170.0, 6: 180.0}   # 파이프 V홈 드롭

GRIPPER_CLOSE = 180                     # 잡기
GRIPPER_OPEN = 60                       # 놓기
GRIP_SETTLE = 0.8                       # 그리퍼 동작 완료 대기[s]
GRIP_HOLD_DELAY = 1.0                   # 잡은 뒤 딜레이[s]

VALID_TARGETS = ("BOLT", "NUT", "PIPE")
# near 미지정 시 기본값 (정석: 브리지가 계약A distance_mm로 도출해 넘김)
NEAR_MM = 250.0


def build_phases(cmd, is_bolt, is_pipe, near):
    """잡기 안무를 phase 리스트(순수 데이터)로 생성. 실행 안 함 → dry-run 가능.
    phase = ("motion",  stage, [(targets, label), ...])
          | ("gripper", stage, angle, dwell_s, label)
    stage = 단계차트/DB용 단계명 (PRE_GRASP/GRASP/LIFT/CARRY/PLACE/RETURN).
    motion phase 안의 leg들은 하나의 seq로 묶여 순서대로 제출된다
    (자기선점 방지 + _move_to가 leg마다 gen 체크 → HALT는 phase 중간도 끊음).
    """
    m5_clear = M5_CLEAR_NEAR if near else M5_CLEAR_FAR
    bin_m1 = BIN_M1_PIPE if is_pipe else BIN_M1_BOLT if is_bolt else BIN_M1_NUT
    drop = DROP_REST_PIPE if is_pipe else DROP_REST

    return [
        # [A] 접근·잡기자세: M5 먼저 클리어(들기) → 어깨·팔꿈치·roll 접기 → M5 최종 그립방향
        ("motion", "PRE_GRASP", [
            ({1: cmd[1], 5: m5_clear}, "M1조준+M5클리어"),
            ({2: cmd[2], 3: cmd[3], 4: cmd[4], 6: cmd[6]}, "M2·3·4·6"),
            ({5: cmd[5]}, "M5 최종(그립방향)"),
        ]),
        # [B] 그리퍼 닫기 + 딜레이
        ("gripper", "GRASP", GRIPPER_CLOSE, GRIP_SETTLE + GRIP_HOLD_DELAY, "그리퍼 닫기(잡기)"),
        # [C] 들어올리기 (M1 유지, 스캔값으로)
        ("motion", "LIFT", [
            ({2: SCAN_POSE[2], 3: SCAN_POSE[3]}, "M2·3→스캔"),
            ({4: SCAN_POSE[4]}, "M4→스캔"),
            ({5: SCAN_POSE[5]}, "M5→스캔"),
            ({6: SCAN_POSE[6]}, "M6→스캔"),
        ]),
        # [D] 분류통 회전 + 하강 (클래스 분기)
        ("motion", "CARRY", [
            ({1: bin_m1}, "M1→%.0f(통 회전)" % bin_m1),
            ({5: drop[5]}, "M5(클리어)"),
            ({4: drop[4]}, "M4"),
            ({2: drop[2], 3: drop[3]}, "M2·3 하강"),
            ({6: drop[6]}, "M6"),
        ]),
        # [E] 그리퍼 열기 (놓기)
        ("gripper", "PLACE", GRIPPER_OPEN, GRIP_SETTLE, "그리퍼 열기(놓기)"),
        # [F] 통에서 빼기 (스캔값)
        ("motion", "LIFT", [
            ({2: SCAN_POSE[2], 3: SCAN_POSE[3]}, "M2·3→스캔"),
            ({4: SCAN_POSE[4]}, "M4→스캔"),
            ({5: SCAN_POSE[5]}, "M5→스캔"),
            ({6: SCAN_POSE[6]}, "M6→스캔"),
        ]),
        # [G] 스캔자세 복귀 (M1→180)
        ("motion", "RETURN", [({1: SCAN_POSE[1]}, "M1→180")]),
    ]


def _dwell(api, seconds, gen0):
    """그리퍼 동작 후 대기. 도중 motion gen이 바뀌면(HALT) 즉시 중단.
    (계약 B에 abortable_sleep 없으므로 get_gen 폴링으로 직접 구현)"""
    end = time.time() + seconds
    while time.time() < end:
        if api.get_gen() != gen0:
            return False               # 선점됨(HALT)
        time.sleep(0.02)
    return True


def run(api, arm_mm, target, angle_deg=0.0, distance_mm=None, near=None,
        dry_run=False, on_step=None):
    """잡기 1회 실행. (브리지 PICK 핸들러가 게이트 통과 후 스레드로 호출)

    api         : 브리지 주입 (submit_and_wait / request_gripper / get_gen)
    arm_mm      : [x, y] — ARM 프레임 좌표(= 계약A arm_mm, = 현 5005 값)
    target      : "BOLT" | "NUT" | "PIPE"
    angle_deg   : 잡기 각도(볼트·파이프만 사용, 너트는 오프셋 안 탐) = ik_server 'bolt'
    near        : True/False (계약A near = detect가 gy>fh/2로 계산). None이면 distance_mm<NEAR_MM 폴백
    dry_run     : True면 제출 없이 절대 목표각만 출력(ik_server 대조용)
    on_step(stage): 단계 콜백(선택) — 단계차트/DB용. None이면 무시

    반환: ("done",)
        | ("aborted", stage, reason)   # reason: preempted|timeout|halt
        | ("error", why)
    """
    if solve_pick is None:
        return ("error", "ik_pose import 실패")
    if target not in VALID_TARGETS:
        return ("error", "알 수 없는 target: %r" % (target,))

    is_pipe = (target == "PIPE")
    is_bolt = (target == "BOLT")
    x, y = float(arm_mm[0]), float(arm_mm[1])
    if near is None:
        near = (distance_mm is not None and float(distance_mm) < NEAR_MM)

    # ── IK 계산 + 한계 검증 (ik_server validate 이식, json의 ik는 무시하고 재계산) ──
    try:
        cmd = solve_pick(x, y, angle_deg, is_bolt or is_pipe)  # 파이프=볼트처럼 각도 잡기
    except ValueError as e:
        return ("error", "solve_pick: %s" % e)
    bad = check_limits(cmd)
    if bad:
        return ("error", "한계 밖: " + str([(m, v) for m, v, _ in bad]))

    # ── 안무 실행 ──
    for kind, stage, *rest in build_phases(cmd, is_bolt, is_pipe, near):
        if on_step:
            on_step(stage)
        if kind == "motion":
            legs = rest[0]
            if dry_run:
                print("  [%s]" % stage)
                for targets, lbl in legs:
                    print("    %-16s %s" % (lbl, targets))
                continue
            res = api.submit_and_wait(legs, speed=PICK_SPEED)
            if res != "done":            # preempted/timeout → 중단 (브리지 HALT가 현재각 홀드)
                return ("aborted", stage, res)
        else:                            # gripper
            angle, dwell, label = rest
            if dry_run:
                print("  [%s] %s  그리퍼 %d, dwell %.1fs" % (stage, label, angle, dwell))
                continue
            gen0 = api.get_gen()
            api.request_gripper(angle)
            if not _dwell(api, dwell, gen0):
                return ("aborted", stage, "halt")   # 물체 든 채 HALT → 그리퍼 닫힘 유지(안 떨어짐)

    return ("done",)


if __name__ == "__main__":
    # 단독 dry-run: ik_server pick_sequence와 같은 절대 목표각이 나오는지 대조
    class _FakeApi:
        def submit_and_wait(self, seq, speed=20):
            return "done"

        def request_gripper(self, a):
            pass

        def get_gen(self):
            return 0

    print("[dry-run] PIPE @ arm_mm(181,60), angle 124, near")
    print(run(_FakeApi(), [181, 60], "PIPE", angle_deg=124, near=True, dry_run=True))
