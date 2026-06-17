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

import json
import time
from pathlib import Path

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
GRIP_OK_MAX = 130.0                     # ToF 파지판정: 거리 ≤ 이 값(mm)=잡음. 잡음84/못잡음(지면)235 사이

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
    near        : True/False. None이면 distance_mm < NEAR_MM 로 도출
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


# ════════════════════════════════════════════════════════════════════
# 워크플로우 신규 동작 (CNC 가공 + 적재). 기존 run()/build_phases/상수는 안 건드림.
#   → 단일 PICK(볼트/너트 분류)·대시보드 그대로 작동. 아래는 "추가"만.
# ════════════════════════════════════════════════════════════════════
# ★ 티칭 포즈: 대시보드 수동모드→[현재자세 저장]→브릿지가 teach_poses.json에 기록.
#   아래 상수는 폴백(미티칭=None → 실모션 시 안전 error). 실행 시점에 _teach()로 파일 우선 읽음(재티칭 즉시 반영).
CNC_INFEED = None          # 파이프 넣는 자세 (폴백; 파일 키 cnc_infeed)
CNC_GRAB = None            # CNC 픽스처서 완성 파이프 잡는 자세 (폴백; cnc_grab)
RACK_V = [None, None]      # 적재 V홈1, V홈2 (폴백; rack_v0/rack_v1). len=슬롯수

_TEACH_FILE = Path(__file__).resolve().parent / "teach_poses.json"


def _teach(name):
    """teach_poses.json에서 포즈 읽어 {1..6: float} 반환. 미티칭/파일없음 → None(=폴백)."""
    try:
        doc = json.loads(_TEACH_FILE.read_text(encoding="utf-8"))
        p = (doc.get("poses") or {}).get(name)
        if not p:
            return None
        return {int(k): float(v) for k, v in p["angles"].items()}
    except Exception:
        return None


def _staged(pose, label):
    """티칭 자세로 단계 이동: M1 → M5·6(그리퍼 방향) → M2·3·4(높이축, 마지막 하강).
       높이는 M2(어깨)·M4(팔꿈치)가 결정 → 팔을 스캔높이로 들고 접근(M1·M5·6) 후
       M2·3·4만 마지막에 내려 슬롯/픽스처에 하강 → 접근 중 그리퍼 충돌 방지.
       (M3는 M2 미러라 같이 움직임). 미티칭(None)→_exec_phases가 안전 error."""
    if not pose:
        return [(None, label)]
    return [({1: pose[1]}, label + " M1"),
            ({5: pose[5], 6: pose[6]}, label + " M5·6(방향, 들고)"),
            ({2: pose[2], 3: pose[3], 4: pose[4]}, label + " M2·3·4 하강(높이)")]


def _return_to_scan():
    """스캔 복귀(충돌 방지) — 접근의 역순:
       M2·3·4 먼저 들어올려(슬롯서 수직으로 빼냄) → M5·6 방향 → M1 회전.
       높이축(M2·3·4) 먼저 올려 슬롯 벗어난 뒤 손목/베이스 움직임 → 슬롯 내 충돌·M5 회전 방지.
       팔도 스캔높이(reach 367→172)로 접힌 뒤 M1 도니 휩쓸림도 최소화."""
    return [({m: SCAN_POSE[m] for m in (2, 3, 4)}, "복귀 M2·3·4(수직 빼냄)"),
            ({m: SCAN_POSE[m] for m in (5, 6)}, "복귀 M5·6(방향)"),
            ({1: SCAN_POSE[1]}, "M1→180")]


def _exec_phases(api, phases, dry_run, on_step):
    """phase 리스트 실행 (run과 동일 규약, 신규 동작 공유).
    targets가 None(티칭 미입력)이면: dry-run은 {TODO 티칭} 표시, 실모션은 error 반환."""
    for kind, stage, *rest in phases:
        if on_step:
            on_step(stage)
        if kind == "motion":
            legs = rest[0]
            if dry_run:
                print("  [%s]" % stage)
                for t, lbl in legs:
                    print("    %-18s %s" % (lbl, t if t is not None else "{TODO 티칭}"))
                continue
            for t, _ in legs:
                if t is None:
                    return ("error", "%s: 티칭 미입력(None)" % stage)
            res = api.submit_and_wait(legs, speed=PICK_SPEED)
            if res != "done":
                return ("aborted", stage, res)
        else:                                  # gripper
            angle, dwell, label = rest
            if dry_run:
                print("  [%s] %s  그리퍼 %d, dwell %.1fs" % (stage, label, angle, dwell))
                continue
            gen0 = api.get_gen()
            api.request_gripper(angle)
            if not _dwell(api, dwell, gen0):
                return ("aborted", stage, "halt")
    return ("done",)


def _grab_phases(cmd, near):
    """작업평면 잡기 (run의 PRE_GRASP→GRASP→LIFT와 동일 안무)."""
    m5_clear = M5_CLEAR_NEAR if near else M5_CLEAR_FAR
    return [
        ("motion", "PRE_GRASP", [({1: cmd[1], 5: m5_clear}, "M1조준+M5클리어"),
                                  ({2: cmd[2], 3: cmd[3], 4: cmd[4], 6: cmd[6]}, "M2·3·4·6"),
                                  ({5: cmd[5]}, "M5 최종(그립방향)")]),
        ("gripper", "GRASP", GRIPPER_CLOSE, GRIP_SETTLE + GRIP_HOLD_DELAY, "그리퍼 닫기(잡기)"),
        ("motion", "LIFT", [({2: SCAN_POSE[2], 3: SCAN_POSE[3]}, "M2·3→스캔"),
                             ({4: SCAN_POSE[4]}, "M4→스캔"), ({5: SCAN_POSE[5]}, "M5→스캔"),
                             ({6: SCAN_POSE[6]}, "M6→스캔")]),
    ]


def _grip_ok(api, dry_run):
    """LIFT 후 ToF로 파지 성공 판정. tof ≤ GRIP_OK_MAX = 잡음.
    dry_run 또는 ToF 미수신이면 통과(판정 보류 → 기존 동작 유지)."""
    if dry_run:
        return True
    read = getattr(api, "read_tof", None)
    mm = read() if read else None
    if mm is None:
        return True                                  # ToF 없음/미수신 → 판정 안 함(안전: 막지 않음)
    ok = mm <= GRIP_OK_MAX
    setr = getattr(api, "set_grip_result", None)
    if setr:
        setr("성공" if ok else "실패", mm)            # 대시보드 tof-judge 표시용
    return ok


def _release_and_scan():
    """파지 실패: 그리퍼 열고 M1→스캔(전체 스캔자세 복귀) → 재검출 준비."""
    return [
        ("gripper", "GRIP_FAIL", GRIPPER_OPEN, GRIP_SETTLE, "파지실패 그리퍼 열기"),
        ("motion", "GRIP_FAIL", [({1: SCAN_POSE[1]}, "M1→스캔")]),
    ]


def pipe_to_cnc(api, arm_mm, angle_deg=0.0, near=None, distance_mm=None,
                dry_run=False, on_step=None):
    """작업평면 파이프 잡기 → (ToF 파지확인) → CNC 인피드에 넣기. (cnc_busy 전환은 호출자=auto_runner)."""
    if solve_pick is None:
        return ("error", "ik_pose import 실패")
    x, y = float(arm_mm[0]), float(arm_mm[1])
    if near is None:
        near = (distance_mm is not None and float(distance_mm) < NEAR_MM)
    try:
        cmd = solve_pick(x, y, angle_deg, True)        # 파이프 = 각도 잡기
    except ValueError as e:
        return ("error", "solve_pick: %s" % e)
    if check_limits(cmd):
        return ("error", "한계 밖")
    grab = _exec_phases(api, _grab_phases(cmd, near), dry_run, on_step)   # PRE_GRASP→GRASP→LIFT(M1제외)
    if grab[0] != "done":
        return grab
    if not _grip_ok(api, dry_run):                     # ★ToF 파지판정 (LIFT 후, CNC 가기 전)
        _exec_phases(api, _release_and_scan(), dry_run, on_step)
        return ("grip_fail",)                          # cnc_busy 안 올림 → 루프 재검출·재잡기
    rest = [
        ("motion", "CNC_MOVE", _staged(_teach("cnc_infeed") or CNC_INFEED, "→CNC 인피드")),
        ("gripper", "CNC_INSERT", GRIPPER_OPEN, GRIP_SETTLE, "파이프 놓기(CNC)"),
        ("motion", "RETURN", _return_to_scan()),
    ]
    return _exec_phases(api, rest, dry_run, on_step)


def grab_from_cnc(api, dry_run=False, on_step=None):
    """CNC 픽스처(고정 티칭자세)서 완성 파이프 잡기 → 들기. (적재는 place_to_rack 별도)."""
    phases = [
        ("motion", "CNC_GRAB_MOVE", _staged(_teach("cnc_grab") or CNC_GRAB, "→CNC 픽스처")),
        ("gripper", "CNC_GRAB", GRIPPER_CLOSE, GRIP_SETTLE + GRIP_HOLD_DELAY, "완성 파이프 잡기"),
        ("motion", "LIFT", [({2: SCAN_POSE[2], 3: SCAN_POSE[3]}, "M2·3→스캔"),
                             ({4: SCAN_POSE[4]}, "M4→스캔"), ({5: SCAN_POSE[5]}, "M5→스캔"),
                             ({6: SCAN_POSE[6]}, "M6→스캔")]),
    ]
    grab = _exec_phases(api, phases, dry_run, on_step)
    if grab[0] != "done":
        return grab
    if not _grip_ok(api, dry_run):                     # ★ToF 회수 확인 (빈 CNC면 실패)
        _exec_phases(api, _release_and_scan(), dry_run, on_step)
        return ("grip_fail",)                          # place_to_rack 안 함 → rack_count 안 올림
    return ("done",)


def place_to_rack(api, slot, dry_run=False, on_step=None):
    """들고 있는 파이프를 적재 V홈[slot]에 놓기 (티칭, M6 가로방향, 5cm 하강)."""
    if not (0 <= slot < len(RACK_V)):
        return ("error", "잘못된 slot: %r" % (slot,))
    phases = [
        ("motion", "RACK_MOVE", _staged(_teach("rack_v%d" % slot) or RACK_V[slot], "→적재 V홈%d" % (slot + 1))),
        ("gripper", "RACK_PLACE", GRIPPER_OPEN, GRIP_SETTLE, "파이프 놓기(적재)"),
        ("motion", "RETURN", _return_to_scan()),
    ]
    return _exec_phases(api, phases, dry_run, on_step)


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
