# -*- coding: utf-8 -*-
"""
auto_runner.py — START 자동 분류 루프 (pick_runner 위의 오케스트레이터) 초안.

★★★ 현재 비활성(PARKED). ENABLED=False — 자동진행 보류(위험). ★★★
   지금 운전은 "S 버튼 → 단일 잡기(pick_runner)" 로만. 자동 루프는 안정화 후 켠다.
   run()은 ENABLED=False면 아무것도 안 하고 즉시 반환(이동·세션·로깅 전부 안 함).
   dry_run은 실모션 없으니 비활성 상태에서도 흐름 점검 허용.

흐름:  SCAN → DETECT(안정 검출 대기) → PICK(pick_runner) → 반복.  STOP/HALT까지.
       pick_runner가 끝나면 팔은 SCAN자세 → 다음 사이클은 DETECT부터.

원칙(pick_runner와 동일):
  - 유일 CAN 권한자(브리지) 안에서 "별도 스레드"로 돈다. 직접 CAN/비전 안 씀 — 오직 api.
  - 빈 sleep 금지: 모든 대기는 should_stop/get_gen 폴링(STOP·HALT 즉시 감지).
  - 단계차트(api.set_step) + DB(log) 가 이 한 곳에서 찍힌다(단일 기록 지점).

연동(DI) — 브리지가 넘긴다:
  api.submit_and_wait(seq, speed=20) -> "done"|"preempted"|"timeout"
  api.request_gripper(angle)
  api.get_gen() -> int
  api.get_vision() -> dict      # 정규화된 최신 비전. 위치는 x_mm/y_mm (normalize가 arm_mm을 풀어줌)
  api.set_step(stage)           # 단계차트 "방송"(live)
  api.should_stop() -> bool     # STOP 눌림(HALT와 별개)

log(선택) = RobotLogger 인스턴스(실제 API, stateful). None이면 DB 생략:
  log.start_session(mode=, note=) · set_cycle(n) · set_step(stage)
  log.log_sort(object_type=, bin=, success=, yolo_conf=) · log_event(level, msg)
  log.end_session(total_sorted=)
  ※ 브리지 can_reader가 같은 log 인스턴스로 log_motor()를 찍으면 현재 cycle/step 태그를 상속.

⚠ 초안.
  - SCAN은 SCAN_POSE 정지+대기. M1 sweep 능동탐색은 ik_server sweep_worker 이식 TODO.
  - HALT(aborted)면 자동 복귀 안 함(물체 든 채일 수 있음) — 그 자리 멈춤(운전자 결정).
  - 풀 dry-run(안무 각도까지)은 ik_pose 있는 라파에서. 데브머신은 pick_runner가 error 반환.
  - 단일 인스턴스 보장(START 중복 방지)은 브리지 상태기계 책임.
"""

import time

import pick_runner
from pick_runner import SCAN_POSE, GRIPPER_OPEN   # 단일 출처(추후 poses.py로 통합)

# ─────────────────────────────────────────────────────────────────────
# ★ 안전장치: 자동진행 비활성. 안정화 전까지 절대 켜지 말 것.
#   현재 운전 = "S 버튼 → 단일 잡기(pick_runner)". 자동 루프는 보류.
#   켜는 조건: dry-run·단일물체·HALT 검증을 충분히 마친 뒤에만 True 로.
ENABLED = False
# ─────────────────────────────────────────────────────────────────────

DEFAULTS = {
    "conf_th": 0.5,          # 신뢰도 임계
    "stable_n": 5,           # 연속 안정 프레임 임계 (계약A §4)
    "ttl_ms": 2500,          # alive 판정(age_ms < ttl) — 버그 있는 fresh 대신 age 사용
    "detect_timeout_s": 8.0, # 이 안에 검출 없으면 빈 작업대로 보고 종료
    "scan_settle_s": 0.4,    # 스캔자세 도착 후 안정 대기
    "poll_s": 0.1,           # 폴링 주기
    "speed": 20,             # 스캔/홈 이동 속도(수동과 동일, pick만 30)
    "stop_on_empty": True,   # 검출 없으면 루프 종료
    "stop_on_abort": True,   # HALT(aborted)면 루프 종료
}

# 클래스 → 분류통 라벨 (DB sort_event.bin 용)
BIN_OF = {"PIPE": "pipe_v", "BOLT": "bolt", "NUT": "nut"}


def _alive(v, cfg):
    """파이프라인 살아있음 = age_ms < TTL. (물체 유무와 분리 — 계약A §6)"""
    return v.get("age_ms", 9e9) < cfg["ttl_ms"]


def _gate_ok(v, cfg):
    """잡기 게이트: 살아있음 · 물체있음 · 신뢰도 · 안정프레임 · 좌표유효."""
    return (_alive(v, cfg)
            and v.get("target", "NONE") != "NONE"
            and v.get("confidence", 0.0) >= cfg["conf_th"]
            and v.get("stable_frames", 0) >= cfg["stable_n"]
            and v.get("x_mm") is not None and v.get("y_mm") is not None)


def _idle(api, seconds, gen0):
    """대기하되 STOP/HALT면 즉시 깸. 끝까지 대기 True / 중단 False."""
    end = time.time() + seconds
    while time.time() < end:
        if api.should_stop() or api.get_gen() != gen0:
            return False
        time.sleep(0.02)
    return True


def _wait_detection(api, cfg, gen0):
    """안정 검출 대기. 게이트 충족 시 vision dict, STOP/HALT/timeout 시 None."""
    deadline = time.time() + cfg["detect_timeout_s"]
    while time.time() < deadline:
        if api.should_stop() or api.get_gen() != gen0:
            return None
        v = api.get_vision()
        if _gate_ok(v, cfg):
            return v
        time.sleep(cfg["poll_s"])
    return None


def _safe_move(api, pose, stage, cfg):
    """스캔/홈 이동. done 아니면(선점) False."""
    return api.submit_and_wait([(pose, stage)], speed=cfg["speed"]) == "done"


def run(api, cfg=None, log=None, dry_run=False):
    """자동 분류 루프. 브리지 START 핸들러가 스레드로 호출.
    반환: {"cycles": n, "sorted": k, "aborted": bool}
    """
    # ★ 안전장치: 비활성이면 실모션 없이 즉시 반환(이동·세션·로깅 전부 안 함).
    if not ENABLED and not dry_run:
        print("[auto_runner] 비활성(ENABLED=False): 자동진행 보류 — S버튼 단일잡기로 운전.")
        return {"cycles": 0, "sorted": 0, "aborted": False, "disabled": True}

    cfg = dict(DEFAULTS, **(cfg or {}))
    cycle = 0
    sorted_ok = 0
    aborted = False

    if log:
        log.start_session(mode="auto")

    def step(stage):
        api.set_step(stage)              # live 차트
        if log:
            log.set_step(stage)          # DB step_event + 현재 step 태그(motor_sample 상속)

    try:
        # 준비: 그리퍼 OPEN 보장(첫 잡기 전 닫힘 방지). 스캔 이동은 루프 첫 SCAN에서.
        if not dry_run:
            api.request_gripper(GRIPPER_OPEN)

        while not aborted and (dry_run or not api.should_stop()):
            cycle += 1
            if log:
                log.set_cycle(cycle)

            # SCAN  (TODO: 검출 없으면 M1 sweep 능동탐색)
            step("SCAN")
            if not dry_run:
                if not _safe_move(api, SCAN_POSE, "SCAN", cfg):
                    break                                  # 이동 선점(HALT) → 종료
                if not _idle(api, cfg["scan_settle_s"], api.get_gen()):
                    break                                  # settle 중 STOP/HALT

            # DETECT  — 안정 검출 대기
            step("DETECT")
            v = api.get_vision() if dry_run else _wait_detection(api, cfg, api.get_gen())
            if v is None or v.get("x_mm") is None:
                if cfg["stop_on_empty"]:
                    break                                  # 빈 작업대 / STOP / HALT
                continue

            # PICK  — pick_runner: GRASP→CARRY→PLACE→LIFT→RETURN (끝나면 SCAN자세)
            target = v.get("target", "NONE")
            result = pick_runner.run(
                api, [v["x_mm"], v["y_mm"]], target,
                angle_deg=v.get("angle_deg", 0.0),
                near=v.get("near"),                  # 계약A: detect가 gy>fh/2로 계산
                distance_mm=v.get("distance_mm"),    # near 없을 때만 폴백
                on_step=step, dry_run=dry_run,
            )
            kind = result[0]

            if kind == "done":
                sorted_ok += 1
                if log:
                    log.log_sort(object_type=target, bin=BIN_OF.get(target, target),
                                 success=True, yolo_conf=v.get("confidence"))
            elif kind == "aborted":                        # HALT/선점/timeout
                aborted = True
                if log:
                    log.log_sort(object_type=target, bin=BIN_OF.get(target, target),
                                 success=False, yolo_conf=v.get("confidence"))
                    log.log_event("WARN", "pick aborted: %s" % (result,))
                if cfg["stop_on_abort"]:
                    break
            else:                                          # error(IK 한계 등) → 그 물체 건너뜀
                if log:
                    log.log_event("WARN", "pick error: %s" % (result,))

            if dry_run:                                    # 드라이런은 1사이클만
                break
    finally:
        # 정상/STOP 종료에만 안전 준비자세(물체 없음). abort면 그 자리 유지.
        if not aborted and not dry_run:
            _safe_move(api, SCAN_POSE, "SCAN", cfg)
        api.set_step("IDLE")
        if log:
            log.end_session(total_sorted=sorted_ok)

    return {"cycles": cycle, "sorted": sorted_ok, "aborted": aborted}


if __name__ == "__main__":
    # 단독 dry-run: 루프 흐름 + pick_runner 안무를 한 사이클 출력 (라파에서 ik_pose 있으면 각도까지)
    class _FakeApi:
        def submit_and_wait(self, seq, speed=20):
            return "done"

        def request_gripper(self, a):
            print("  (gripper %d)" % a)

        def get_gen(self):
            return 0

        def should_stop(self):
            return False

        def set_step(self, s):
            print("[STEP] %s" % s)

        def get_vision(self):
            return {"target": "PIPE", "confidence": 0.77, "stable_frames": 6,
                    "x_mm": 181.0, "y_mm": 60.0, "angle_deg": 124.0,
                    "distance_mm": 215.0, "age_ms": 100}

    print("[dry-run] 1 cycle")
    print(run(_FakeApi(), dry_run=True))
