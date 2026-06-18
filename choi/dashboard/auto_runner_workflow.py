# -*- coding: utf-8 -*-
"""
auto_runner_workflow.py — 전체 작업 워크플로우 상태기계 (골격/초안).

choi auto_runner.py 초안(SCAN→DETECT→PICK 단순루프) 위에 실제 워크플로우를 얹은 것:
  파이프 → CNC가공(타이머20초) → 가공중 볼트/너트 제거 → 완성파이프 적재 → 반복.

★★★ ENABLED=False (PARKED). dry_run으로 "로직 흐름"만 검증. 실모션·티칭 없음. ★★★
  새 동작(pipe_to_cnc/grab_from_cnc/place_to_rack)은 티칭 나오면 pick_runner에 구현.
  여기선 placeholder(dry-run 출력)로 흐름만 확인.

연동(DI, choi 초안과 동일) — 브리지가 넘김:
  api.submit_and_wait(seq, speed=20) -> "done"|"preempted"|"timeout"
  api.request_gripper(angle) / api.get_gen() / api.get_vision() / api.set_step(stage) / api.should_stop()
  api.set_cnc_busy(bool)   # ★신규: cnc_busy면 detect가 파이프 제외(볼트/너트만 best). cnc_busy.flag 파일.

워크플로우 정책 (작업_워크플로우.md):
  - CNC 완료 = 타이머 20초 (센서 없음)
  - 적재함 가득(2개) = 미구현 (파레트로 빠짐 가정)
  - 가공중 + 볼트너트 없음 = 타이머 대기
  - CNC 1대 = 가공중엔 파이프 무시
  - 우선순위: CNC빔=파이프1순위 / 가공중=볼트너트(볼트먼저)
"""

import time
import threading

import pick_runner
from pick_runner import SCAN_POSE, GRIPPER_OPEN

# LED+부저 Nano 상태 송신(CAN 0x410). 모듈 없거나 python-can 미설치면 자동 비활성(워크플로우는 정상 동작).
try:
    from cnc_status_buzzer import (CncStatusCanNode, calc_cnc_status,
                                   remaining_cnc_seconds, open_socketcan_bus)
except Exception:
    CncStatusCanNode = None

ENABLED = False          # ★ 자동진행 비활성. 검증 끝나기 전 절대 True 금지.
CNC_TIMER_S = 20.0       # CNC 가공 시간(타이머)
MAX_GRIP_RETRY = 3       # 파지/회수 연속 실패 N회 → 정지(사람 확인)

DEFAULTS = {
    "conf_th": 0.5, "stable_n": 5, "ttl_ms": 2500,
    "detect_timeout_s": 8.0, "scan_settle_s": 0.4, "poll_s": 0.1,
    "speed": 20,
    "rack_slots": 2,     # 적재함 V홈 수
}


# ── 게이트/대기 (choi 초안과 동일) ──
def _alive(v, cfg):
    return v.get("age_ms", 9e9) < cfg["ttl_ms"]


def _gate_ok(v, cfg):
    return (_alive(v, cfg) and v.get("target", "NONE") != "NONE"
            and v.get("confidence", 0.0) >= cfg["conf_th"]
            and v.get("stable_frames", 0) >= cfg["stable_n"]
            and v.get("x_mm") is not None and v.get("y_mm") is not None)


def _wait_detection(api, cfg, gen0):
    """안정 검출 대기(detect가 스윕/센터링 알아서). 게이트 충족 시 vision, 아니면 None."""
    deadline = time.time() + cfg["detect_timeout_s"]
    while time.time() < deadline:
        if api.should_stop() or api.get_gen() != gen0:
            return None
        v = api.get_vision()
        if _gate_ok(v, cfg):
            return v
        time.sleep(cfg["poll_s"])
    return None


def _idle(api, seconds, gen0):
    end = time.time() + seconds
    while time.time() < end:
        if api.should_stop() or api.get_gen() != gen0:
            return False
        time.sleep(0.02)
    return True


def _safe_move(api, pose, stage, cfg):
    return api.submit_and_wait([(pose, stage)], speed=cfg["speed"]) == "done"


# ── 신규 동작 = pick_runner의 실제 함수 호출 (포즈는 pick_runner에서 티칭 빈칸) ──
def _pipe_to_cnc(api, arm_mm, v, dry_run):
    return pick_runner.pipe_to_cnc(api, arm_mm, angle_deg=v.get("angle_deg", 0.0),
                                   near=v.get("near"), distance_mm=v.get("distance_mm"),
                                   dry_run=dry_run, on_step=api.set_step)


def _grab_from_cnc(api, dry_run):
    return pick_runner.grab_from_cnc(api, dry_run=dry_run, on_step=api.set_step)


def _place_to_rack(api, slot, dry_run):
    return pick_runner.place_to_rack(api, slot, dry_run=dry_run, on_step=api.set_step)


def _sort_to_bin(api, arm_mm, target, v, dry_run):
    """볼트/너트 → 분류통 (기존 pick_runner.run)."""
    return pick_runner.run(api, arm_mm, target,
                           angle_deg=v.get("angle_deg", 0.0),
                           near=v.get("near"), distance_mm=v.get("distance_mm"),
                           on_step=api.set_step, dry_run=dry_run)


# ── 메인 워크플로우 루프 ──
def run(api, cfg=None, log=None, dry_run=False):
    if not ENABLED and not dry_run:
        print("[auto_runner] 비활성(ENABLED=False): 자동진행 보류 — S버튼 단일잡기로 운전.")
        return {"disabled": True}

    cfg = dict(DEFAULTS, **(cfg or {}))
    _now = lambda: getattr(api, "now", time.time)()   # 테스트 시계 주입(없으면 실시간)
    cnc_busy = False           # CNC에 파이프 있고 가공중?
    cnc_start = 0.0            # 파이프 넣은 시각
    rack_count = 0            # 적재한 파이프 수
    pipe_fail = 0            # 파이프 파지 연속 실패(재시도 카운터)
    cnc_fail = 0             # CNC 회수 연속 실패(재시도 카운터)
    cycle = 0
    sorted_ok = 0
    aborted = False
    # 0x410 송신 스레드가 읽는 공유 상태(루프가 갱신). loaded_until = 적재 직후 RACK_LOADED(부저3회) 표시 구간.
    led_ctx = {"busy": False, "start": 0.0, "rack": 0, "loaded_until": 0.0}
    _led_stop = threading.Event()
    _led_thread = None
    _led_bus = None

    def _led_worker(node):
        # CNC 상태(led_ctx) → calc_cnc_status → 0x410 송신(~5Hz). 워크플로우 루프 타이밍과 독립.
        while not _led_stop.is_set():
            now = time.time()
            busy = led_ctx["busy"]
            start = led_ctx["start"] if busy else None
            loaded = now < led_ctx["loaded_until"]
            try:
                st = calc_cnc_status(now=now, cnc_occupied=busy, cnc_process_started_at=start,
                                     rack_count=led_ctx["rack"], rack_capacity=cfg["rack_slots"],
                                     rack_loaded_event=loaded, cnc_process_sec=CNC_TIMER_S)
                rem = remaining_cnc_seconds(now=now, cnc_occupied=busy, cnc_process_started_at=start,
                                            cnc_process_sec=CNC_TIMER_S)
                node.send_status(st, rack_count=led_ctx["rack"],
                                 rack_capacity=cfg["rack_slots"], remaining_sec=rem)
            except Exception:
                pass
            _led_stop.wait(0.2)

    if log:
        log.start_session(mode="auto")

    def step(stage):
        api.set_step(stage)
        if log:
            log.set_step(stage)

    def set_cnc_busy(b):
        nonlocal cnc_busy
        cnc_busy = b
        led_ctx["busy"] = b    # 0x410 송신 스레드와 상태 공유
        api.set_cnc_busy(b)    # detect가 파이프 제외(cnc_busy면 볼트/너트만 best)

    try:
        if not dry_run:
            api.request_gripper(GRIPPER_OPEN)
            if CncStatusCanNode is not None:   # LED+부저 Nano 상태 송신(0x410) 시작
                try:
                    _led_bus = open_socketcan_bus("can0")
                    _led_thread = threading.Thread(target=_led_worker,
                                                   args=(CncStatusCanNode(_led_bus),), daemon=True)
                    _led_thread.start()
                    print("[auto_runner] LED 상태 송신(0x410) 시작")
                except Exception as e:
                    print(f"[auto_runner] LED 송신 비활성(CAN 못 엶): {e}")
                    _led_bus = None

        while not aborted and not api.should_stop():
            cycle += 1
            if cycle > 100:               # 안전 상한(무한 dry-run 방지)
                break
            if log:
                log.set_cycle(cycle)

            # ── SCAN ──
            step("SCAN")
            if not dry_run:
                if not _safe_move(api, SCAN_POSE, "SCAN", cfg):
                    break
                if not _idle(api, cfg["scan_settle_s"], api.get_gen()):
                    break

            # ── 가공중 타이머 만료 체크 (검출보다 먼저) ──
            if cnc_busy and (_now() - cnc_start) >= CNC_TIMER_S:
                if rack_count >= cfg["rack_slots"]:
                    # 적재함 가득 — 미구현(파레트 가정). 일단 경고+대기.
                    if log:
                        log.log_event("WARN", "적재함 가득(파레트 비우기 대기)")
                    if not _idle(api, 1.0, api.get_gen()):
                        break
                    continue
                step("RETRIEVE")
                print(f"  [회수] 타이머 만료 → CNC 파이프 회수 → 적재 V홈{rack_count+1}")
                r1 = _grab_from_cnc(api, dry_run)
                if r1[0] == "aborted":
                    aborted = True; break
                if r1[0] != "done":               # grip_fail/error → CNC 비었거나 회수 실패
                    cnc_fail += 1
                    print(f"  [재시도] CNC 회수 실패 {cnc_fail}/{MAX_GRIP_RETRY}")
                    if log:
                        log.log_event("WARN", f"CNC 회수 실패 {cnc_fail}회")
                    if cnc_fail >= MAX_GRIP_RETRY:
                        if log:
                            log.log_event("ERROR", "CNC 회수 연속 실패 — 정지(사람 확인)")
                        aborted = True; break
                    if not dry_run and not _idle(api, 0.5, api.get_gen()):
                        break
                    continue
                cnc_fail = 0
                r2 = _place_to_rack(api, rack_count, dry_run)
                if r2[0] == "aborted":
                    aborted = True; break
                if r2[0] == "done":
                    rack_count += 1
                    led_ctx["rack"] = rack_count
                    led_ctx["loaded_until"] = time.time() + 2.0   # 적재 직후 2초 RACK_LOADED(부저3회)
                    set_cnc_busy(False)
                    if log:
                        log.log_sort(object_type="PIPE", bin=f"rack{rack_count}", success=True)
                continue

            # ── DETECT (detect가 스윕/센터링 알아서, cnc_busy면 파이프 제외하고 best) ──
            step("DETECT")
            v = api.get_vision() if dry_run else _wait_detection(api, cfg, api.get_gen())
            if v is None or v.get("x_mm") is None:
                # 작업평면 비어있음
                if cnc_busy:
                    # 가공중 + 볼트너트 없음 → 타이머 대기
                    if not dry_run and not _idle(api, 0.5, api.get_gen()):
                        break
                    continue
                else:
                    # CNC 비고 물체 없음 → 스캔 대기 (다음 루프)
                    if not dry_run and not _idle(api, 0.5, api.get_gen()):
                        break
                    continue

            target = v.get("target", "NONE")
            arm_mm = [v["x_mm"], v["y_mm"]]

            # ── 판단 (상태 의존 우선순위) ──
            if not cnc_busy:
                if target == "PIPE":
                    step("PIPE_TO_CNC")
                    r = _pipe_to_cnc(api, arm_mm, v, dry_run)
                    if r[0] == "done":
                        set_cnc_busy(True); cnc_start = _now(); led_ctx["start"] = cnc_start; pipe_fail = 0
                        print("  [CNC] 파이프 투입 → 가공 시작(타이머 20초)")
                    elif r[0] == "grip_fail":
                        pipe_fail += 1
                        print(f"  [재시도] 파이프 파지 실패 {pipe_fail}/{MAX_GRIP_RETRY} → 재검출")
                        if log:
                            log.log_event("WARN", f"파이프 파지 실패 {pipe_fail}회")
                        if pipe_fail >= MAX_GRIP_RETRY:
                            if log:
                                log.log_event("ERROR", "파이프 파지 연속 실패 — 정지(사람 확인)")
                            aborted = True; break
                    elif r[0] == "aborted":
                        aborted = True; break
                else:  # BOLT/NUT → 분류통
                    step("SORT")
                    r = _sort_to_bin(api, arm_mm, target, v, dry_run)
                    if r[0] == "done":
                        sorted_ok += 1
                        if log:
                            log.log_sort(object_type=target, bin=target.lower(),
                                         success=True, yolo_conf=v.get("confidence"))
                    elif r[0] == "aborted":
                        aborted = True; break
            else:
                # cnc_busy → detect가 파이프 제외했으니 target은 볼트/너트
                step("SORT")
                r = _sort_to_bin(api, arm_mm, target, v, dry_run)
                if r[0] == "done":
                    sorted_ok += 1
                    if log:
                        log.log_sort(object_type=target, bin=target.lower(),
                                     success=True, yolo_conf=v.get("confidence"))
                elif r[0] == "aborted":
                    aborted = True; break
    finally:
        _led_stop.set()                      # 0x410 송신 스레드 정지
        if _led_thread is not None:
            _led_thread.join(timeout=1.0)
        if _led_bus is not None:
            _sd = getattr(_led_bus, "shutdown", None)
            if _sd is not None:
                try:
                    _sd()
                except Exception:
                    pass
        if not aborted and not dry_run:
            _safe_move(api, SCAN_POSE, "SCAN", cfg)
        api.set_step("IDLE")
        if log:
            log.end_session(total_sorted=sorted_ok)

    return {"cycles": cycle, "sorted": sorted_ok, "rack": rack_count, "aborted": aborted}


if __name__ == "__main__":
    # ── 다중사이클 dry-run: 전체 워크플로우 순서 검증 (실모션·티칭 없음) ──
    # api.now() 시계 + cnc_busy 따라 detect가 주는 vision을 시나리오로 주입.
    def _obj(t, x=181, y=60):
        return {"target": t, "x_mm": x, "y_mm": y, "confidence": .77,
                "stable_frames": 6, "age_ms": 100, "angle_deg": 124, "near": True}
    NONE = {"target": "NONE", "x_mm": None, "y_mm": None, "age_ms": 100}

    # 각 사이클: (now초, detect가 주는 vision). cnc_busy면 detect가 파이프 제외(여기선 시나리오로 반영).
    SCENES = [
        (0,  _obj("PIPE")),   # 1: CNC빔 → 파이프 잡아 CNC (cnc_start=0)
        (5,  _obj("BOLT")),   # 2: 가공중 → 볼트 분류
        (15, _obj("BOLT")),   # 3: 가공중 → 볼트 분류
        (21, NONE),           # 4: 타이머21≥20 → 회수 → 적재 V홈1
        (21, _obj("PIPE")),   # 5: CNC빔 → 파이프 잡아 CNC (cnc_start=21)
        (26, _obj("BOLT")),   # 6: 가공중 → 볼트 분류
        (42, _obj("BOLT")),   # 7: 타이머42-21=21≥20 → 회수 → 적재 V홈2
        (42, _obj("NUT")),    # 8: CNC빔 → 너트 분류
        (50, NONE),           # 9: 작업평면 빔 → 스캔 대기(종료)
    ]

    class _ScenApi:
        def __init__(self, scenes):
            self.s = scenes
            self.i = -1
            self.busy = False
        def _cur(self):
            return self.s[min(self.i, len(self.s) - 1)]
        def now(self):
            return float(self._cur()[0])
        def get_vision(self):
            return self._cur()[1]
        def should_stop(self):
            return self.i >= len(self.s) - 1
        def submit_and_wait(self, seq, speed=20):
            return "done"
        def request_gripper(self, a):
            pass
        def get_gen(self):
            return 0
        def set_cnc_busy(self, b):
            self.busy = b
        def set_step(self, stage):
            if stage == "SCAN":                       # 사이클 경계 = SCAN
                self.i += 1
                now, v = self._cur()
                tgt = v.get("target")
                print("\n=== CYCLE %d  now=%ss  busy=%s ===" % (self.i + 1, now, self.busy))
                print("  detect: %s" % tgt)
            elif stage in ("PIPE_TO_CNC", "SORT", "RETRIEVE"):
                print("  → 결정: %s" % stage)

    print("######## 다중사이클 워크플로우 dry-run (실모션 없음) ########")
    res = run(_ScenApi(SCENES), dry_run=True)
    print("\n######## 결과:", res, "########")
