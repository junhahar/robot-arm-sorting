#!/usr/bin/env python3
"""
rpi_keyboard_control_v1_10.py — v1_9 + '7' 임의 각도 입력 자세 이동(S-curve)
================================================================
v1_9 대비 추가된 것 (캘리브 k/j/p/l, 이동·영점·홈·자세·그리퍼 전부 그대로):
  · ★ '7' 키: 모터 각도 6개(M1..M6)를 직접 입력 → 그 자세로 S-curve 이동.
    카메라(detect_and_ik)가 준 IK각 6개를 손으로 입력해 그 자세로 가보는 용도.
    8·9(카메라 자세)·0(홈)과 동일한 순차 S-curve(M1→M4→M2·3→M5→M6) 사용.
    입력 동안 수동이동 정지. 빈 줄 입력=취소.
    ★ 관절 소프트 한계(MOTOR_LIMITS) 검사: 입력각이 한계 밖이면 이동 거부+경고.
    수동이동(q/a 등)도 같은 한계에서 멈춘다(clamp) → 하드스톱 충돌 방지.

v1_8 대비 바뀐 점 (이동·영점·홈·자세·그리퍼 기능은 전부 그대로):
  · 노트북 detect_and_ik가 계산한 "IK 목표각"과, 키보드로 실제 그 물체를
    집었을 때 STM이 보내준 "실측각"을 짝지어 CSV로 모은다.
  · 모인 (IK각, 실측각) 쌍으로 모터별 DIR(±1)·OFFSET을 회귀 추정 →
    ik.py의 M*_OFFSET / M*_DIR 에 그대로 넣을 값을 출력한다.
  · IK각은 비교/캘리브 용도일 뿐, 이 스크립트는 IK각으로 모터를 움직이지 않는다.

캘리브 워크플로 (물체 1개 = 1샘플):
  1) 카메라 앞 물체 감지 → 노트북 detect_and_ik에서 C → IK각 6개 화면에 표시.
  2) RPi(이 창)에서  k  → 그 IK각 6개를 한 줄로 입력 (예: 180 100.7 252.3 144.2 170.8 90).
  3) 키보드(q/a w/s ...)로 실제 그 물체를 집는 자세까지 이동.
  4) 집은 자세에서  j  → 현재 실측각 6개 + 방금 입력한 IK각 → CSV 한 줄 저장.
  5) 물체 위치 바꿔가며 1~4 반복 (모터당 6개 이상 권장, 각도 폭 넓게).
  6)  p  → 쌓인 CSV로 모터별 DIR/OFFSET 회귀 → ik.py에 넣을 값 출력.

캘리브 키 (기존 키와 안 겹침)
  k = IK각 6개 입력 (이 동안 모터 정지)
  j = 현재 실측각으로 샘플 저장 (직전 k 입력 IK각과 짝)
  p = 회귀 실행 → DIR/OFFSET 출력
  l = 지금까지 모은 샘플 수 보기
CSV 파일: ik_calib_samples.csv  (sample,motor,ik_deg,real_deg 형식, append)

기존 키 매핑 (v1_8과 동일)
  이동:  M1 q/a   M2·3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g
  영점:  z=M1  x=M2·3  c=M4  v=M5  b=M6   (180° 중위보정)
  홈복귀(S-curve): 1~6 그 모터 180°,  0 전체 180°
  카메라 자세(S-curve): 9 / 8 (M1→M4→M2·3→M5→M6 순차)
  프리로드(M2·3): ] = +0.5,  [ = -0.5
  그리퍼(MG90): y=+2°  h=-2° (톡톡, 꾹누르면 또박또박)     종료: ESC

  ★자세입력: 7 = 각도 6개(M1..M6) 입력 → 그 자세로 S-curve 이동
CAN 셋업: sudo ip link set can0 type can bitrate 125000 sample-point 0.625
실행:     python3 rpi_keyboard_control_v1_10.py
"""

import sys
import os
import csv
import time
import struct
import threading
import termios
import tty
import select
import can

# ── CAN 설정 ──
CAN_INTERFACE = "can0"
CAN_ID_CMD = 0x100
CAN_ID_TELEM = 0x201
CMD_TORQUE = 0x02
CMD_SET_ANGLE = 0x03
CMD_SET_MIDDLE = 0x04
CMD_GRIPPER = 0x06  # STM32 MG90 그리퍼 PWM (0x100, [cmd, angle])

# ── 제어 파라미터 ──
MOTOR_COUNT = 6
ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
DEFAULT_ANGLE = 180.0

# ── 관절 소프트 한계 (실측, ik.py MOTOR_LIMITS와 동일) ──
# 명령각이 이 범위를 넘지 않게: 수동이동은 끝에서 멈추고(clamp), 7번 입력은 거부.
# M1은 회전 자유 → 0~360. 하드스톱보다 약간 안쪽으로 잡은 소프트 한계.
MOTOR_LIMITS = {
    1: (0.0, 360.0),
    2: (63.0, 182.0),
    3: (178.0, 297.0),
    4: (84.0, 181.0),
    5: (115.0, 250.0),
    6: (90.0, 270.0),
}


def clamp_limit(mid, deg):
    """모터 mid의 소프트 한계 안으로 deg를 자른다(수동이동용)."""
    lo, hi = MOTOR_LIMITS.get(mid, (ANGLE_MIN, ANGLE_MAX))
    return max(lo, min(hi, deg))


def check_pose_limits(pose):
    """자세 dict{mid:deg}에서 한계 밖 항목을 (mid, deg, (lo,hi)) 리스트로 반환."""
    bad = []
    for mid, deg in pose.items():
        lo, hi = MOTOR_LIMITS.get(mid, (ANGLE_MIN, ANGLE_MAX))
        if not (lo <= deg <= hi):
            bad.append((mid, round(deg, 1), (lo, hi)))
    return bad


HOME_ANGLE = 180.0
RATE_HZ = 40.0
DT = 1.0 / RATE_HZ
STREAM_MOVE_MS = 80
RAMP_RATE = 20.0
MAX_SPEED = 20.0
HOLD_TIMEOUT = 0.5

# ── 홈복귀(S-curve) 파라미터 ──
HOME_SPEED = 20.0
HOME_MIN_DURATION = 0.8

# ── 카메라 자세(S-curve) 파라미터 ──
CAMERA_SPEED = 20.0
CAMERA_MIN_DURATION = 0.8

# ── 카메라 촬영 자세 (모터별 명령각) ──
# 두 자세 모두 실제로 그 위치로 손수 옮긴 뒤 상태줄 실측각을 보고 확정한 값(v1_4~v1_8 이력).
# 9번: 기준 촬영 자세.  8번: M5만 165.6→170.8로 직접 조정한 변형 자세.
# M1·M6=180 고정. 값 바꾸려면 다시 손으로 옮겨 상태줄 각을 읽어 갱신할 것.
CAMERA_POSE = {1: 180.0, 2: 104.8, 3: 248.2, 4: 112.7, 5: 146.0, 6: 180.0}  # 9번
CAMERA_POSE2 = {1: 180.0, 2: 100.7, 3: 252.3, 4: 144.2, 5: 170.8, 6: 180.0}  # 8번

# ── MG90 그리퍼 파라미터 ──
GRIPPER_INIT = 90
GRIPPER_STEP = 30
GRIPPER_MIN = 0
GRIPPER_MAX = 180

# M2·M3 거울쌍 부하 분담용 프리로드[deg].
MIRROR_PRELOAD = 0.0
PRELOAD_STEP = 0.5
PRELOAD_MAX = 20.0

# ── 캘리브 설정 ──
CALIB_CSV = "ik_calib_samples.csv"  # 샘플 누적 파일 (sample,motor,ik_deg,real_deg)
CALIB_NEUTRAL = 180.0  # ik.py와 동일한 중립 기준각 (cmd_real = OFFSET + DIR*(ik-180))

# ── 논리 축 정의 ── members = ((motor_id, sign), ...)
# gain = 그 축의 속도 배율. M4는 무거워 잘 안 움직여 키울 필요가 있는데,
#   처음 5.0은 너무 급해서(끊김/과주행) 사용자 요청으로 2.0으로 낮춰 확정(v1_5~).
AXES = [
    {
        "label": "M1",
        "fwd": "q",
        "back": "a",
        "calib": "z",
        "gain": 1.0,
        "preload": 0.0,
        "members": ((1, +1),),
    },
    {
        "label": "M2·3",
        "fwd": "w",
        "back": "s",
        "calib": "x",
        "gain": 1.0,
        "preload": MIRROR_PRELOAD,
        "members": ((2, +1), (3, -1)),
    },
    {
        "label": "M4",
        "fwd": "e",
        "back": "d",
        "calib": "c",
        "gain": 2.0,
        "preload": 0.0,
        "members": ((4, +1),),
    },  # gain 2.0: 5.0에서 낮춤
    {
        "label": "M5",
        "fwd": "r",
        "back": "f",
        "calib": "v",
        "gain": 1.0,
        "preload": 0.0,
        "members": ((5, +1),),
    },
    {
        "label": "M6",
        "fwd": "t",
        "back": "g",
        "calib": "b",
        "gain": 1.0,
        "preload": 0.0,
        "members": ((6, +1),),
    },
]

# 카메라 자세 이동 순서 (축 인덱스): M1 → M4 → M2·3 → M5 → M6
POSE_SEQ = [0, 2, 1, 3, 4]

MOVE_KEYS = {k for ax in AXES for k in (ax["fwd"], ax["back"])}
CALIB_KEYS = {ax["calib"]: ai for ai, ax in enumerate(AXES)}
MOTOR_AXIS = {
    mid: (ai, sign) for ai, ax in enumerate(AXES) for (mid, sign) in ax["members"]
}
KEY_AXIS = {ax["fwd"]: ai for ai, ax in enumerate(AXES)}
KEY_AXIS.update({ax["back"]: ai for ai, ax in enumerate(AXES)})
MIRROR_AXIS_IDX = next(i for i, ax in enumerate(AXES) if len(ax["members"]) > 1)


def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)  # s'(0)=s'(1)=0


class MeasuredAngles(can.Listener):
    def __init__(self):
        self._lock = threading.Lock()
        self._angle = {}

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == CAN_ID_TELEM and len(msg.data) >= 7:
            mid, ax10, _t, _l, flags = struct.unpack("<BHBHB", bytes(msg.data[0:7]))
            if flags & 0x01:
                with self._lock:
                    self._angle[mid] = ax10 / 10.0

    def get(self, motor_id):
        with self._lock:
            return self._angle.get(motor_id)

    def clear(self, motor_id):
        with self._lock:
            self._angle.pop(motor_id, None)

    def snapshot(self):
        with self._lock:
            return dict(self._angle)


def send_torque(bus, motor_id, on):
    data = [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass


def send_angle(bus, motor_id, deg, move_ms=0):
    deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))
    ax10 = int(round(deg * 10))
    mt = int(move_ms) & 0xFFFF
    data = [
        CMD_SET_ANGLE,
        motor_id,
        ax10 & 0xFF,
        (ax10 >> 8) & 0xFF,
        mt & 0xFF,
        (mt >> 8) & 0xFF,
        0,
        0,
    ]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass


def send_set_middle(bus, motor_id):
    data = [CMD_SET_MIDDLE, motor_id, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass


def send_gripper(bus, angle):
    angle = max(GRIPPER_MIN, min(GRIPPER_MAX, int(round(angle))))
    data = [CMD_GRIPPER, angle & 0xFF, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass
    return angle


# ══════════════════════════════════════════════════════════════
# 캘리브: 입력 / 저장 / 회귀
# ══════════════════════════════════════════════════════════════
def prompt_ik_angles(fd, old_attr):
    """raw 모드를 잠깐 풀고 IK각 6개를 한 줄로 입력받아 dict{mid:deg} 반환(취소면 None)."""
    termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)  # 일반 입력 모드로 복귀
    try:
        sys.stdout.write(
            "\n[캘리브] 노트북 IK각 6개 입력 (M1..M6, 공백구분). 취소=빈줄\n  IK> "
        )
        sys.stdout.flush()
        line = sys.stdin.readline().strip()
        if not line:
            print("[캘리브] 입력 취소")
            return None
        vals = line.replace(",", " ").split()
        if len(vals) != MOTOR_COUNT:
            print(f"[캘리브] 6개 필요 (입력 {len(vals)}개) — 취소")
            return None
        ik = {i + 1: float(v) for i, v in enumerate(vals)}
        print(
            "[캘리브] IK각 잠금: " + "  ".join(f"M{m}={ik[m]:.1f}" for m in range(1, 7))
        )
        print("        → 키보드로 그 물체 집은 뒤  j  눌러 저장")
        return ik
    except ValueError:
        print("[캘리브] 숫자 변환 실패 — 취소")
        return None
    finally:
        tty.setcbreak(fd)  # raw 모드 복귀


def prompt_pose_angles(fd, old_attr):
    """raw 모드를 잠깐 풀고 자세이동용 모터각 6개(M1..M6)를 입력받아
    dict{mid:deg} 반환(취소면 None). 여기선 0~360 클램프만; 관절 한계 검사는
    호출부(7번)에서 check_pose_limits로 하고 밖이면 이동을 거부한다."""
    termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)  # 일반 입력 모드로 복귀
    try:
        sys.stdout.write(
            "\n[자세입력] 이동할 모터각 6개 입력 (M1..M6, 공백구분). 취소=빈줄\n  POSE> "
        )
        sys.stdout.flush()
        line = sys.stdin.readline().strip()
        if not line:
            print("[자세입력] 취소")
            return None
        vals = line.replace(",", " ").split()
        if len(vals) != MOTOR_COUNT:
            print(f"[자세입력] 6개 필요 (입력 {len(vals)}개) — 취소")
            return None
        pose = {}
        for i, v in enumerate(vals):
            deg = float(v)
            deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))  # 0~360 클램프(한계검사 아님)
            pose[i + 1] = deg
        print(
            "[자세입력] 목표: " + "  ".join(f"M{m}={pose[m]:.1f}" for m in range(1, 7))
        )
        print("        → S-curve로 그 자세로 이동(M1→M4→M2·3→M5→M6)")
        return pose
    except ValueError:
        print("[자세입력] 숫자 변환 실패 — 취소")
        return None
    finally:
        tty.setcbreak(fd)  # raw 모드 복귀


def save_sample(ik_angles, real_angles):
    """(IK각, 실측각) 한 샘플을 CSV에 append. 다음 sample 번호 반환."""
    new = not os.path.exists(CALIB_CSV)
    sample_no = 0
    if not new:
        try:
            with open(CALIB_CSV, "r") as f:
                rows = list(csv.reader(f))
            nums = [int(r[0]) for r in rows[1:] if r and r[0].isdigit()]
            sample_no = (max(nums) + 1) if nums else 0
        except Exception:
            sample_no = 0
    with open(CALIB_CSV, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["sample", "motor", "ik_deg", "real_deg"])
        for m in range(1, MOTOR_COUNT + 1):
            r = real_angles.get(m)
            if r is None:
                continue
            w.writerow([sample_no, m, f"{ik_angles[m]:.2f}", f"{r:.2f}"])
    return sample_no


def count_samples():
    if not os.path.exists(CALIB_CSV):
        return 0, {}
    per_motor = {m: 0 for m in range(1, MOTOR_COUNT + 1)}
    samples = set()
    with open(CALIB_CSV, "r") as f:
        for r in list(csv.reader(f))[1:]:
            if len(r) >= 4 and r[0].isdigit():
                samples.add(int(r[0]))
                per_motor[int(r[1])] = per_motor.get(int(r[1]), 0) + 1
    return len(samples), per_motor


def run_regression():
    """CSV 전체로 모터별 선형보정: real = a*ik + b (최소제곱).
    DIR을 ±1로 강제하지 않고 기울기 a(스케일)를 그대로 출력 → IK→실물 변환식.
    잔차(RMS)와 적합도(R²)도 함께 출력해 신뢰도를 판단."""
    if not os.path.exists(CALIB_CSV):
        print("[회귀] 샘플 파일 없음 — 먼저 k/j로 모으세요")
        return
    data = {m: [] for m in range(1, MOTOR_COUNT + 1)}
    with open(CALIB_CSV, "r") as f:
        for r in list(csv.reader(f))[1:]:
            if len(r) >= 4 and r[0].isdigit():
                m = int(r[1])
                data[m].append((float(r[2]), float(r[3])))  # (ik, real)

    print("\n" + "=" * 60)
    print(" IK→실물 선형보정 결과   real = a*IK + b   (스케일 a 포함)")
    print(" (DIR ±1 강제 안 함. a가 IK→실물 변환 기울기, b가 절편)")
    print("=" * 60)
    print(" ik.py 변환식에 넣을 값  cmd[m] = a*theta_ik + b :")
    for m in range(1, MOTOR_COUNT + 1):
        pts = data[m]
        n = len(pts)
        if n < 2:
            print(f"  M{m}: 샘플 {n}개 — 부족(2개 이상 필요), 건너뜀")
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        syy = sum((y - my) ** 2 for y in ys)
        if sxx < 1e-6:
            print(f"  M{m}: IK각이 거의 안 변함(샘플 폭 좁음) — 각도 폭 넓혀 재수집")
            continue
        a = sxy / sxx  # 기울기(스케일)
        b = my - a * mx  # 절편
        # 잔차: 실제 적용식 real=a*IK+b 로 평가
        res = [(a * x + b) - y for x, y in pts]
        rms = (sum(e * e for e in res) / n) ** 0.5
        # 적합도 R² (1에 가까울수록 직선이 잘 맞음). syy=0이면(실측 변화 없음) 의미없음.
        r2 = 1.0 - (sum(e * e for e in res) / syy) if syy > 1e-6 else float("nan")
        flag = ""
        if rms > 5.0:
            flag = "  ⚠잔차 큼(직선으로 설명 안 됨 → IK좌표/사람자세 일관성 의심)"
        elif r2 < 0.9:
            flag = "  ⚠적합도 낮음(R²<0.9)"
        print(
            f"  M{m}: a={a:+.4f}, b={b:+7.2f}"
            f"   (n={n}, 잔차RMS={rms:.2f}°, R²={r2:.3f}){flag}"
        )
    print("=" * 60)
    print(" 해석: a≈1이면 스케일 OK(영점만 차이), a≠1이면 IK가 스케일까지 어긋남.")
    print(" 잔차RMS<3°·R²>0.95 면 신뢰. 크면 IK좌표(링크/호모그래피) 또는")
    print(" 사람이 집는 자세 일관성 문제 → 샘플 더(폭 넓게) 모으거나 IK 재검토.\n")


def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CAN_INTERFACE} 열기 실패: {e}")
        print("       sudo ip link set can0 type can bitrate 125000 sample-point 0.625")
        return

    measured = MeasuredAngles()
    notifier = can.Notifier(bus, [measured])

    for m in range(1, MOTOR_COUNT + 1):
        send_torque(bus, m, True)
        time.sleep(0.02)
    time.sleep(0.5)

    target = {m: None for m in range(1, MOTOR_COUNT + 1)}
    axis_speed = {ai: 0.0 for ai in range(len(AXES))}
    axis_prev_dir = {ai: 0 for ai in range(len(AXES))}
    last_seen = {}
    homing = set()
    home_start = {}
    pose_idx = None
    pose_goal = CAMERA_POSE
    pose_start = {}
    gripper_angle = send_gripper(bus, GRIPPER_INIT)
    pending_ik = None  # k로 입력해 잠가둔 IK각 dict (j로 저장하면 비움)

    print("키보드 제어 v1_10 시작 (STS3215 + MG90, 캘리브 + 7번 자세입력)")
    print("  이동:  M1 q/a   M2·3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g")
    print("  영점:  z=M1  x=M2·3  c=M4  v=M5  b=M6   (180° 중위보정)")
    print("  홈복귀(S-curve): 1~6 그 모터 180°,  0 전체 180°")
    print("  카메라 자세(S-curve): 9 / 8 (M1→M4→M2·3→M5→M6 순차)   프리로드: ] [")
    print("  ★자세입력(S-curve): 7 = 각도 6개 입력 → 그 자세로 이동")
    print("  그리퍼(MG90): y=+2°  h=-2°        ESC=종료")
    print("  ★캘리브: k=IK각입력  j=샘플저장  p=회귀실행  l=샘플수")
    print(f"   CSV: {CALIB_CSV}\n")

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_status = 0.0
        next_hold = 0.0

        while True:
            loop_start = time.monotonic()

            # 1) 입력 처리
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    raise KeyboardInterrupt

                # ── 캘리브 키 ──
                if ch == "k":  # IK각 6개 입력 (모터 정지 상태로 입력받음)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                    last_seen.clear()
                    if pending_ik is not None:
                        # j로 저장 안 한 이전 입력이 남아 있으면 조용히 덮지 말고 알린다.
                        sys.stdout.write(
                            "\n[캘리브] ⚠ 저장 안 한 이전 IK각이 있었음 → 새 입력으로 교체\n"
                        )
                        sys.stdout.flush()
                    ik = prompt_ik_angles(fd, old_attr)
                    if ik is not None:
                        pending_ik = ik
                    next_status = 0.0
                    continue
                if ch == "j":  # 현재 실측각으로 샘플 저장
                    if pending_ik is None:
                        sys.stdout.write("\n[캘리브] 먼저 k로 IK각 입력하세요\n")
                        sys.stdout.flush()
                    else:
                        real = measured.snapshot()
                        missing = [
                            m for m in range(1, MOTOR_COUNT + 1) if m not in real
                        ]
                        if missing:
                            sys.stdout.write(
                                f"\n[캘리브] 실측 누락 M{missing} — 잠깐 기다렸다 다시 j\n"
                            )
                            sys.stdout.flush()
                        else:
                            sno = save_sample(pending_ik, real)
                            line = "  ".join(
                                f"M{m}:{pending_ik[m]:.1f}/{real[m]:.1f}"
                                for m in range(1, 7)
                            )
                            sys.stdout.write(
                                f"\n[캘리브] 샘플#{sno} 저장  (IK/실측)  {line}\n"
                            )
                            sys.stdout.flush()
                            pending_ik = None
                    next_status = 0.0
                    continue
                if ch == "p":  # 회귀 실행
                    run_regression()
                    next_status = 0.0
                    continue
                if ch == "l":  # 샘플 수
                    nsamp, per = count_samples()
                    sys.stdout.write(
                        f"\n[캘리브] 샘플 {nsamp}개  모터별 점수: "
                        + " ".join(f"M{m}={per.get(m,0)}" for m in range(1, 7))
                        + "\n"
                    )
                    sys.stdout.flush()
                    next_status = 0.0
                    continue

                # ── 기존 키 ──
                if ch in MOVE_KEYS:
                    last_seen[ch] = loop_start
                    ai = KEY_AXIS[ch]
                    for mid, _s in AXES[ai]["members"]:
                        homing.discard(mid)
                        home_start.pop(mid, None)
                    pose_idx = None
                    pose_start.clear()
                elif ch == "y":
                    gripper_angle = send_gripper(bus, gripper_angle + GRIPPER_STEP)
                elif ch == "h":
                    gripper_angle = send_gripper(bus, gripper_angle - GRIPPER_STEP)
                elif ch == "]":
                    p = min(
                        AXES[MIRROR_AXIS_IDX]["preload"] + PRELOAD_STEP, PRELOAD_MAX
                    )
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch == "[":
                    p = max(
                        AXES[MIRROR_AXIS_IDX]["preload"] - PRELOAD_STEP, -PRELOAD_MAX
                    )
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch == "9":
                    pose_idx = 0
                    pose_goal = CAMERA_POSE
                    pose_start.clear()
                    homing.clear()
                    home_start.clear()
                    sys.stdout.write(
                        "\n[자세] 카메라 자세 9로 순차 이동 (M1→M4→M2·3→M5→M6)\n"
                    )
                    sys.stdout.flush()
                elif ch == "8":
                    pose_idx = 0
                    pose_goal = CAMERA_POSE2
                    pose_start.clear()
                    homing.clear()
                    home_start.clear()
                    sys.stdout.write(
                        "\n[자세] 카메라 자세 8로 순차 이동 (M1→M4→M2·3→M5→M6)\n"
                    )
                    sys.stdout.flush()
                elif ch == "7":  # ★ 임의 각도 6개 입력 → 그 자세로 S-curve 이동
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                    last_seen.clear()
                    pose = prompt_pose_angles(fd, old_attr)
                    next_status = 0.0
                    if pose is not None:
                        bad = check_pose_limits(pose)
                        if bad:
                            # 한계 밖이면 이동 거부 — 하드스톱에 박는 걸 막는다.
                            sys.stdout.write(
                                "\n[자세입력] ❌ 한계 밖이라 이동 안 함: "
                                + "  ".join(
                                    f"M{m}={v}(허용 {lo:.0f}~{hi:.0f})"
                                    for m, v, (lo, hi) in bad
                                )
                                + "\n"
                            )
                            sys.stdout.flush()
                        else:
                            pose_goal = pose  # 8·9와 같은 자세 시스템 재사용
                            pose_idx = 0
                            pose_start.clear()
                            homing.clear()
                            home_start.clear()
                    continue
                elif ch in CALIB_KEYS:
                    ai = CALIB_KEYS[ch]
                    msgs = []
                    for mid, _sign in AXES[ai]["members"]:
                        if mid > MOTOR_COUNT:
                            continue
                        send_set_middle(bus, mid)
                        target[mid] = None
                        measured.clear(mid)
                        homing.discard(mid)
                        home_start.pop(mid, None)
                        msgs.append(f"M{mid}->180")
                    axis_speed[ai] = 0.0
                    pose_idx = None
                    pose_start.clear()
                    sys.stdout.write("\n[영점] " + "  ".join(msgs) + "\n")
                    sys.stdout.flush()
                elif ch == "0":
                    t0 = time.monotonic()
                    for hm in range(1, MOTOR_COUNT + 1):
                        cur = target[hm] if target[hm] is not None else measured.get(hm)
                        if cur is None:
                            cur = DEFAULT_ANGLE
                        dur = max(abs(HOME_ANGLE - cur) / HOME_SPEED, HOME_MIN_DURATION)
                        home_start[hm] = (cur, t0, dur)
                        homing.add(hm)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                    pose_idx = None
                    pose_start.clear()
                elif ch in "123456":
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        cur = target[hm] if target[hm] is not None else measured.get(hm)
                        if cur is None:
                            cur = DEFAULT_ANGLE
                        dur = max(abs(HOME_ANGLE - cur) / HOME_SPEED, HOME_MIN_DURATION)
                        home_start[hm] = (cur, time.monotonic(), dur)
                        homing.add(hm)
                        pose_idx = None
                        pose_start.clear()

            now = loop_start

            # 2) 축별 수동 이동
            for ai, ax in enumerate(AXES):
                fwd = (
                    ax["fwd"] in last_seen
                    and (now - last_seen[ax["fwd"]]) < HOLD_TIMEOUT
                )
                back = (
                    ax["back"] in last_seen
                    and (now - last_seen[ax["back"]]) < HOLD_TIMEOUT
                )

                direction = 0
                if fwd and not back:
                    direction = +1
                elif back and not fwd:
                    direction = -1

                if direction == 0:
                    axis_speed[ai] = 0.0
                else:
                    if direction != axis_prev_dir[ai]:
                        axis_speed[ai] = 0.0
                    axis_speed[ai] = min(axis_speed[ai] + RAMP_RATE * DT, MAX_SPEED)
                    delta = direction * axis_speed[ai] * DT * ax["gain"]
                    for mid, sign in ax["members"]:
                        if target[mid] is None:
                            a = measured.get(mid)
                            if a is None:
                                continue
                            target[mid] = a
                        target[mid] += sign * delta
                        # 소프트 한계에서 멈춤(clamp) — 하드스톱 충돌 방지
                        target[mid] = clamp_limit(mid, target[mid])
                        phys = target[mid] + sign * ax["preload"]
                        send_angle(bus, mid, phys, move_ms=STREAM_MOVE_MS)

                axis_prev_dir[ai] = direction

            # 2.3) S-curve 홈복귀
            for mid in list(homing):
                if mid not in home_start:
                    homing.discard(mid)
                    continue
                sa, st, dur = home_start[mid]
                tt = min((now - st) / dur, 1.0)
                target[mid] = sa + (HOME_ANGLE - sa) * smoothstep(tt)
                send_angle(bus, mid, target[mid], move_ms=STREAM_MOVE_MS)
                if tt >= 1.0:
                    target[mid] = HOME_ANGLE
                    homing.discard(mid)
                    home_start.pop(mid, None)

            # 2.4) S-curve 카메라 자세
            posing_motors = set()
            if pose_idx is not None:
                members = AXES[POSE_SEQ[pose_idx]]["members"]
                if all(mid not in pose_start for mid, _s in members):
                    seeds = {}
                    ready = True
                    for mid, _s in members:
                        cur = (
                            target[mid]
                            if target[mid] is not None
                            else measured.get(mid)
                        )
                        if cur is None:
                            ready = False
                            break
                        seeds[mid] = cur
                    if ready:
                        maxdist = max(
                            abs(pose_goal[mid] - seeds[mid]) for mid, _s in members
                        )
                        dur = max(maxdist / CAMERA_SPEED, CAMERA_MIN_DURATION)
                        for mid, _s in members:
                            pose_start[mid] = (seeds[mid], now, dur, pose_goal[mid])
                            target[mid] = seeds[mid]
                axis_done = True
                for mid, _s in members:
                    posing_motors.add(mid)
                    if mid not in pose_start:
                        axis_done = False
                        continue
                    sa, st, dur, goal = pose_start[mid]
                    tt = min((now - st) / dur, 1.0)
                    target[mid] = sa + (goal - sa) * smoothstep(tt)
                    send_angle(bus, mid, target[mid], move_ms=STREAM_MOVE_MS)
                    if tt < 1.0:
                        axis_done = False
                if axis_done:
                    for mid, _s in members:
                        target[mid] = pose_goal[mid]
                        pose_start.pop(mid, None)
                    pose_idx += 1
                    if pose_idx >= len(POSE_SEQ):
                        pose_idx = None
                        sys.stdout.write("\n[자세] 자세 도착 완료\n")
                        sys.stdout.flush()

            # 2.5) 프리로드 축 정지 중 재송신
            if now >= next_hold:
                next_hold = now + 0.3
                for ai, ax in enumerate(AXES):
                    if ax["preload"] != 0.0 and axis_prev_dir[ai] == 0:
                        for mid, sign in ax["members"]:
                            if (
                                target[mid] is not None
                                and mid not in homing
                                and mid not in posing_motors
                            ):
                                phys = target[mid] + sign * ax["preload"]
                                send_angle(bus, mid, phys, move_ms=200)

            # 3) 상태 한 줄 — 목표/실측 + 그리퍼 (+캘리브 대기표시)
            if now >= next_status:
                next_status = now + 0.2
                parts = []
                for m in range(1, MOTOR_COUNT + 1):
                    if m in posing_motors:
                        arrow = "P"
                    elif m in homing:
                        arrow = "H"
                    else:
                        ai, sign = MOTOR_AXIS.get(m, (None, 1))
                        d = axis_prev_dir[ai] * sign if ai is not None else 0
                        arrow = "+" if d > 0 else ("-" if d < 0 else " ")
                    tgt = target[m]
                    act = measured.get(m)
                    tgt_s = f"{tgt:.1f}" if tgt is not None else "--"
                    act_s = f"{act:.1f}" if act is not None else "--"
                    parts.append(f"M{m}{arrow}{tgt_s}/{act_s}")
                pre = AXES[MIRROR_AXIS_IDX]["preload"]
                cal = " [k입력대기:j로저장]" if pending_ik is not None else ""
                sys.stdout.write(
                    "\r"
                    + "  ".join(parts)
                    + f" |pre{pre:+.1f} grip{gripper_angle}{cal}  "
                )
                sys.stdout.flush()

            remain = DT - (time.monotonic() - loop_start)
            if remain > 0:
                time.sleep(remain)

    except KeyboardInterrupt:
        print("\n종료.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        notifier.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()
