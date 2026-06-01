"""
웨이포인트 티칭 도구 — MPU6050 또는 키보드

사용법:
  python mpu_teaching.py --port COM3        (MPU 모드)
  python mpu_teaching.py --keyboard         (키보드 모드)

조작 (공통):
  s     웨이포인트 저장
  1-4   checkpoint 태그 (SCAN / PRE_GRASP / GRASP / PLACE)
  g     그리퍼 토글
  r     홈 자세 리셋 (MPU 재캘리브레이션)
  u     마지막 웨이포인트 삭제
  p     궤적 시뮬레이션 재생
  q     종료 + CSV 저장

키보드 모드 추가 조작:
  ← →   J1 ±5°  (shift: ±1°)
  ↑ ↓   J2 ±5°
  w/e   J3 ±5°
  a/d   J4 ±5°
  z/x   J5 ±5°
"""
import cv2
import numpy as np
import csv
import os
import math
import argparse
import time

from config import HOME, JOINTS
import ik

TRAJECTORY_DIR = "trajectories"
os.makedirs(TRAJECTORY_DIR, exist_ok=True)

CHECKPOINT_NAMES = {
    ord("1"): "SCAN",
    ord("2"): "PRE_GRASP",
    ord("3"): "GRASP",
    ord("4"): "PLACE",
}

CHECKPOINT_COLORS = {
    "SCAN": (255, 255, 0),
    "PRE_GRASP": (0, 165, 255),
    "GRASP": (0, 0, 255),
    "PLACE": (0, 255, 0),
}

MPU_SCALE = {
    "J1": 2.5,
    "J2": 2.0,
    "J3": 2.0,
    "J4": 2.0,
    "J5": 1.5,
}

DEAD_ZONE = 1.5
SMOOTH_ALPHA = 0.3
JUMP_WARN_THRESHOLD = 30.0

# ═══════════════════════════════════════════════════════════════
# FK 좌표 계산
# ═══════════════════════════════════════════════════════════════

def gripper_xyz(robot):
    r, z = ik.forward(robot["J2"], robot["J3"], robot["J4"])
    j1_rad = math.radians(robot["J1"])
    x = r * math.sin(j1_rad)
    y = r * math.cos(j1_rad)
    return x, y, z


def check_reachable(robot):
    r, z = ik.forward(robot["J2"], robot["J3"], robot["J4"])
    return ik.reachable(r, z)


def check_jump(robot, last_wp):
    if last_wp is None:
        return False, 0.0
    max_delta = 0.0
    for j in ["J1", "J2", "J3", "J4", "J5"]:
        delta = abs(robot[j] - last_wp[j])
        max_delta = max(max_delta, delta)
    return max_delta > JUMP_WARN_THRESHOLD, max_delta


# ═══════════════════════════════════════════════════════════════
# 시각화
# ═══════════════════════════════════════════════════════════════

LINK_LENGTHS = [24, 148, 128]
VIEW_AZIM = math.radians(35)
VIEW_ELEV = math.radians(25)
TOPDOWN_CENTER = (820, 280)
TOPDOWN_SCALE = 0.7


def _project_3d(pt, center=(270, 380)):
    x, y, z = pt
    ca, sa = math.cos(VIEW_AZIM), math.sin(VIEW_AZIM)
    rx = x * ca + z * sa
    rz = -x * sa + z * ca
    ce, se = math.cos(VIEW_ELEV), math.sin(VIEW_ELEV)
    ry = y * ce - rz * se
    return (int(center[0] + rx), int(center[1] - ry))


def _topdown(x_mm, y_mm):
    px = int(TOPDOWN_CENTER[0] + x_mm * TOPDOWN_SCALE)
    py = int(TOPDOWN_CENTER[1] - y_mm * TOPDOWN_SCALE)
    return (px, py)


def _draw_3d_arm(canvas, robot):
    j1 = math.radians(robot["J1"])
    a1 = -(math.pi / 2) + (robot["J2"] - 75) * math.pi / 150
    a2 = a1 + (robot["J3"] - 90) * math.pi / 135
    a3 = a2 + (robot["J4"] - 90) * math.pi / 120

    r, h = 0.0, 0.0
    joints_rh = [(0.0, 0.0)]
    for length, angle in zip(LINK_LENGTHS, [a1, a2, a3]):
        r += length * math.cos(angle)
        h += -length * math.sin(angle)
        joints_rh.append((r, h))

    joints_3d = []
    for rad, height in joints_rh:
        joints_3d.append((rad * math.sin(j1), height, rad * math.cos(j1)))

    grip_angles = [a3 + 0.4, a3 - 0.4]
    for ga in grip_angles:
        gr = joints_rh[3][0] + 20 * math.cos(ga)
        gh = joints_rh[3][1] + (-20 * math.sin(ga))
        joints_3d.append((gr * math.sin(j1), gh, gr * math.cos(j1)))

    pts = [_project_3d(j) for j in joints_3d]

    n_ellipse = 32
    base_ring = []
    for i in range(n_ellipse):
        t = 2 * math.pi * i / n_ellipse
        base_ring.append(_project_3d((30 * math.cos(t), -5, 30 * math.sin(t))))
    cv2.fillPoly(canvas, [np.array(base_ring)], (60, 60, 60))

    colors = [(0, 200, 255), (0, 255, 200), (200, 255, 0)]
    for i in range(3):
        cv2.line(canvas, pts[i], pts[i + 1], colors[i], 7)
    cv2.line(canvas, pts[3], pts[4], (255, 100, 100), 3)
    cv2.line(canvas, pts[3], pts[5], (255, 100, 100), 3)

    cv2.circle(canvas, pts[0], 7, (255, 255, 255), -1)
    for i in range(1, 4):
        cv2.circle(canvas, pts[i], 6, (0, 0, 255), -1)

    labels = ["Base", "Shoulder", "Elbow", "Wrist"]
    for i, label in enumerate(labels):
        cv2.putText(canvas, label, (pts[i][0] + 8, pts[i][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)


def _draw_topdown(canvas, robot, waypoints):
    cx, cy = TOPDOWN_CENTER
    s = TOPDOWN_SCALE

    cv2.putText(canvas, "TOP VIEW (mm)", (cx - 80, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    max_reach = (ik.IK_L2 + ik.IK_L3) * s
    cv2.circle(canvas, (cx, cy), int(max_reach), (40, 40, 40), 1)
    cv2.circle(canvas, (cx, cy), int(max_reach * 0.3), (30, 30, 30), 1)

    cv2.circle(canvas, (cx, cy), 8, (100, 100, 100), -1)
    cv2.putText(canvas, "BASE", (cx + 12, cy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    for axis_len in [100, 200, 300]:
        pt = _topdown(0, axis_len)
        cv2.line(canvas, (cx, cy), pt, (30, 30, 30), 1)
        cv2.putText(canvas, f"{axis_len}", (pt[0] + 3, pt[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60, 60, 60), 1)

    bin_positions = {"bolt": 45.0, "nut": 135.0, "washer": 180.0}
    for name, j1_deg in bin_positions.items():
        j1_rad = math.radians(j1_deg)
        bx = 250 * math.sin(j1_rad)
        by = 250 * math.cos(j1_rad)
        bp = _topdown(bx, by)
        cv2.rectangle(canvas, (bp[0] - 15, bp[1] - 10),
                      (bp[0] + 15, bp[1] + 10), (0, 100, 0), -1)
        cv2.putText(canvas, name, (bp[0] - 12, bp[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

    gx, gy, gz = gripper_xyz(robot)
    gp = _topdown(gx, gy)
    cv2.circle(canvas, gp, 6, (0, 255, 255), -1)
    cv2.line(canvas, (cx, cy), gp, (0, 100, 100), 1)

    # 저장된 웨이포인트 궤적선
    if len(waypoints) > 0:
        wp_pts = []
        for wp in waypoints:
            wx, wy, wz = gripper_xyz(wp)
            wp_pts.append(_topdown(wx, wy))

        for i in range(len(wp_pts) - 1):
            cv2.line(canvas, wp_pts[i], wp_pts[i + 1], (100, 100, 0), 1)

        for i, (pt, wp) in enumerate(zip(wp_pts, waypoints)):
            cp = wp.get("checkpoint", "")
            color = CHECKPOINT_COLORS.get(cp, (180, 180, 180))
            cv2.circle(canvas, pt, 4, color, -1)
            cv2.putText(canvas, f"{i + 1}", (pt[0] + 5, pt[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)


def draw_teaching_view(robot, waypoints, gripper_open, next_checkpoint,
                       is_keyboard, warnings):
    canvas = np.zeros((550, 1100, 3), dtype=np.uint8)

    cv2.line(canvas, (550, 0), (550, 550), (50, 50, 50), 1)

    _draw_3d_arm(canvas, robot)
    _draw_topdown(canvas, robot, waypoints)

    # ── 좌측 패널: 관절값 + FK 좌표 + 상태 ──

    # 관절값
    for i, (joint, val) in enumerate(robot.items()):
        near_limit = (val <= JOINTS[joint]["min"] + 5 or
                      val >= JOINTS[joint]["max"] - 5)
        color = (0, 0, 255) if near_limit else (0, 255, 0)
        text = f"{joint}: {val:.1f}"
        if near_limit:
            text += " !"
        cv2.putText(canvas, text, (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # FK 좌표
    gx, gy, gz = gripper_xyz(robot)
    cv2.putText(canvas, f"Gripper: x={gx:.0f} y={gy:.0f} z={gz:.0f} mm",
                (10, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

    r_val, z_val = ik.forward(robot["J2"], robot["J3"], robot["J4"])
    cv2.putText(canvas, f"Reach: r={r_val:.0f} z={z_val:.0f} mm",
                (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

    # 도달 가능 여부
    reachable = check_reachable(robot)
    reach_text = "REACHABLE" if reachable else "OUT OF REACH"
    reach_color = (0, 255, 0) if reachable else (0, 0, 255)
    cv2.putText(canvas, reach_text, (10, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, reach_color, 1)

    # 그리퍼 상태
    grip_text = "GRIP: OPEN" if gripper_open else "GRIP: CLOSE"
    grip_color = (0, 255, 0) if gripper_open else (0, 0, 255)
    cv2.putText(canvas, grip_text, (10, 215),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, grip_color, 1)

    # checkpoint
    if next_checkpoint:
        cp_color = CHECKPOINT_COLORS.get(next_checkpoint, (0, 255, 255))
        cv2.putText(canvas, f"NEXT CP: {next_checkpoint}", (10, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cp_color, 1)

    # 경고
    for i, warn in enumerate(warnings):
        cv2.putText(canvas, warn, (10, 270 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    # 모드
    if is_keyboard:
        cv2.putText(canvas, "KEYBOARD", (450, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    else:
        cv2.putText(canvas, "MPU", (450, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ── 웨이포인트 목록 (좌측 하단) ──

    wp_y = 350
    cv2.putText(canvas, f"Waypoints: {len(waypoints)}", (10, wp_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    start = max(0, len(waypoints) - 8)
    for i, wp in enumerate(waypoints[start:]):
        idx = start + i
        cp = wp.get("checkpoint", "")
        color = CHECKPOINT_COLORS.get(cp, (150, 150, 150))
        cp_text = f" [{cp}]" if cp else ""
        wx, wy, wz = gripper_xyz(wp)
        text = f"#{idx + 1}: ({wx:.0f},{wy:.0f},{wz:.0f}){cp_text}"
        cv2.putText(canvas, text, (10, wp_y + 22 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

    # ── 조작법 (좌측 최하단) ──
    helps = [
        "s:save  1-4:checkpoint  g:gripper  p:play",
        "u:undo  r:reset  q:quit+save",
    ]
    for i, h in enumerate(helps):
        cv2.putText(canvas, h, (10, 520 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (80, 80, 80), 1)

    return canvas


# ═══════════════════════════════════════════════════════════════
# 궤적 시뮬레이션 재생
# ═══════════════════════════════════════════════════════════════

def play_trajectory(waypoints):
    if len(waypoints) < 2:
        print("[PLAY] Need at least 2 waypoints")
        return

    print(f"[PLAY] Simulating {len(waypoints)} waypoints...")
    steps_between = 30

    for wi in range(len(waypoints) - 1):
        wp_start = waypoints[wi]
        wp_end = waypoints[wi + 1]

        for step in range(steps_between + 1):
            t = step / steps_between
            t_smooth = t * t * (3.0 - 2.0 * t)

            interp = {}
            for j in ["J1", "J2", "J3", "J4", "J5"]:
                interp[j] = wp_start[j] + (wp_end[j] - wp_start[j]) * t_smooth

            canvas = draw_teaching_view(
                interp, waypoints,
                wp_end.get("gripper", "open") == "open",
                None, False, []
            )

            cp = wp_end.get("checkpoint", "")
            cv2.putText(canvas, f"PLAY: WP#{wi + 1} -> #{wi + 2}",
                        (400, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 0), 1)
            if cp:
                cv2.putText(canvas, f"-> [{cp}]", (400, 540),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            CHECKPOINT_COLORS.get(cp, (255, 255, 0)), 1)

            cv2.imshow("Teaching", canvas)
            if cv2.waitKey(30) & 0xFF == ord("q"):
                print("[PLAY] Stopped")
                return

    print("[PLAY] Done")


# ═══════════════════════════════════════════════════════════════
# MPU Serial 입력 (스무딩 + 데드존)
# ═══════════════════════════════════════════════════════════════

class MPUReader:
    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.baseline = None
        self.prev = dict(HOME)
        self.smooth = [0.0] * 9
        self.drift_rate = 0.0
        self.t0 = 0.0
        self.prev_delta = None
        time.sleep(2)
        self._flush()

    def _flush(self):
        while self.ser.in_waiting:
            self.ser.readline()

    def calibrate(self, samples=50):
        print("[CAL] Hold arm still for 3 seconds...")
        time.sleep(1)
        readings = []
        timestamps = []
        while len(readings) < samples:
            raw = self._read_raw()
            if raw is not None:
                readings.append(raw)
                timestamps.append(time.time())
        self.baseline = np.mean(readings, axis=0)

        # yaw 드리프트 속도 측정 (앞 10개 vs 뒤 10개 평균 비교)
        self.drift_rate = 0.0
        if len(readings) > 20:
            dt = timestamps[-1] - timestamps[0]
            if dt > 0.1:
                yaw_start = np.mean([r[0] for r in readings[:10]])
                yaw_end = np.mean([r[0] for r in readings[-10:]])
                self.drift_rate = (yaw_end - yaw_start) / dt

        self.smooth = [0.0] * 9
        self.prev = dict(HOME)
        self.prev_delta = None
        self.t0 = time.time()
        print(f"[CAL] Done. Yaw drift: {self.drift_rate:.3f} deg/s")

    def _read_raw(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line or line.startswith("["):
                return None
            vals = [float(v) for v in line.split(",")]
            if len(vals) != 9:
                return None
            return vals
        except (ValueError, UnicodeDecodeError):
            return None

    def read(self):
        raw = self._read_raw()
        if raw is None:
            return self.prev
        if self.baseline is None:
            return self.prev

        delta_raw = np.array(raw) - self.baseline

        # yaw 드리프트 보상
        elapsed = time.time() - self.t0
        delta_raw[0] -= self.drift_rate * elapsed

        # 적응형 스무딩: 정지 시 강하게, 이동 시 약하게
        alpha = SMOOTH_ALPHA
        if self.prev_delta is not None:
            motion = np.linalg.norm(delta_raw - self.prev_delta)
            if motion < 2.0:
                alpha = 0.05
            elif motion < 8.0:
                alpha = 0.2
            else:
                alpha = 0.5
        self.prev_delta = delta_raw.copy()

        for i in range(9):
            if abs(delta_raw[i] - self.smooth[i]) < DEAD_ZONE:
                delta_raw[i] = self.smooth[i]
            self.smooth[i] += alpha * (delta_raw[i] - self.smooth[i])

        d = self.smooth

        j1 = HOME["J1"] + d[0] * MPU_SCALE["J1"]
        j2 = HOME["J2"] + d[1] * MPU_SCALE["J2"]
        j3 = HOME["J3"] + (d[4] - d[1]) * MPU_SCALE["J3"]
        j4 = HOME["J4"] + (d[7] - d[4]) * MPU_SCALE["J4"]
        j5 = HOME["J5"] + d[8] * MPU_SCALE["J5"]

        robot = {
            "J1": float(np.clip(j1, JOINTS["J1"]["min"], JOINTS["J1"]["max"])),
            "J2": float(np.clip(j2, JOINTS["J2"]["min"], JOINTS["J2"]["max"])),
            "J3": float(np.clip(j3, JOINTS["J3"]["min"], JOINTS["J3"]["max"])),
            "J4": float(np.clip(j4, JOINTS["J4"]["min"], JOINTS["J4"]["max"])),
            "J5": float(np.clip(j5, JOINTS["J5"]["min"], JOINTS["J5"]["max"])),
        }
        self.prev = robot
        return robot

    def close(self):
        self.ser.close()


# ═══════════════════════════════════════════════════════════════
# 메인 루프
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Waypoint Teaching Tool")
    parser.add_argument("--port", type=str, help="Nano Serial port (e.g., COM3)")
    parser.add_argument("--keyboard", action="store_true", help="Keyboard-only mode")
    args = parser.parse_args()

    if not args.keyboard and not args.port:
        print("--port COM3 or --keyboard required")
        return

    mpu = None
    if args.port:
        mpu = MPUReader(args.port)
        mpu.calibrate()

    robot = dict(HOME)
    waypoints = []
    gripper_open = True
    next_checkpoint = None
    step = 5.0

    print("[TEACH] Waypoint Teaching Tool")
    print("[TEACH] s:save 1-4:checkpoint g:gripper u:undo r:reset p:play q:quit")

    while True:
        if mpu:
            robot = mpu.read()

        # 경고 계산
        warnings = []
        last_wp = waypoints[-1] if waypoints else None
        has_jump, jump_val = check_jump(robot, last_wp)
        if has_jump:
            warnings.append(f"JUMP WARNING: {jump_val:.0f} deg from last WP")
        if not check_reachable(robot):
            warnings.append("OUT OF REACH")
        if mpu:
            elapsed = time.time() - mpu.t0
            if elapsed > 45:
                warnings.append(f"DRIFT {elapsed:.0f}s — press r to recal")

        canvas = draw_teaching_view(
            robot, waypoints, gripper_open,
            next_checkpoint, args.keyboard, warnings
        )
        cv2.imshow("Teaching", canvas)
        key = cv2.waitKey(20) & 0xFF

        if key == 255:
            continue

        # 키보드 관절 제어
        if args.keyboard:
            d = step
            if key == 81:    # left
                robot["J1"] = max(JOINTS["J1"]["min"], robot["J1"] - d)
            elif key == 83:  # right
                robot["J1"] = min(JOINTS["J1"]["max"], robot["J1"] + d)
            elif key == 82:  # up
                robot["J2"] = max(JOINTS["J2"]["min"], robot["J2"] - d)
            elif key == 84:  # down
                robot["J2"] = min(JOINTS["J2"]["max"], robot["J2"] + d)
            elif key == ord("w"):
                robot["J3"] = max(JOINTS["J3"]["min"], robot["J3"] - d)
            elif key == ord("e"):
                robot["J3"] = min(JOINTS["J3"]["max"], robot["J3"] + d)
            elif key == ord("a"):
                robot["J4"] = max(JOINTS["J4"]["min"], robot["J4"] - d)
            elif key == ord("d"):
                robot["J4"] = min(JOINTS["J4"]["max"], robot["J4"] + d)
            elif key == ord("z"):
                robot["J5"] = max(JOINTS["J5"]["min"], robot["J5"] - d)
            elif key == ord("x"):
                robot["J5"] = min(JOINTS["J5"]["max"], robot["J5"] + d)

        # 웨이포인트 저장 (검증 포함)
        if key == ord("s"):
            if not check_reachable(robot):
                print("[WARN] Out of reach — save anyway? (press s again)")
            else:
                wp = dict(robot)
                wp["gripper"] = "open" if gripper_open else "close"
                wp["checkpoint"] = next_checkpoint or ""

                gx, gy, gz = gripper_xyz(robot)
                cp_msg = f" [{next_checkpoint}]" if next_checkpoint else ""

                if has_jump:
                    print(f"[WARN] {jump_val:.0f} deg jump from last WP")

                waypoints.append(wp)
                print(f"[SAVE] #{len(waypoints)}{cp_msg} "
                      f"pos=({gx:.0f},{gy:.0f},{gz:.0f})mm")
                next_checkpoint = None

        # checkpoint 태그
        elif key in CHECKPOINT_NAMES:
            next_checkpoint = CHECKPOINT_NAMES[key]
            print(f"[CP] Next waypoint -> {next_checkpoint}")

        # 그리퍼 토글
        elif key == ord("g"):
            gripper_open = not gripper_open
            print(f"[GRIP] {'OPEN' if gripper_open else 'CLOSE'}")

        # 홈 리셋 / MPU 재캘리브레이션
        elif key == ord("r"):
            robot = dict(HOME)
            if mpu:
                mpu.calibrate()
            print("[RESET] Home")

        # 마지막 웨이포인트 삭제
        elif key == ord("u"):
            if waypoints:
                waypoints.pop()
                print(f"[UNDO] Removed. {len(waypoints)} remaining")
            else:
                print("[UNDO] Nothing to undo")

        # 궤적 시뮬레이션 재생
        elif key == ord("p"):
            play_trajectory(waypoints)

        # 종료 + 저장
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()
    if mpu:
        mpu.close()

    if not waypoints:
        print("[EXIT] No waypoints saved")
        return

    name = input("[SAVE] Trajectory name (e.g., master_cycle): ").strip()
    if not name:
        name = f"master_{int(time.time())}"
    fname = f"{name}.csv"
    fpath = os.path.join(TRAJECTORY_DIR, fname)

    fields = ["seq", "J1", "J2", "J3", "J4", "J5", "gripper", "checkpoint"]
    with open(fpath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for i, wp in enumerate(waypoints):
            row = {"seq": i + 1}
            row.update(wp)
            writer.writerow(row)

    print(f"[DONE] Saved: {fpath} ({len(waypoints)} waypoints)")


if __name__ == "__main__":
    main()
