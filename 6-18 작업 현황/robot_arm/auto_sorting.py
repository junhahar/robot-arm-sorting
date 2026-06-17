"""
마스터 궤적 재생 + Checkpoint AI 보정

웨이포인트 CSV를 순서대로 재생하며, checkpoint 지점에서
카메라/ToF 기반 실시간 보정을 수행한다.

Checkpoint:
  SCAN      — YOLO 검출 → 물체 오프셋 계산
  PRE_GRASP — Visual Servoing → 미세 정렬
  GRASP     — ToF 하강 → 그리퍼 닫기
  PLACE     — ArUco → bin 위치 보정 → 그리퍼 열기

실행:
  python auto_sorting.py --trajectory master_cycle.csv
  python auto_sorting.py --trajectory master_cycle.csv --dry-run
"""
import os
import csv
import time
import math
import argparse

from config import (
    HOME, CARRY,
    GRASP_DIST, PLACE_DIST, DESCEND_STEP_MM, LIFT_HEIGHT,
    MAX_PLACE_DEPTH, MAX_DESCEND_DEPTH,
    GRIP_CLOSE_WAIT,
    ARUCO_DICT, BIN_ARUCO,
    CAMERA_ID, CAM_FX, CAM_FY,
    CAM_OFFSET_X, CAM_OFFSET_Y,
    GRIPPER_PX, GRIPPER_PY, VS_GAIN, VS_TOLERANCE, VS_MAX_ITER,
    YOLO_CONF, STABLE_FRAMES,
    TRAJECTORY_DIR,
    MOVE_DURATION, MOVE_DURATION_FAST,
)
from servo_driver import ServoDriver
import ik
import s_curve


# ═══════════════════════════════════════════════════════════════
# 설정
# ═══════════════════════════════════════════════════════════════

CORRECTION_LIMIT = 15.0  # 보정 최대 범위 (degrees)
DEFAULT_TRAJECTORY = "master_cycle.csv"


# ═══════════════════════════════════════════════════════════════
# Trajectory I/O
# ═══════════════════════════════════════════════════════════════

def load_trajectory(name):
    path = os.path.join(TRAJECTORY_DIR, name)
    if not os.path.exists(path):
        return None
    with open(path, "r") as f:
        rows = []
        for row in csv.DictReader(f):
            parsed = {}
            for k, v in row.items():
                if k in ("gripper", "checkpoint"):
                    parsed[k] = v.strip()
                elif k == "seq":
                    parsed[k] = int(v)
                else:
                    parsed[k] = float(v)
            rows.append(parsed)
        return rows


def traj_to_pose(row):
    return {j: row[j] for j in ["J1", "J2", "J3", "J4", "J5"] if j in row}


# ═══════════════════════════════════════════════════════════════
# YOLO Detector
# ═══════════════════════════════════════════════════════════════

class YoloDetector:

    def __init__(self, model_path=None, dry_run=False):
        self.dry_run = dry_run
        self.model = None
        if not dry_run and model_path:
            self._load_model(model_path)

    def _load_model(self, path):
        try:
            from hailo_platform import HEF, VDevice
            self.hef = HEF(path)
            self.device = VDevice()
            self.model = self.device.configure(self.hef)
            print(f"[YOLO] Hailo model loaded: {path}")
        except ImportError:
            print("[YOLO] hailo_platform not installed — dry-run")
            self.dry_run = True
        except Exception as e:
            print(f"[YOLO] Model load failed: {e} — dry-run")
            self.dry_run = True

    def detect(self, frame):
        if self.dry_run:
            return []
        return []


# ═══════════════════════════════════════════════════════════════
# Camera
# ═══════════════════════════════════════════════════════════════

class Camera:

    def __init__(self, camera_id=CAMERA_ID, dry_run=False):
        self.dry_run = dry_run
        self.cap = None
        self.aruco_detector = None

        if not dry_run:
            import cv2
            import cv2.aruco as aruco
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                print(f"[CAM] Camera {camera_id} failed — dry-run")
                self.dry_run = True
            else:
                dictionary = aruco.getPredefinedDictionary(
                    getattr(aruco, ARUCO_DICT)
                )
                self.aruco_detector = aruco.ArucoDetector(
                    dictionary, aruco.DetectorParameters()
                )

    def read(self):
        if self.dry_run:
            return None
        import cv2
        ret, frame = self.cap.read()
        return frame if ret else None

    def pixel_to_mm(self, px, py, height_mm):
        from config import CAM_CX, CAM_CY
        x_mm = (px - CAM_CX) * height_mm / CAM_FX
        y_mm = (py - CAM_CY) * height_mm / CAM_FY
        return x_mm, y_mm

    def detect_aruco(self, frame):
        if self.aruco_detector is None:
            return {}
        import cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return {}
        result = {}
        for i, marker_id in enumerate(ids.flatten()):
            center = corners[i][0].mean(axis=0)
            result[int(marker_id)] = (float(center[0]), float(center[1]))
        return result

    def close(self):
        if self.cap:
            self.cap.release()


# ═══════════════════════════════════════════════════════════════
# 보정 유틸
# ═══════════════════════════════════════════════════════════════

def clamp_correction(base, corrected, limit=CORRECTION_LIMIT):
    delta = corrected - base
    if abs(delta) > limit:
        return base + limit * (1.0 if delta > 0 else -1.0)
    return corrected


def apply_offset(pose, offset_j1):
    adjusted = dict(pose)
    adjusted["J1"] = clamp_correction(pose["J1"], pose["J1"] + offset_j1)
    return adjusted


# ═══════════════════════════════════════════════════════════════
# Trajectory Player
# ═══════════════════════════════════════════════════════════════

class TrajectoryPlayer:

    def __init__(self, trajectory_name=DEFAULT_TRAJECTORY, dry_run=False):
        self.trajectory_name = trajectory_name
        self.servo = ServoDriver(dry_run=dry_run)
        self.detector = YoloDetector(dry_run=dry_run)
        self.camera = Camera(dry_run=dry_run)

        self.pose = dict(HOME)
        self.cycle_count = 0
        self.detected_class = None
        self.offset_j1 = 0.0
        self.offset_r = 0.0

    # ── Main Loop ────────────────────────────────────────────

    def run(self):
        print(f"[PLAY] Master trajectory: {self.trajectory_name}")
        print("[PLAY] Starting... (Ctrl+C to stop)")

        self.servo.home()
        self.pose = dict(HOME)

        try:
            while True:
                success = self._run_cycle()
                if not success:
                    print("[PLAY] Cycle failed — retrying after 3s")
                    time.sleep(3)
        except KeyboardInterrupt:
            print("\n[PLAY] Stopped")
        finally:
            self.servo.gripper(True)
            self.servo.home()
            self.camera.close()
            self.servo.close()

    def _run_cycle(self):
        traj = load_trajectory(self.trajectory_name)
        if not traj:
            print(f"[ERR] {self.trajectory_name} not found")
            return False

        self.offset_j1 = 0.0
        self.offset_r = 0.0
        self.detected_class = None

        for i, wp in enumerate(traj):
            target = traj_to_pose(wp)
            checkpoint = wp.get("checkpoint", "")
            gripper_state = wp.get("gripper", "open")

            # 보정 오프셋 적용 (SCAN에서 계산된 값)
            if self.offset_j1 != 0.0:
                target = apply_offset(target, self.offset_j1)

            # s-curve로 이동
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION
            )

            # 그리퍼 (checkpoint가 아닌 일반 웨이포인트에서만 자동 처리)
            if not checkpoint:
                if gripper_state == "close":
                    self.servo.gripper(False)
                else:
                    self.servo.gripper(True)

            # Checkpoint 보정
            if checkpoint == "SCAN":
                if not self._correct_scan():
                    return self._error_recovery()
            elif checkpoint == "PRE_GRASP":
                if not self._correct_pre_grasp():
                    return self._error_recovery()
            elif checkpoint == "GRASP":
                if not self._correct_grasp():
                    return self._error_recovery()
            elif checkpoint == "PLACE":
                if not self._correct_place():
                    return self._error_recovery()

            print(f"  WP#{i + 1}/{len(traj)} done"
                  + (f" [{checkpoint}]" if checkpoint else ""))

        self.cycle_count += 1
        print(f"[CYCLE] Complete #{self.cycle_count}")
        return True

    # ── CP1: SCAN ────────────────────────────────────────────

    def _correct_scan(self):
        print("[SCAN] Detecting objects...")
        stable_count = 0
        last_class = None

        for attempt in range(30):
            frame = self.camera.read()
            if frame is None:
                time.sleep(0.05)
                continue

            detections = self.detector.detect(frame)
            if not detections:
                stable_count = 0
                last_class = None
                time.sleep(0.05)
                continue

            best = max(detections, key=lambda d: d["conf"])
            if best["conf"] < YOLO_CONF:
                stable_count = 0
                continue

            if best["class"] == last_class:
                stable_count += 1
            else:
                stable_count = 1
                last_class = best["class"]

            if stable_count >= STABLE_FRAMES:
                tof = self.servo.read_tof()
                if tof is None or tof > 2000:
                    continue

                actual = self.servo.read_all()
                if not actual:
                    continue

                curr_r, curr_z = ik.forward(
                    actual["J2"], actual["J3"], actual["J4"]
                )

                px, py = best["cx"], best["cy"]
                x_cam, y_cam = self.camera.pixel_to_mm(px, py, tof)
                x_grip = x_cam + CAM_OFFSET_X
                y_grip = y_cam + CAM_OFFSET_Y

                raw_j1_offset = math.degrees(
                    math.atan2(x_grip, curr_r)
                )
                self.offset_j1 = max(-CORRECTION_LIMIT,
                                     min(CORRECTION_LIMIT, raw_j1_offset))
                self.offset_r = y_grip
                self.detected_class = best["class"]

                print(f"[SCAN] Found: {best['class']} "
                      f"(conf={best['conf']:.2f}) "
                      f"offset_j1={self.offset_j1:.1f}° "
                      f"offset_r={self.offset_r:.0f}mm")
                return True

            time.sleep(0.05)

        print("[SCAN] No object detected")
        return False

    # ── CP2: PRE_GRASP ───────────────────────────────────────

    def _correct_pre_grasp(self):
        print("[PRE_GRASP] Visual Servoing...")

        for i in range(VS_MAX_ITER):
            frame = self.camera.read()
            if frame is None:
                break

            detections = self.detector.detect(frame)
            if not detections:
                break

            best = max(detections, key=lambda d: d["conf"])
            dx = best["cx"] - GRIPPER_PX
            dy = best["cy"] - GRIPPER_PY

            if abs(dx) < VS_TOLERANCE and abs(dy) < VS_TOLERANCE:
                print(f"[PRE_GRASP] Converged ({i + 1} iterations)")
                return True

            tof = self.servo.read_tof()
            h = tof if tof and tof < 2000 else 200

            actual = self.servo.read_all()
            if actual:
                curr_r, curr_z = ik.forward(
                    actual["J2"], actual["J3"], actual["J4"]
                )
            else:
                curr_r, curr_z = 200.0, 0.0

            new_r = curr_r + dy * VS_GAIN * h / CAM_FY
            raw_j1 = self.pose["J1"] + math.degrees(
                math.atan2(dx * VS_GAIN * h / CAM_FX, curr_r)
            )
            new_j1 = clamp_correction(self.pose["J1"], raw_j1)

            result = ik.solve(new_r, curr_z)
            if result is None:
                continue
            j2, j3, j4 = result
            adj = {"J1": new_j1, "J2": j2, "J3": j3, "J4": j4,
                   "J5": self.pose["J5"]}
            self.pose = s_curve.execute(
                self.servo, self.pose, adj, MOVE_DURATION_FAST
            )

        print("[PRE_GRASP] VS done")
        return True

    # ── CP3: GRASP ───────────────────────────────────────────

    def _correct_grasp(self):
        print("[GRASP] ToF descent...")

        actual = self.servo.read_all()
        if actual:
            curr_r, curr_z = ik.forward(
                actual["J2"], actual["J3"], actual["J4"]
            )
        else:
            curr_r = 200.0
            curr_z = 0.0

        current_z = curr_z

        for step in range(int(MAX_DESCEND_DEPTH / DESCEND_STEP_MM)):
            current_z -= DESCEND_STEP_MM

            result = ik.solve(curr_r, current_z)
            if result is None:
                print("[GRASP] IK limit reached")
                break
            j2, j3, j4 = result
            target = dict(self.pose)
            target.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            tof = self.servo.read_tof()
            if tof is not None and tof <= GRASP_DIST:
                print(f"[GRASP] ToF={tof}mm — closing gripper")
                break

        self.servo.gripper(False)
        time.sleep(GRIP_CLOSE_WAIT)

        # 들어올리기
        lift_z = current_z + LIFT_HEIGHT
        result = ik.solve(curr_r, lift_z)
        if result:
            j2, j3, j4 = result
            lift = dict(self.pose)
            lift.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, lift, MOVE_DURATION_FAST
            )

        print(f"[GRASP] Picked: {self.detected_class}")
        return True

    # ── CP4: PLACE ───────────────────────────────────────────

    def _correct_place(self):
        print("[PLACE] ArUco correction...")

        cls = self.detected_class
        aruco_cfg = BIN_ARUCO.get(cls) if cls else None

        if aruco_cfg:
            for attempt in range(3):
                frame = self.camera.read()
                if frame is None:
                    continue

                markers = self.camera.detect_aruco(frame)
                target_id = aruco_cfg["id"]

                if target_id in markers:
                    px, py = markers[target_id]
                    tof = self.servo.read_tof()
                    h = tof if tof and tof < 2000 else 200

                    x_mm, y_mm = self.camera.pixel_to_mm(px, py, h)
                    bin_x = x_mm + aruco_cfg["offset_x"]

                    actual = self.servo.read_all()
                    if actual:
                        curr_r, _ = ik.forward(
                            actual["J2"], actual["J3"], actual["J4"]
                        )
                    else:
                        curr_r = 200.0

                    raw_j1 = self.pose["J1"] + math.degrees(
                        math.atan2(bin_x, curr_r)
                    )
                    adj_j1 = clamp_correction(self.pose["J1"], raw_j1)

                    if adj_j1 != self.pose["J1"]:
                        adj = dict(self.pose)
                        adj["J1"] = adj_j1
                        self.pose = s_curve.execute(
                            self.servo, self.pose, adj, MOVE_DURATION_FAST
                        )

                    print(f"[PLACE] ArUco #{target_id} found")
                    break

        # ToF 하강 + 놓기
        actual = self.servo.read_all()
        if actual:
            curr_r, curr_z = ik.forward(
                actual["J2"], actual["J3"], actual["J4"]
            )
        else:
            curr_r, curr_z = 200.0, 0.0

        current_z = curr_z
        for step in range(int(MAX_PLACE_DEPTH / DESCEND_STEP_MM)):
            current_z -= DESCEND_STEP_MM
            result = ik.solve(curr_r, current_z)
            if result is None:
                break
            j2, j3, j4 = result
            target = dict(self.pose)
            target.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            tof = self.servo.read_tof()
            if tof is not None and tof <= PLACE_DIST:
                break

        self.servo.gripper(True)
        time.sleep(0.3)

        # 들어올리기
        lift_z = current_z + LIFT_HEIGHT * 2
        result = ik.solve(curr_r, lift_z)
        if result:
            j2, j3, j4 = result
            lift = dict(self.pose)
            lift.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, lift, MOVE_DURATION_FAST
            )

        self.cycle_count += 1
        print(f"[PLACE] {self.detected_class} placed ({self.cycle_count})")
        return True

    # ── Error Recovery ───────────────────────────────────────

    def _error_recovery(self):
        print("[ERR] Recovery — returning home")
        self.servo.gripper(True)
        time.sleep(0.3)

        carry = dict(CARRY)
        carry["J1"] = self.pose.get("J1", HOME["J1"])
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)
        self.pose = s_curve.execute(self.servo, self.pose, HOME, MOVE_DURATION)

        self.detected_class = None
        self.offset_j1 = 0.0
        self.offset_r = 0.0
        return False


# ═══════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Master Trajectory Player with Checkpoint Correction"
    )
    parser.add_argument("--trajectory", type=str, default=DEFAULT_TRAJECTORY,
                        help=f"Trajectory CSV file (default: {DEFAULT_TRAJECTORY})")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run without hardware")
    args = parser.parse_args()

    player = TrajectoryPlayer(
        trajectory_name=args.trajectory,
        dry_run=args.dry_run,
    )
    player.run()
