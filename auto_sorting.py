"""
자동 분류 시스템 - 티칭 궤적 + IK 보정

구조:
  티칭 = 경로 이동 (웨이포인트 CSV, joint angle 직접 저장)
  IK   = 정밀 작업 (접근, 하강, 파지, 배치)

사이클:
  HOME(토크OFF) -> 토크ON -> 스캔(티칭) -> 검출 -> IK접근 -> 파지 -> lift
  -> 분류궤적(티칭) -> ToF하강 -> 배치 -> HOME -> 토크OFF -> 반복

실행:
  python auto_sorting.py
  python auto_sorting.py --dry-run
"""
import os
import csv
import time
import math
import argparse

from config import (
    HOME,
    GRASP_DIST, PLACE_DIST, DESCEND_STEP_MM, LIFT_HEIGHT,
    MAX_PLACE_DEPTH, MAX_DESCEND_DEPTH,
    GRIP_CLOSE_WAIT,
    ARUCO_DICT, BIN_ARUCO, BINS,
    CAMERA_ID, CAM_FX, CAM_FY,
    CAM_OFFSET_X, CAM_OFFSET_Y,
    GRIPPER_PX, GRIPPER_PY, VS_GAIN, VS_TOLERANCE, VS_MAX_ITER,
    YOLO_CONF, STABLE_FRAMES,
    TRAJECTORY_DIR, SCAN_TRAJ,
    MOVE_DURATION, MOVE_DURATION_FAST,
)
from servo_driver import ServoDriver
import arm_ik
import s_curve


# ═══════════════════════════════════════════════════════════════
# Trajectory I/O
# ═══════════════════════════════════════════════════════════════

def load_trajectory(name):
    path = os.path.join(TRAJECTORY_DIR, name)
    if not os.path.exists(path):
        print(f"[ERR] Trajectory not found: {path}")
        return None
    with open(path, "r") as f:
        rows = []
        for row in csv.DictReader(f):
            parsed = {}
            for k, v in row.items():
                if k in ("checkpoint", "gripper"):
                    parsed[k] = v.strip()
                elif k == "seq":
                    continue
                else:
                    parsed[k] = float(v)
            rows.append(parsed)
        return rows


def wp_to_pose(row):
    return {j: row[j] for j in ["J1", "J2", "J3", "J4", "J5"] if j in row}


def _pose_to_rad(pose_deg):
    """s_curve용 degree pose -> arm_ik용 radian dict."""
    return {j: math.radians(pose_deg.get(j, 0)) for j in ["J1", "J2", "J3", "J4", "J5"]}


def _ik_to_pose(q_rad):
    """arm_ik IK 결과(rad) -> s_curve용 degree pose."""
    return arm_ik.rad_to_deg(q_rad)


def _current_xyz(pose_deg):
    """현재 pose(deg) -> TCP (x, y, z) mm."""
    q = _pose_to_rad(pose_deg)
    return arm_ik.fk(q["J1"], q["J2"], q["J3"], q["J4"])


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
            print(f"[YOLO] Model loaded: {path}")
        except ImportError:
            print("[YOLO] hailo_platform not available - dry-run")
            self.dry_run = True
        except Exception as e:
            print(f"[YOLO] Load failed: {e} - dry-run")
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
                print(f"[CAM] Camera {camera_id} failed - dry-run")
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

    def close(self):
        if self.cap:
            self.cap.release()


# ═══════════════════════════════════════════════════════════════
# Auto Sorter
# ═══════════════════════════════════════════════════════════════

class AutoSorter:

    def __init__(self, dry_run=False):
        self.servo = ServoDriver(dry_run=dry_run)
        self.detector = YoloDetector(dry_run=dry_run)
        self.camera = Camera(dry_run=dry_run)

        self.pose = dict(HOME)
        self.cycle_count = 0

    # ── Main Loop ────────────────────────────────────────────

    def run(self):
        print("[AUTO] Starting... (Ctrl+C to stop)")

        self.servo.home()
        self.pose = dict(HOME)

        try:
            while True:
                self.servo.torque_on()
                time.sleep(0.2)

                result = self._scan()
                if result is None:
                    self._go_home()
                    continue

                detected_class, tx, ty, tz = result

                if not self._approach_and_pick(tx, ty, tz):
                    self._error_recovery()
                    continue

                if not self._place(detected_class):
                    self._error_recovery()
                    continue

                self._go_home()
                self.cycle_count += 1
                print(f"[AUTO] Cycle #{self.cycle_count} complete")

        except KeyboardInterrupt:
            print("\n[AUTO] Stopped by user")
        finally:
            self.servo.gripper(True)
            self._go_home()
            self.camera.close()
            self.servo.close()

    # ── SCAN: 티칭 궤적 재생 + YOLO ─────────────────────────

    def _scan(self):
        traj = load_trajectory(SCAN_TRAJ)
        if not traj:
            return None

        print("[SCAN] Playing scan trajectory...")

        for wp in traj:
            target = wp_to_pose(wp)
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION
            )
            time.sleep(0.3)

            detection = self._try_detect()
            if detection:
                return detection

        print("[SCAN] No object found")
        return None

    def _try_detect(self):
        stable_count = 0
        last_class = None

        for _ in range(STABLE_FRAMES * 3):
            frame = self.camera.read()
            if frame is None:
                return None

            detections = self.detector.detect(frame)
            if not detections:
                stable_count = 0
                last_class = None
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
                    return None

                cx, cy, cz = _current_xyz(self.pose)
                px, py = best["cx"], best["cy"]
                dx_mm, dy_mm = self.camera.pixel_to_mm(px, py, tof)

                target_x = cx + dx_mm + CAM_OFFSET_X
                target_y = cy + dy_mm + CAM_OFFSET_Y
                target_z = cz

                print(f"[SCAN] Found: {best['class']} "
                      f"(conf={best['conf']:.2f}) "
                      f"at ({target_x:.0f},{target_y:.0f},{target_z:.0f})")
                return best["class"], target_x, target_y, target_z

        return None

    # ── APPROACH + PICK: IK 정밀 작업 ────────────────────────

    def _approach_and_pick(self, tx, ty, tz):
        pre_z = tz + LIFT_HEIGHT
        q = arm_ik.ik(tx, ty, pre_z)
        if q is None:
            print("[APPROACH] IK failed")
            return False
        target = _ik_to_pose(q)
        self.pose = s_curve.execute(
            self.servo, self.pose, target, MOVE_DURATION
        )

        self._visual_servo(tx, ty)

        cx, cy, cz = _current_xyz(self.pose)
        current_z = cz

        for _ in range(int(MAX_DESCEND_DEPTH / DESCEND_STEP_MM)):
            current_z -= DESCEND_STEP_MM
            q = arm_ik.ik(cx, cy, current_z)
            if q is None:
                print("[DESCEND] IK limit")
                break
            target = _ik_to_pose(q)
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            tof = self.servo.read_tof()
            if tof is not None and tof <= GRASP_DIST:
                print(f"[DESCEND] ToF={tof}mm - grasp distance")
                break

        self.servo.gripper(False)
        time.sleep(GRIP_CLOSE_WAIT)

        lift_z = current_z + LIFT_HEIGHT
        q = arm_ik.ik(cx, cy, lift_z)
        if q is None:
            print("[PICK] Lift IK failed")
            return False
        target = _ik_to_pose(q)
        self.pose = s_curve.execute(
            self.servo, self.pose, target, MOVE_DURATION_FAST
        )

        print("[PICK] Object picked")
        return True

    def _visual_servo(self, tx, ty):
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
                print(f"[VS] Converged ({i + 1} iter)")
                return

            tof = self.servo.read_tof()
            h = tof if tof and tof < 2000 else 200

            cx, cy, cz = _current_xyz(self.pose)
            new_x = cx + dx * VS_GAIN * h / CAM_FX
            new_y = cy + dy * VS_GAIN * h / CAM_FY

            q = arm_ik.ik(new_x, new_y, cz)
            if q is None:
                continue
            target = _ik_to_pose(q)
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

    # ── PLACE: 티칭 궤적 + ToF 하강 ─────────────────────────

    def _place(self, detected_class):
        bin_cfg = BINS.get(detected_class)
        if not bin_cfg:
            print(f"[PLACE] Unknown class: {detected_class}")
            return False

        traj = load_trajectory(bin_cfg["traj"])
        if not traj:
            print(f"[PLACE] No trajectory for {detected_class}")
            return False

        print(f"[PLACE] Playing {detected_class} trajectory...")
        current = self.servo.read_all() or self.pose
        for wp in traj:
            target = wp_to_pose(wp)
            current = s_curve.execute(
                self.servo, current, target, MOVE_DURATION
            )
        self.pose = current

        cx, cy, cz = _current_xyz(self.pose)
        current_z = cz

        for _ in range(int(MAX_PLACE_DEPTH / DESCEND_STEP_MM)):
            current_z -= DESCEND_STEP_MM
            q = arm_ik.ik(cx, cy, current_z)
            if q is None:
                break
            target = _ik_to_pose(q)
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            tof = self.servo.read_tof()
            if tof is not None and tof <= PLACE_DIST:
                print(f"[PLACE] ToF={tof}mm - place distance")
                break

        self.servo.gripper(True)
        time.sleep(0.3)

        lift_z = current_z + LIFT_HEIGHT * 2
        q = arm_ik.ik(cx, cy, lift_z)
        if q:
            target = _ik_to_pose(q)
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

        print(f"[PLACE] {detected_class} placed")
        return True

    # ── HOME + Torque OFF ────────────────────────────────────

    def _go_home(self):
        self.pose = s_curve.execute(
            self.servo, self.pose, HOME, MOVE_DURATION
        )
        time.sleep(0.5)
        self.servo.torque_off()

    # ── Error Recovery ───────────────────────────────────────

    def _error_recovery(self):
        print("[ERR] Recovery - returning home")
        self.servo.gripper(True)
        time.sleep(0.3)
        self._go_home()


# ═══════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto Sorting System")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    sorter = AutoSorter(dry_run=args.dry_run)
    sorter.run()
