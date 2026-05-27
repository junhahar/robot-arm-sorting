"""
자동 분류 상태머신 — YOLO + IK + 티칭 궤적 하이브리드

상태 흐름:
  IDLE → SCAN → DETECT → APPROACH → DESCEND → PICK
       → TO_CARRY → ROTATE → PLACE → RETURN → IDLE

실행:
  python auto_sorting.py --dry-run   (하드웨어 없이 테스트)
  python auto_sorting.py             (실제 로봇)
"""
import os
import csv
import time
import math
import argparse
from enum import Enum, auto

import numpy as np

from config import (
    HOME, CARRY, JOINTS,
    PRE_GRASP_Z, WORK_SURFACE_Z, TOF_GRASP_DIST, DESCEND_STEP_MM,
    BIN_BOLT_J1, BIN_NUT_J1,
    CAMERA_ID, HOMOGRAPHY_FILE, YOLO_CONF, CLASS_NAMES,
    TRAJECTORY_DIR, SCAN_TRAJ, BOLT_PLACE_TRAJ, NUT_PLACE_TRAJ,
    MOVE_DURATION, MOVE_DURATION_FAST,
)
from servo_driver import ServoDriver
import ik
import s_curve


# ═══════════════════════════════════════════════════════════════
# State Machine
# ═══════════════════════════════════════════════════════════════

class State(Enum):
    IDLE = auto()
    SCAN = auto()
    DETECT = auto()
    APPROACH = auto()
    DESCEND = auto()
    PICK = auto()
    TO_CARRY = auto()
    ROTATE = auto()
    PLACE = auto()
    RETURN = auto()
    ERROR = auto()


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
                if k == "gripper":
                    parsed[k] = v
                else:
                    parsed[k] = float(v)
            rows.append(parsed)
        return rows


def traj_to_pose(row):
    return {j: row[j] for j in ["J1", "J2", "J3", "J4", "J5"] if j in row}


# ═══════════════════════════════════════════════════════════════
# YOLO Detector (Hailo / CPU fallback / dry-run)
# ═══════════════════════════════════════════════════════════════

class YoloDetector:
    """YOLO 추론 — Hailo-8L 또는 dry-run 모드"""

    def __init__(self, model_path=None, dry_run=False):
        self.dry_run = dry_run
        self.model = None
        if not dry_run and model_path:
            self._load_model(model_path)

    def _load_model(self, path):
        # Hailo HEF 로딩 — RPi5에서만 동작
        try:
            from hailo_platform import HEF, VDevice
            self.hef = HEF(path)
            self.device = VDevice()
            self.model = self.device.configure(self.hef)
            print(f"[YOLO] Hailo 모델 로드: {path}")
        except ImportError:
            print("[YOLO] hailo_platform 미설치 — dry-run 전환")
            self.dry_run = True
        except Exception as e:
            print(f"[YOLO] 모델 로드 실패: {e} — dry-run 전환")
            self.dry_run = True

    def detect(self, frame):
        """프레임에서 객체 검출 → [{"class": str, "conf": float, "cx": int, "cy": int, "bbox": tuple}]"""
        if self.dry_run:
            return []

        # TODO: Hailo 추론 파이프라인 구현
        # results = self.model.infer(frame)
        # detections = parse_yolo_output(results, YOLO_CONF, CLASS_NAMES)
        # return detections
        return []


# ═══════════════════════════════════════════════════════════════
# Camera + Homography
# ═══════════════════════════════════════════════════════════════

class Camera:

    def __init__(self, camera_id=CAMERA_ID, dry_run=False):
        self.dry_run = dry_run
        self.cap = None
        self.homography = None
        self.camera_matrix = None
        self.dist_coeffs = None

        if not dry_run:
            import cv2
            self.cap = cv2.VideoCapture(camera_id)
            if not self.cap.isOpened():
                print(f"[CAM] 카메라 {camera_id} 열기 실패 — dry-run 전환")
                self.dry_run = True

        self._load_homography()

    def _load_homography(self):
        if os.path.exists(HOMOGRAPHY_FILE):
            data = np.load(HOMOGRAPHY_FILE)
            self.homography = data["H"]
            print(f"[CAM] 호모그래피 로드: {HOMOGRAPHY_FILE}")

    def read(self):
        if self.dry_run:
            return None
        import cv2
        ret, frame = self.cap.read()
        return frame if ret else None

    def pixel_to_mm(self, px, py):
        """호모그래피 변환: (px, py) → (x_mm, y_mm)"""
        if self.homography is None:
            return None, None
        pt = np.array([px, py, 1.0], dtype=np.float64)
        result = self.homography @ pt
        result /= result[2]
        return float(result[0]), float(result[1])

    def close(self):
        if self.cap:
            self.cap.release()


# ═══════════════════════════════════════════════════════════════
# Sorting Controller
# ═══════════════════════════════════════════════════════════════

class SortingController:

    def __init__(self, dry_run=False):
        self.servo = ServoDriver(dry_run=dry_run)
        self.detector = YoloDetector(dry_run=dry_run)
        self.camera = Camera(dry_run=dry_run)

        self.state = State.IDLE
        self.pose = dict(HOME)
        self.detected = None   # {"class": str, "cx": int, "cy": int, ...}
        self.target_j1 = 90.0
        self.target_r = 0.0
        self.cycle_count = 0

    # ── Main Loop ──

    def run(self):
        print("[SORT] 자동 분류 시작 (Ctrl+C 종료)")
        self.servo.home()
        self.pose = dict(HOME)
        self.state = State.SCAN

        handlers = {
            State.IDLE:     self._on_idle,
            State.SCAN:     self._on_scan,
            State.DETECT:   self._on_detect,
            State.APPROACH: self._on_approach,
            State.DESCEND:  self._on_descend,
            State.PICK:     self._on_pick,
            State.TO_CARRY: self._on_to_carry,
            State.ROTATE:   self._on_rotate,
            State.PLACE:    self._on_place,
            State.RETURN:   self._on_return,
            State.ERROR:    self._on_error,
        }

        try:
            while True:
                handler = handlers.get(self.state, self._on_error)
                prev = self.state
                self.state = handler()
                if prev != self.state:
                    print(f"[STATE] {prev.name} → {self.state.name}")
        except KeyboardInterrupt:
            print("\n[SORT] 중단됨")
        finally:
            self.servo.gripper(True)
            self.servo.home()
            self.camera.close()
            self.servo.close()

    # ── State Handlers ──

    def _on_idle(self):
        time.sleep(1)
        return State.SCAN

    def _on_scan(self):
        """티칭된 탐색 경로를 재생하면서 YOLO로 물체 탐색"""
        traj = load_trajectory(SCAN_TRAJ)
        if not traj:
            print(f"[SCAN] {SCAN_TRAJ} 없음 — 대기")
            time.sleep(3)
            return State.IDLE

        t0 = time.time()
        for wp in traj:
            target = traj_to_pose(wp)
            self.servo.move_all(target)
            self.pose = target

            target_t = wp["time_ms"] / 1000.0
            while time.time() - t0 < target_t:
                frame = self.camera.read()
                if frame is not None:
                    detections = self.detector.detect(frame)
                    if detections:
                        best = max(detections, key=lambda d: d["conf"])
                        self.detected = best
                        print(f"[SCAN] 발견: {best['class']} (conf={best['conf']:.2f})")
                        return State.DETECT
                time.sleep(0.01)

        print("[SCAN] 물체 미발견")
        return State.IDLE

    def _on_detect(self):
        """YOLO 결과 → 호모그래피 → 실제 좌표 → IK 가능 여부 확인"""
        det = self.detected
        x_mm, y_mm = self.camera.pixel_to_mm(det["cx"], det["cy"])

        if x_mm is None:
            print("[DETECT] 호모그래피 미설정")
            return State.ERROR

        j1, r = ik.xy_to_polar(x_mm, y_mm)

        if not ik.reachable(r, PRE_GRASP_Z):
            print(f"[DETECT] IK 도달 불가: r={r:.0f}mm — 스킵")
            return State.SCAN

        self.target_j1 = j1
        self.target_r = r
        print(f"[DETECT] 좌표: ({x_mm:.0f}, {y_mm:.0f})mm → J1={j1:.1f}° r={r:.0f}mm")
        return State.APPROACH

    def _on_approach(self):
        """IK로 PRE_GRASP 위치까지 S-curve 이동"""
        j2, j3, j4 = ik.solve(self.target_r, PRE_GRASP_Z)
        target = {
            "J1": self.target_j1,
            "J2": j2, "J3": j3, "J4": j4,
            "J5": 90.0,
        }
        self.pose = s_curve.execute(self.servo, self.pose, target, MOVE_DURATION)

        # 2차 보정: 카메라로 재확인
        frame = self.camera.read()
        if frame is not None:
            detections = self.detector.detect(frame)
            if detections:
                best = max(detections, key=lambda d: d["conf"])
                x2, y2 = self.camera.pixel_to_mm(best["cx"], best["cy"])
                if x2 is not None:
                    j1_c, r_c = ik.xy_to_polar(x2, y2)
                    j2_c, j3_c, j4_c = ik.solve(r_c, PRE_GRASP_Z)
                    correction = {
                        "J1": j1_c, "J2": j2_c, "J3": j3_c, "J4": j4_c, "J5": 90.0,
                    }
                    self.pose = s_curve.execute(
                        self.servo, self.pose, correction, MOVE_DURATION_FAST,
                    )
                    self.target_r = r_c
                    print("[APPROACH] 2차 보정 완료")

        return State.DESCEND

    def _on_descend(self):
        """ToF 모니터링하며 천천히 하강"""
        current_z = PRE_GRASP_Z
        min_z = WORK_SURFACE_Z - 20

        while current_z > min_z:
            current_z -= DESCEND_STEP_MM
            j2, j3, j4 = ik.solve(self.target_r, current_z)
            target = dict(self.pose)
            target.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST,
            )

            tof = self.servo.read_tof()
            if tof is not None and tof < TOF_GRASP_DIST:
                print(f"[DESCEND] ToF={tof}mm — 파지 거리 도달")
                return State.PICK

        print("[DESCEND] 최대 깊이 도달, 물체 미감지")
        return State.ERROR

    def _on_pick(self):
        """그리퍼 닫기 + 들어올리기"""
        self.servo.gripper(False)
        time.sleep(0.5)

        # 살짝 들어올려서 파지 확인
        j2, j3, j4 = ik.solve(self.target_r, PRE_GRASP_Z)
        lift = dict(self.pose)
        lift.update({"J2": j2, "J3": j3, "J4": j4})
        self.pose = s_curve.execute(self.servo, self.pose, lift, MOVE_DURATION_FAST)

        tof_after = self.servo.read_tof()
        if tof_after is not None and tof_after > TOF_GRASP_DIST * 3:
            print("[PICK] 파지 실패 — 물체 없음")
            self.servo.gripper(True)
            return State.ERROR

        print("[PICK] 파지 성공")
        return State.TO_CARRY

    def _on_to_carry(self):
        """현재 위치 → CARRY (J1은 현재값 유지)"""
        target = dict(CARRY)
        target["J1"] = self.pose["J1"]
        self.pose = s_curve.execute(self.servo, self.pose, target, MOVE_DURATION)
        return State.ROTATE

    def _on_rotate(self):
        """J1만 분류 통 방향으로 회전 (J2-J5 CARRY 유지)"""
        cls = self.detected["class"]
        bin_j1 = BIN_BOLT_J1 if cls == "bolt" else BIN_NUT_J1
        self.pose = s_curve.execute_j1_only(
            self.servo, self.pose, bin_j1, MOVE_DURATION,
        )
        print(f"[ROTATE] {cls} → J1={bin_j1:.0f}°")
        return State.PLACE

    def _on_place(self):
        """티칭된 놓기 궤적 재생 + 그리퍼 열기"""
        cls = self.detected["class"]
        traj_name = BOLT_PLACE_TRAJ if cls == "bolt" else NUT_PLACE_TRAJ
        traj = load_trajectory(traj_name)

        if not traj:
            print(f"[PLACE] {traj_name} 없음 — 에러")
            return State.ERROR

        t0 = time.time()
        for wp in traj:
            target = traj_to_pose(wp)
            target_t = wp["time_ms"] / 1000.0
            elapsed = time.time() - t0
            if target_t > elapsed:
                time.sleep(target_t - elapsed)
            self.servo.move_all(target)
            self.pose = target

            if wp.get("gripper") == "close":
                self.servo.gripper(False)
            elif wp.get("gripper") == "open":
                self.servo.gripper(True)

        self.servo.gripper(True)
        time.sleep(0.3)
        print(f"[PLACE] {cls} 배치 완료")
        self.cycle_count += 1
        return State.RETURN

    def _on_return(self):
        """CARRY → HOME 복귀"""
        carry = dict(CARRY)
        carry["J1"] = self.pose["J1"]
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)

        self.pose = s_curve.execute(self.servo, self.pose, HOME, MOVE_DURATION)

        # 서보 위치 검증
        actual = self.servo.read_all()
        if actual:
            for j in HOME:
                diff = abs(actual.get(j, HOME[j]) - HOME[j])
                if diff > 5.0:
                    print(f"[RETURN] {j} 위치 오차: {diff:.1f}° — 재보정")

        print(f"[RETURN] 홈 복귀 완료 (누적 {self.cycle_count}회)")
        return State.IDLE

    def _on_error(self):
        """에러 복구: 그리퍼 열고 → CARRY → HOME"""
        print("[ERROR] 에러 복구 시작")
        self.servo.gripper(True)
        time.sleep(0.3)

        carry = dict(CARRY)
        carry["J1"] = self.pose.get("J1", HOME["J1"])
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)
        self.pose = s_curve.execute(self.servo, self.pose, HOME, MOVE_DURATION)

        self.detected = None
        return State.IDLE


# ═══════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="볼트/너트 자동 분류")
    parser.add_argument("--dry-run", action="store_true", help="하드웨어 없이 테스트")
    args = parser.parse_args()

    ctrl = SortingController(dry_run=args.dry_run)
    ctrl.run()
