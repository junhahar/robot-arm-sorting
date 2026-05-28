"""
자동 분류 상태머신 -7단계 알고리즘

0단계: 환경 보정 (ArUco 오프셋)
1단계: SCAN (티칭 궤적 + YOLO 3-프레임 안정 검출)
2단계: APPROACH (IK + Visual Servoing)
3단계: DESCEND (카메라 + ToF 병행 하강)
4단계: PICK (그리퍼 + FSR 확인)
5단계: TO_CARRY → ROTATE (운반 + 티칭 + ArUco bin 탐색)
6단계: PLACE (ToF 하강 + 놓기)
7단계: RETURN (홈 복귀)

실행:
  python auto_sorting.py --dry-run
  python auto_sorting.py
"""
import os
import csv
import time
import math
import argparse
from enum import Enum, auto

from config import (
    HOME, CARRY,
    GRASP_DIST, PLACE_DIST, DESCEND_STEP_MM, LIFT_HEIGHT,
    MAX_PLACE_DEPTH, MAX_DESCEND_DEPTH,
    GRIP_CLOSE_WAIT, FSR_THRESHOLD,
    ARUCO_DICT, ENV_ARUCO_ID, BIN_ARUCO, BINS,
    CAMERA_ID, CAM_FX, CAM_FY, CAM_CX, CAM_CY,
    CAM_OFFSET_X, CAM_OFFSET_Y,
    GRIPPER_PX, GRIPPER_PY, VS_GAIN, VS_TOLERANCE, VS_MAX_ITER,
    YOLO_CONF, STABLE_FRAMES,
    TRAJECTORY_DIR, SCAN_TRAJ,
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
            print(f"[YOLO] Hailo 모델 로드: {path}")
        except ImportError:
            print("[YOLO] hailo_platform 미설치 -dry-run 전환")
            self.dry_run = True
        except Exception as e:
            print(f"[YOLO] 모델 로드 실패: {e} -dry-run 전환")
            self.dry_run = True

    def detect(self, frame):
        """프레임에서 객체 검출 → [{"class": str, "conf": float, "cx": int, "cy": int}]"""
        if self.dry_run:
            return []
        # TODO: Hailo 추론 파이프라인 구현
        return []


# ═══════════════════════════════════════════════════════════════
# Camera (핀홀 모델 + ArUco)
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
                print(f"[CAM] 카메라 {camera_id} 열기 실패 -dry-run 전환")
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
        """핀홀 모델: 픽셀 → 카메라 기준 mm 오프셋"""
        x_mm = (px - CAM_CX) * height_mm / CAM_FX
        y_mm = (py - CAM_CY) * height_mm / CAM_FY
        return x_mm, y_mm

    def detect_aruco(self, frame):
        """ArUco 검출 → {marker_id: (center_x_px, center_y_px)}"""
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
# Sorting Controller
# ═══════════════════════════════════════════════════════════════

class SortingController:

    def __init__(self, dry_run=False):
        self.servo = ServoDriver(dry_run=dry_run)
        self.detector = YoloDetector(dry_run=dry_run)
        self.camera = Camera(dry_run=dry_run)

        self.state = State.IDLE
        self.pose = dict(HOME)
        self.env_offset = (0.0, 0.0)
        self.detected_class = None
        self.target_j1 = 90.0
        self.target_r = 0.0
        self.target_z = -150.0
        self.cycle_count = 0

    # ── 0단계: 환경 보정 ─────────────────────────────────────

    def _calibrate_env(self):
        """ArUco 마커로 환경 오프셋 보정"""
        print("[CALIB] 환경 보정 시작")
        self.servo.home()
        self.pose = dict(HOME)
        time.sleep(1)

        frame = self.camera.read()
        if frame is None:
            print("[CALIB] 카메라 읽기 실패 -오프셋 (0, 0)")
            return

        markers = self.camera.detect_aruco(frame)
        if ENV_ARUCO_ID not in markers:
            print("[CALIB] 환경 ArUco 미검출 -오프셋 (0, 0)")
            return

        px, py = markers[ENV_ARUCO_ID]
        tof = self.servo.read_tof()
        if tof is None or tof > 2000:
            print("[CALIB] ToF 읽기 실패 -오프셋 (0, 0)")
            return

        x_mm, y_mm = self.camera.pixel_to_mm(px, py, tof)
        self.env_offset = (x_mm, y_mm)
        print(f"[CALIB] 환경 오프셋: ({x_mm:.1f}, {y_mm:.1f})mm")

    # ── Main Loop ────────────────────────────────────────────

    def run(self):
        print("[SORT] 자동 분류 시작 (Ctrl+C 종료)")
        self._calibrate_env()
        self.servo.home()
        self.pose = dict(HOME)
        self.state = State.SCAN

        handlers = {
            State.IDLE:     self._on_idle,
            State.SCAN:     self._on_scan,
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

    # ── 1단계: SCAN ──────────────────────────────────────────

    def _on_idle(self):
        time.sleep(1)
        return State.SCAN

    def _on_scan(self):
        """티칭 궤적 재생 + YOLO 3-프레임 안정 검출"""
        traj = load_trajectory(SCAN_TRAJ)
        if not traj:
            print(f"[SCAN] {SCAN_TRAJ} 없음 -대기")
            time.sleep(3)
            return State.IDLE

        stable_count = 0
        last_class = None

        t0 = time.time()
        for wp in traj:
            target = traj_to_pose(wp)
            target_t = wp["time_ms"] / 1000.0

            # 타이밍 맞춰서 이동 + 카메라 검출
            while time.time() - t0 < target_t:
                frame = self.camera.read()
                if frame is None:
                    time.sleep(0.01)
                    continue

                detections = self.detector.detect(frame)
                if not detections:
                    stable_count = 0
                    last_class = None
                    time.sleep(0.01)
                    continue

                best = max(detections, key=lambda d: d["conf"])
                if best["conf"] < YOLO_CONF:
                    stable_count = 0
                    last_class = None
                    time.sleep(0.01)
                    continue

                # 3-프레임 안정 검출
                if best["class"] == last_class:
                    stable_count += 1
                else:
                    stable_count = 1
                    last_class = best["class"]

                if stable_count >= STABLE_FRAMES:
                    tof = self.servo.read_tof()
                    if tof is None or tof > 2000:
                        time.sleep(0.01)
                        continue

                    # 현재 팔 위치 읽기
                    actual = self.servo.read_all()
                    if not actual:
                        continue
                    curr_r, curr_z = ik.forward(
                        actual["J2"], actual["J3"], actual["J4"]
                    )

                    # 핀홀 모델로 물체 오프셋 계산 (카메라 기준)
                    px, py = best["cx"], best["cy"]
                    x_cam, y_cam = self.camera.pixel_to_mm(px, py, tof)

                    # 카메라 마운트 + 환경 오프셋 보정
                    x_grip = x_cam + CAM_OFFSET_X - self.env_offset[0]
                    y_grip = y_cam + CAM_OFFSET_Y - self.env_offset[1]

                    # IK 타겟 계산
                    self.target_j1 = actual["J1"] + math.degrees(
                        math.atan2(x_grip, curr_r)
                    )
                    self.target_r = curr_r + y_grip
                    self.target_z = curr_z - tof

                    if not ik.reachable(self.target_r, self.target_z + LIFT_HEIGHT):
                        print(f"[SCAN] 도달 불가: r={self.target_r:.0f}mm")
                        stable_count = 0
                        continue

                    self.detected_class = best["class"]
                    print(
                        f"[SCAN] 발견: {best['class']} "
                        f"(conf={best['conf']:.2f}) "
                        f"J1={self.target_j1:.1f}° "
                        f"r={self.target_r:.0f}mm"
                    )
                    return State.APPROACH

                time.sleep(0.01)

            self.servo.move_all(target)
            self.pose = target

        print("[SCAN] 물체 미발견")
        return State.IDLE

    # ── 2단계: APPROACH ──────────────────────────────────────

    def _on_approach(self):
        """IK 대략 이동 + Visual Servoing 수렴"""
        pre_z = self.target_z + LIFT_HEIGHT
        result = ik.solve(self.target_r, pre_z)
        if result is None:
            print("[APPROACH] IK 실패 - 도달 불가")
            return State.ERROR
        j2, j3, j4 = result
        target = {
            "J1": self.target_j1,
            "J2": j2, "J3": j3, "J4": j4, "J5": 90.0,
        }
        self.pose = s_curve.execute(self.servo, self.pose, target, MOVE_DURATION)

        # Visual Servoing 루프
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
                print(f"[APPROACH] VS 수렴 ({i + 1}회)")
                return State.DESCEND

            tof = self.servo.read_tof()
            h = tof if tof and tof < 2000 else abs(pre_z)

            actual = self.servo.read_all()
            if actual:
                curr_r, curr_z = ik.forward(
                    actual["J2"], actual["J3"], actual["J4"]
                )
            else:
                curr_r, curr_z = self.target_r, pre_z

            new_r = curr_r + dy * VS_GAIN * h / CAM_FY
            new_j1 = self.pose["J1"] + math.degrees(
                math.atan2(dx * VS_GAIN * h / CAM_FX, curr_r)
            )

            result = ik.solve(new_r, curr_z)
            if result is None:
                continue
            j2, j3, j4 = result
            adj = {"J1": new_j1, "J2": j2, "J3": j3, "J4": j4, "J5": 90.0}
            self.pose = s_curve.execute(
                self.servo, self.pose, adj, MOVE_DURATION_FAST
            )
            self.target_r = new_r

        print("[APPROACH] VS 완료 → DESCEND")
        return State.DESCEND

    # ── 3단계: DESCEND ───────────────────────────────────────

    def _on_descend(self):
        """카메라 + ToF 병행 하강"""
        current_z = self.target_z + LIFT_HEIGHT
        min_z = current_z - MAX_DESCEND_DEPTH

        while current_z > min_z:
            current_z -= DESCEND_STEP_MM
            result = ik.solve(self.target_r, current_z)
            if result is None:
                print("[DESCEND] IK 실패 - 하강 한계")
                break
            j2, j3, j4 = result
            target = dict(self.pose)
            target.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            # Visual Servoing 미세 보정
            frame = self.camera.read()
            if frame is not None:
                detections = self.detector.detect(frame)
                if detections:
                    best = max(detections, key=lambda d: d["conf"])
                    dx = best["cx"] - GRIPPER_PX
                    dy = best["cy"] - GRIPPER_PY
                    if abs(dx) > VS_TOLERANCE or abs(dy) > VS_TOLERANCE:
                        h = self.servo.read_tof() or abs(current_z)
                        adj_r = self.target_r + dy * VS_GAIN * h / CAM_FY
                        adj_j1 = self.pose["J1"] + math.degrees(
                            math.atan2(dx * VS_GAIN * h / CAM_FX, self.target_r)
                        )
                        result = ik.solve(adj_r, current_z)
                        if result is None:
                            continue
                        j2, j3, j4 = result
                        adj_pose = dict(self.pose)
                        adj_pose.update({
                            "J1": adj_j1, "J2": j2, "J3": j3, "J4": j4,
                        })
                        self.pose = s_curve.execute(
                            self.servo, self.pose, adj_pose, MOVE_DURATION_FAST
                        )
                        self.target_r = adj_r

            # ToF 거리 확인
            tof = self.servo.read_tof()
            if tof is not None and tof <= GRASP_DIST:
                print(f"[DESCEND] ToF={tof}mm -파지 거리 도달")
                return State.PICK

        print("[DESCEND] 최대 깊이 도달")
        return State.ERROR

    # ── 4단계: PICK ──────────────────────────────────────────

    def _on_pick(self):
        """그리퍼 닫기 + FSR 확인 + 들어올리기"""
        self.servo.gripper(False)
        time.sleep(GRIP_CLOSE_WAIT)

        fsr = self.servo.read_fsr()
        if fsr is not None and fsr < FSR_THRESHOLD:
            print(f"[PICK] FSR={fsr} -파지 실패, SCAN 복귀")
            self.servo.gripper(True)
            return State.SCAN

        lift_z = self.target_z + LIFT_HEIGHT
        result = ik.solve(self.target_r, lift_z)
        if result is None:
            print("[PICK] 리프트 IK 실패")
            return State.ERROR
        j2, j3, j4 = result
        lift = dict(self.pose)
        lift.update({"J2": j2, "J3": j3, "J4": j4})
        self.pose = s_curve.execute(self.servo, self.pose, lift, MOVE_DURATION_FAST)

        print(f"[PICK] 파지 성공: {self.detected_class}")
        return State.TO_CARRY

    # ── 5단계: TO_CARRY → ROTATE ─────────────────────────────

    def _on_to_carry(self):
        """CARRY 자세 + 분류통 티칭 궤적"""
        carry = dict(CARRY)
        carry["J1"] = self.pose["J1"]
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)

        cls = self.detected_class
        bin_cfg = BINS.get(cls)
        if not bin_cfg:
            print(f"[TO_CARRY] 미등록 클래스: {cls}")
            return State.ERROR

        traj = load_trajectory(bin_cfg["traj"])
        if traj:
            t0 = time.time()
            for wp in traj:
                target = traj_to_pose(wp)
                target_t = wp["time_ms"] / 1000.0
                elapsed = time.time() - t0
                if target_t > elapsed:
                    time.sleep(target_t - elapsed)
                self.servo.move_all(target)
                self.pose = target
        else:
            self.pose = s_curve.execute_j1_only(
                self.servo, self.pose, bin_cfg["j1"], MOVE_DURATION
            )

        return State.ROTATE

    def _on_rotate(self):
        """ArUco로 분류통 정밀 위치 확인"""
        cls = self.detected_class
        aruco_cfg = BIN_ARUCO.get(cls)
        if not aruco_cfg:
            return State.PLACE

        target_id = aruco_cfg["id"]

        for attempt in range(3):
            frame = self.camera.read()
            if frame is None:
                continue

            markers = self.camera.detect_aruco(frame)
            if target_id in markers:
                px, py = markers[target_id]
                tof = self.servo.read_tof()
                h = tof if tof and tof < 2000 else 200

                x_mm, y_mm = self.camera.pixel_to_mm(px, py, h)
                bin_x = x_mm + aruco_cfg["offset_x"]
                bin_y = y_mm + aruco_cfg["offset_y"]

                actual = self.servo.read_all()
                if actual:
                    curr_r, _ = ik.forward(
                        actual["J2"], actual["J3"], actual["J4"]
                    )
                else:
                    curr_r = 200.0

                j1_adj = self.pose["J1"] + math.degrees(
                    math.atan2(bin_x, curr_r)
                )
                self.target_j1 = j1_adj
                self.target_r = curr_r + bin_y
                print(f"[ROTATE] ArUco #{target_id} 발견 → PLACE")
                return State.PLACE

            adj = 5.0 if attempt == 0 else -10.0
            self.pose = s_curve.execute_j1_only(
                self.servo, self.pose,
                self.pose["J1"] + adj, MOVE_DURATION_FAST,
            )

        print("[ROTATE] ArUco 미발견 -대략 위치로 PLACE")
        return State.PLACE

    # ── 6단계: PLACE ─────────────────────────────────────────

    def _on_place(self):
        """bin 위 정렬 + ToF 하강 + 그리퍼 열기"""
        actual = self.servo.read_all()
        if actual:
            curr_r, curr_z = ik.forward(
                actual["J2"], actual["J3"], actual["J4"]
            )
        else:
            curr_r, curr_z = 200.0, 0.0

        current_z = curr_z
        descent = 0.0

        while descent < MAX_PLACE_DEPTH:
            current_z -= DESCEND_STEP_MM
            descent += DESCEND_STEP_MM

            result = ik.solve(curr_r, current_z)
            if result is None:
                print("[PLACE] IK 실패 - 하강 한계")
                break
            j2, j3, j4 = result
            target = dict(self.pose)
            target.update({"J2": j2, "J3": j3, "J4": j4})
            self.pose = s_curve.execute(
                self.servo, self.pose, target, MOVE_DURATION_FAST
            )

            tof = self.servo.read_tof()
            if tof is not None and tof <= PLACE_DIST:
                print(f"[PLACE] ToF={tof}mm -놓기 위치 도달")
                break

        self.servo.gripper(True)
        time.sleep(0.3)

        lift_z = current_z + LIFT_HEIGHT * 2
        result = ik.solve(curr_r, lift_z)
        if result is None:
            print("[PLACE] 리프트 IK 실패")
            return State.ERROR
        j2, j3, j4 = result
        lift = dict(self.pose)
        lift.update({"J2": j2, "J3": j3, "J4": j4})
        self.pose = s_curve.execute(self.servo, self.pose, lift, MOVE_DURATION_FAST)

        self.cycle_count += 1
        print(f"[PLACE] {self.detected_class} 배치 완료 ({self.cycle_count}회)")
        return State.RETURN

    # ── 7단계: RETURN ────────────────────────────────────────

    def _on_return(self):
        """CARRY → HOME 복귀"""
        carry = dict(CARRY)
        carry["J1"] = self.pose["J1"]
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)

        self.pose = s_curve.execute(self.servo, self.pose, HOME, MOVE_DURATION)
        self.detected_class = None
        print(f"[RETURN] 홈 복귀 (누적 {self.cycle_count}회)")
        return State.SCAN

    # ── ERROR ────────────────────────────────────────────────

    def _on_error(self):
        print("[ERROR] 에러 복구 시작")
        self.servo.gripper(True)
        time.sleep(0.3)

        carry = dict(CARRY)
        carry["J1"] = self.pose.get("J1", HOME["J1"])
        self.pose = s_curve.execute(self.servo, self.pose, carry, MOVE_DURATION)
        self.pose = s_curve.execute(self.servo, self.pose, HOME, MOVE_DURATION)

        self.detected_class = None
        return State.IDLE


# ═══════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="볼트/너트/와셔 자동 분류")
    parser.add_argument("--dry-run", action="store_true", help="하드웨어 없이 테스트")
    args = parser.parse_args()

    ctrl = SortingController(dry_run=args.dry_run)
    ctrl.run()
