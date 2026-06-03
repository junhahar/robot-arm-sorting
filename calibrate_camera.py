"""
카메라 호모그래피 캘리브레이션 — 픽셀 좌표 → 실제 mm 변환

사용법:
  python calibrate_camera.py                  (기본 카메라)
  python calibrate_camera.py --camera 1       (카메라 ID 지정)

조작:
  SPACE  화면 고정/해제
  클릭   고정된 화면에서 캘리브레이션 점 선택 (최소 4개)
  ENTER  점 선택 완료 → 실제 좌표 입력
  Q      종료
"""
import cv2
import numpy as np
import argparse
import os
from config import CAMERA_ID, HOMOGRAPHY_FILE


class HomographyCalibrator:

    def __init__(self, camera_id=CAMERA_ID):
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라 {camera_id} 열기 실패")
        self.image_points = []
        self.frame = None
        self.frame_clean = None
        self.frozen = False

    def _on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or not self.frozen:
            return
        self.image_points.append((x, y))
        n = len(self.image_points)
        cv2.circle(self.frame, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(self.frame, f"P{n}", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        print(f"  P{n}: pixel=({x}, {y})")

    def _draw_hud(self, display):
        n = len(self.image_points)
        if self.frozen:
            cv2.putText(display, f"[FROZEN] Click P{n + 1} / ENTER when done",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display, "SPACE: freeze / Q: quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def _input_world_coords(self):
        print(f"\n선택한 {len(self.image_points)}개 점의 실제 좌표를 입력하세요 (mm)")
        print("형식: x,y  (예: 100,200)\n")
        world_points = []
        for i, (px, py) in enumerate(self.image_points):
            while True:
                try:
                    raw = input(f"  P{i + 1} pixel=({px},{py}) → x_mm, y_mm: ")
                    x, y = map(float, raw.strip().split(","))
                    world_points.append((x, y))
                    break
                except (ValueError, KeyboardInterrupt):
                    print("    다시 입력 (예: 100,200)")
        return world_points

    def _compute_and_verify(self, world_points):
        img = np.array(self.image_points, dtype=np.float64)
        wld = np.array(world_points, dtype=np.float64)
        H, mask = cv2.findHomography(img, wld)

        print("\n── 검증 ──")
        errors = []
        for i in range(len(self.image_points)):
            pt = np.array([*self.image_points[i], 1.0])
            result = H @ pt
            result /= result[2]
            actual = world_points[i]
            err = np.sqrt((result[0] - actual[0]) ** 2 + (result[1] - actual[1]) ** 2)
            errors.append(err)
            print(f"  P{i + 1}: 예측=({result[0]:.1f}, {result[1]:.1f})mm "
                  f"실제=({actual[0]:.1f}, {actual[1]:.1f})mm 오차={err:.2f}mm")

        mean_err = np.mean(errors)
        print(f"\n  평균 오차: {mean_err:.2f}mm")
        if mean_err > 5.0:
            print("  ⚠ 오차가 큽니다. 점 위치를 확인하고 재시도하세요.")

        np.savez(HOMOGRAPHY_FILE, H=H,
                 image_points=img, world_points=wld)
        print(f"\n저장 완료: {HOMOGRAPHY_FILE}")
        return H

    def _show_verification(self, H):
        """호모그래피 그리드를 원본 프레임에 오버레이"""
        if self.frame_clean is None:
            return
        display = self.frame_clean.copy()
        H_inv = np.linalg.inv(H)

        for x_mm in range(-200, 201, 50):
            for y_mm in range(-200, 201, 50):
                pt = H_inv @ np.array([x_mm, y_mm, 1.0])
                pt /= pt[2]
                px, py = int(pt[0]), int(pt[1])
                h, w = display.shape[:2]
                if 0 <= px < w and 0 <= py < h:
                    cv2.circle(display, (px, py), 3, (255, 0, 0), -1)
                    cv2.putText(display, f"{x_mm},{y_mm}", (px + 5, py - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 100, 0), 1)

        cv2.imshow("Verification", display)
        print("\n검증 그리드 표시 중 — 아무 키나 누르면 종료")
        cv2.waitKey(0)

    def run(self):
        print("=" * 50)
        print("  호모그래피 캘리브레이션")
        print("=" * 50)
        print()
        print("  1. 작업면에 4개 이상 마커를 놓으세요")
        print("  2. SPACE로 화면 고정")
        print("  3. 마커를 순서대로 클릭 (최소 4개)")
        print("  4. ENTER → 실제 좌표 입력")
        print()

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", self._on_click)

        while True:
            if not self.frozen:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                self.frame = frame.copy()
                self.frame_clean = frame.copy()

            display = self.frame.copy()
            self._draw_hud(display)
            cv2.imshow("Calibration", display)
            key = cv2.waitKey(30) & 0xFF

            if key == ord(" "):
                self.frozen = not self.frozen
                if self.frozen:
                    self.image_points.clear()
                    print("[FREEZE] 화면 고정 — 마커를 클릭하세요")
                else:
                    print("[LIVE] 라이브 모드")
            elif key == 13 and len(self.image_points) >= 4:
                break
            elif key == ord("q"):
                self.cap.release()
                cv2.destroyAllWindows()
                return

        cv2.destroyWindow("Calibration")
        world_points = self._input_world_coords()
        H = self._compute_and_verify(world_points)
        self._show_verification(H)

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="카메라 호모그래피 캘리브레이션")
    parser.add_argument("--camera", type=int, default=CAMERA_ID)
    args = parser.parse_args()

    cal = HomographyCalibrator(camera_id=args.camera)
    cal.run()
