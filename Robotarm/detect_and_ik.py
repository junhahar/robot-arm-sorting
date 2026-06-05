# -*- coding: utf-8 -*-
"""
detect_and_ik.py
카메라 → YOLO/OpenCV 감지 → 호모그래피 → IK 계산 → 모터 각도 출력

사용법
  C 키 : 현재 감지된 물체의 모터 각도를 터미널에 출력
  Q 키 : 종료
"""
import sys
import cv2
import numpy as np
import time
from pathlib import Path
from collections import deque

sys.stdout.reconfigure(encoding='utf-8')

# ══════════════════════════════════════════
# 경로 설정
# ══════════════════════════════════════════
BASE_DIR   = Path(r"C:\anaconda\envs\bigdata\PROJECT1")
MODEL_PATH = BASE_DIR / "runs" / "bolt_nut_v1" / "weights" / "best.pt"
CALIB_PATH = BASE_DIR / "Robotarm" / "camera_calib.npz"
HOMO_PATH  = BASE_DIR / "Robotarm" / "homography_64cm.npz"

CONF_THRESH = 0.5

# ══════════════════════════════════════════
# IK 모듈 로드
# ══════════════════════════════════════════
sys.path.insert(0, str(BASE_DIR / "Robotarm"))
from ik import solve_ik, homography_to_arm, GRASP_HEIGHT

# ══════════════════════════════════════════
# YOLO 모델 로드
# ══════════════════════════════════════════
try:
    from ultralytics import YOLO
    model = YOLO(str(MODEL_PATH))
    print(f"YOLO 로드 완료  |  클래스: {model.names}")
except Exception as e:
    print(f"[오류] YOLO 로드 실패: {e}")
    sys.exit(1)

# ══════════════════════════════════════════
# 캘리브레이션 / 호모그래피 로드
# ══════════════════════════════════════════
calib    = np.load(CALIB_PATH)
cam_mtx  = calib["mtx"]
cam_dist = calib["dist"]
H        = np.load(HOMO_PATH)["H"]

# ══════════════════════════════════════════
# 카메라 열기
# ══════════════════════════════════════════
CAMERA_INDEX = 1   # USB 카메라 고정

cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print(f"[오류] 카메라 index {CAMERA_INDEX} 를 열 수 없습니다.")
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("카메라 초기화 중...")
time.sleep(2.0)
for _ in range(30):
    cap.read()

fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"해상도: {fw}x{fh}\n")

# ══════════════════════════════════════════
# 마우스 콜백
# ══════════════════════════════════════════
mouse_x, mouse_y = 0, 0

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"[클릭] 픽셀({x}, {y})")

cv2.namedWindow("Detect & IK")
cv2.setMouseCallback("Detect & IK", on_mouse)

# ══════════════════════════════════════════
# 설정
# ══════════════════════════════════════════
CLASS_COLOR   = {0: (0, 165, 255), 1: (0, 255, 0)}   # nut=주황, bolt=초록
angle_history = deque(maxlen=20)

fps_time    = time.time()
fps         = 0.0
frame_count = 0
last_cmd    = None   # 마지막으로 계산된 IK 결과

print("=== 실시간 감지 시작 ===")
print("C: 모터 각도 터미널 출력  |  Q: 종료")

# ══════════════════════════════════════════
# 메인 루프
# ══════════════════════════════════════════
while True:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.05)
        continue

    # 렌즈 왜곡 보정 (호모그래피 캘리브레이션과 동일 조건)
    frame = cv2.undistort(frame, cam_mtx, cam_dist)

    results = model(frame, verbose=False, conf=CONF_THRESH)
    display = frame.copy()
    current_cmd = None

    for box in results[0].boxes:
        cls_id       = int(box.cls)
        conf         = float(box.conf)
        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
        cx, cy       = (x1 + x2) // 2, (y1 + y2) // 2
        label        = model.names[cls_id]
        color        = CLASS_COLOR.get(cls_id, (255, 255, 255))

        # ── 픽셀 → 실제 좌표 (호모그래피) ──────
        pt   = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
        real = cv2.perspectiveTransform(pt, H)[0][0]
        hx, hy = float(real[0]), float(real[1])

        # ── 호모그래피 좌표 → 로봇 팔 좌표 ─────
        x_arm, y_arm = homography_to_arm(hx, hy)

        # ── 화면 표시 ────────────────────────────
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.circle(display, (cx, cy), 5, color, -1)
        cv2.putText(display, f"{label} {conf:.2f}",
                    (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(display, f"real ({hx:.0f}, {hy:.0f}) mm",
                    (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        cv2.putText(display, f"arm  ({x_arm:.0f}, {y_arm:.0f}) mm",
                    (x1, y2 + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        bolt_angle = 0.0

        # ── bolt: OpenCV 각도 계산 ───────────────
        if cls_id == 1:
            roi = frame[y1:y2, x1:x2]
            if roi.size > 0:
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (3, 3), 0)
                _, thresh = cv2.threshold(gray, 0, 255,
                                          cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(largest) > 100:
                        rect = cv2.minAreaRect(largest)
                        rw, rh = rect[1]
                        if min(rw, rh) > 0 and max(rw, rh) / min(rw, rh) >= 1.5:
                            ang = rect[2]
                            if rw < rh:
                                ang += 90
                            ang = ang % 180
                            angle_history.append(ang)
                            angles_rad = np.deg2rad(list(angle_history))
                            bolt_angle = np.rad2deg(
                                np.arctan2(np.mean(np.sin(angles_rad)),
                                           np.mean(np.cos(angles_rad)))
                            ) % 180
                            cv2.putText(display, f"angle {bolt_angle:.1f} deg",
                                        (x1, y2 + 56),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        else:
            angle_history.clear()

        # ── IK 계산 ──────────────────────────────
        try:
            cmd = solve_ik(x_arm, y_arm, bolt_angle)
            current_cmd = dict(label=label, conf=conf,
                               hx=hx, hy=hy, xa=x_arm, ya=y_arm,
                               ba=bolt_angle, cmd=cmd)
        except ValueError as e:
            cv2.putText(display, f"IK: {e}",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ── 우측 상단에 모터 각도 실시간 표시 ────────
    if current_cmd:
        last_cmd = current_cmd
        cmd = current_cmd["cmd"]
        labels = ["", "M1 Base  ", "M2 Shldr ", "M3 Mirror",
                     "M4 Elbow ", "M5 Wrist ", "M6 Roll  "]
        cv2.putText(display, "== Motor Cmd ==",
                    (fw - 290, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        for mid in range(1, 7):
            cv2.putText(display, f"{labels[mid]}: {cmd[mid]:6.1f}",
                        (fw - 290, 28 + mid * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(display, "[ C ] to print",
                    (fw - 290, 28 + 7 * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    # ── FPS / 마우스 ──────────────────────────────
    frame_count += 1
    if frame_count % 10 == 0:
        fps = 10.0 / (time.time() - fps_time)
        fps_time = time.time()

    cv2.line(display, (mouse_x-15, mouse_y), (mouse_x+15, mouse_y), (0,255,255), 1)
    cv2.line(display, (mouse_x, mouse_y-15), (mouse_x, mouse_y+15), (0,255,255), 1)
    cv2.putText(display, f"({mouse_x},{mouse_y})",
                (mouse_x+8, mouse_y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    cv2.putText(display, f"FPS {fps:.1f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)
    cv2.putText(display, "C: print motor cmd  |  Q: Quit",
                (10, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

    cv2.imshow("Detect & IK", display)

    key = cv2.waitKey(1)
    if key == ord('q') or key == ord('Q'):
        break

    elif key == ord('c') or key == ord('C'):
        if last_cmd:
            d = last_cmd
            print("\n" + "━"*42)
            print(f"  감지: {d['label']}  (신뢰도 {d['conf']:.2f})")
            print(f"  실제 좌표 : ({d['hx']:.1f}, {d['hy']:.1f}) mm")
            print(f"  팔 좌표   : ({d['xa']:.1f}, {d['ya']:.1f}) mm")
            if d['label'] == "bolt":
                print(f"  볼트 각도 : {d['ba']:.1f}°")
            print("━"*42)
            print(f"  M1  베이스   : {d['cmd'][1]:7.1f}°")
            print(f"  M2  어깨     : {d['cmd'][2]:7.1f}°")
            print(f"  M3  어깨mir  : {d['cmd'][3]:7.1f}°")
            print(f"  M4  팔꿈치   : {d['cmd'][4]:7.1f}°")
            print(f"  M5  손목pit  : {d['cmd'][5]:7.1f}°")
            print(f"  M6  손목roll : {d['cmd'][6]:7.1f}°")
            print("━"*42 + "\n")
        else:
            print("[!] 감지된 물체 없음 — 물체를 카메라 앞에 놓으세요")

cap.release()
cv2.destroyAllWindows()
print("종료")
