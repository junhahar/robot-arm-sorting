# -*- coding: utf-8 -*-
"""
detect_and_ik_calib.py
카메라 → YOLO 감지 → 호모그래피 → IK 각도 (캘리브 비교용, 모터엔 안 보냄)

원본 detect_and_ik.py에서 바뀐 점:
  · C 키 = 선택된 1개 물체의 IK각 6개를 화면에 크게 + 클립보드 복사 + 파일 저장.
    → 그 6개를 RPi rpi_keyboard_control_v1_9.py 에서 k로 입력해 캘리브 샘플로 쓴다.
  · 여러 물체 중 '신뢰도 최고' 1개만 선택 (랜덤 마지막 박스 문제 해결).
  · bolt 클래스 인덱스를 model.names 에서 자동 탐색 (인덱스/문자열 하드코딩 제거).
  · 어깨~손목 거리 D 와 도달 가능 여부(REACHABLE/OUT) 화면 표시.
  · 볼트 각도 deque를 선택된 물체에만 사용.

키: C = IK각 출력/복사/저장,  Q = 종료
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
# BASE_DIR = 이 스크립트가 있는 폴더. 아래 파일들을 전부 같은 폴더에 두면
#   폴더를 어디에 두든(이름이 뭐든) 경로를 다시 안 고쳐도 된다.
#   필요한 파일(같은 폴더): best.pt, ik.py, camera_calib.npz, homography_64cm.npz
BASE_DIR   = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "best.pt"
CALIB_PATH = BASE_DIR / "camera_calib.npz"
HOMO_PATH  = BASE_DIR / "homography_64cm.npz"

CONF_THRESH = 0.5
IK_LOG_PATH = BASE_DIR / "ik_angles_log.txt"   # C 누를 때마다 append

# ══════════════════════════════════════════
# IK 모듈 로드 (같은 폴더의 ik.py)
# ══════════════════════════════════════════
sys.path.insert(0, str(BASE_DIR))
# ik.py의 H(바닥~어깨 높이)는 아래 호모그래피 행렬 H_HOMO와 글자가 겹쳐 헷갈리므로
# 가져오면서 ARM_H 로 이름을 바꾼다. MIN_REACH도 도달 하한 표시에 쓰려고 추가 import.
from ik import (solve_ik, homography_to_arm, GRASP_HEIGHT,
                H as ARM_H, L1, L2, L3, MAX_REACH, MIN_REACH)

# ══════════════════════════════════════════
# 클립보드 (실패해도 동작) — RPi에 옮겨적기 편하게
# ══════════════════════════════════════════
def copy_clipboard(text):
    try:
        import pyperclip
        pyperclip.copy(text)
        return True
    except Exception:
        try:
            import subprocess
            # check=False: clip 실패해도 예외 던지지 말고 아래 return True를 건너뛰지 않게
            # (어차피 바깥 except가 잡지만, 조용한 폴백이 의도라 False가 맞음)
            subprocess.run("clip", input=text.encode("utf-16le"), check=False)  # Windows clip
            return True
        except Exception:
            return False

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

# bolt 클래스 인덱스 자동 탐색 (이름에 'bolt' 들어간 것). 없으면 None.
BOLT_ID = next((k for k, v in model.names.items() if "bolt" in str(v).lower()), None)
if BOLT_ID is None:
    print("[경고] 클래스 이름에 'bolt'가 없음 — 볼트 각도 계산 비활성. names 확인 필요")
else:
    print(f"bolt 클래스 인덱스 = {BOLT_ID} ('{model.names[BOLT_ID]}')")

# ══════════════════════════════════════════
# 캘리브 / 호모그래피 / 카메라
# ══════════════════════════════════════════
calib    = np.load(CALIB_PATH)
cam_mtx  = calib["mtx"]
cam_dist = calib["dist"]
H_HOMO   = np.load(HOMO_PATH)["H"]

CAMERA_INDEX = 1
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
# 일부 USB캠/백엔드는 카메라가 늦게 열리면 0을 반환한다. 0이면 (0,0) 맵이 만들어져
# 메인 루프 cv2.remap에서 크래시 → 요청 해상도로 폴백.
if fw == 0 or fh == 0:
    print(f"[경고] 해상도 0 반환({fw}x{fh}) — 1280x720으로 폴백")
    fw, fh = 1280, 720
print(f"해상도: {fw}x{fh}\n")

# undistort 맵 미리 계산 (매 프레임 undistort보다 빠름)
mapx, mapy = cv2.initUndistortRectifyMap(
    cam_mtx, cam_dist, None, cam_mtx, (fw, fh), cv2.CV_16SC2)

# ══════════════════════════════════════════
# 마우스 콜백 (픽셀 좌표 확인용)
# ══════════════════════════════════════════
mouse_x, mouse_y = 0, 0
def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"[클릭] 픽셀({x}, {y})")

cv2.namedWindow("Detect & IK (calib)")
cv2.setMouseCallback("Detect & IK (calib)", on_mouse)

# ══════════════════════════════════════════
CLASS_COLOR   = {0: (0, 165, 255), 1: (0, 255, 0)}
angle_history = deque(maxlen=20)
prev_bolt_track = False    # 직전 프레임에 볼트를 추적 중이었나 (deque 리셋 판단)

fps_time, fps, frame_count = time.time(), 0.0, 0
last_cmd = None
labels6 = ["", "M1 Base  ", "M2 Shldr ", "M3 Mirror",
           "M4 Elbow ", "M5 Wrist ", "M6 Roll  "]

print("=== 실시간 감지 시작 (캘리브 비교용 — 모터엔 안 보냄) ===")
print("C: IK각 6개 출력+클립보드 복사+파일 저장  |  Q: 종료")

while True:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.05)
        continue

    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)   # 렌즈 왜곡 보정(빠름)
    results = model(frame, verbose=False, conf=CONF_THRESH)
    display = frame.copy()

    # ── 신뢰도 최고 박스 1개 선택 ───────────────────────────
    best = None
    for box in results[0].boxes:
        conf = float(box.conf)
        if best is None or conf > best["conf"]:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            best = {"cls": int(box.cls), "conf": conf,
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2}

    current_cmd = None

    if best is not None:
        cls_id = best["cls"]
        conf   = best["conf"]
        x1, y1, x2, y2 = best["x1"], best["y1"], best["x2"], best["y2"]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        label  = model.names[cls_id]
        color  = CLASS_COLOR.get(cls_id, (255, 255, 255))

        # 픽셀 → 실제 좌표(호모그래피) → 팔 좌표
        pt   = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
        real = cv2.perspectiveTransform(pt, H_HOMO)[0][0]
        hx, hy = float(real[0]), float(real[1])
        x_arm, y_arm = homography_to_arm(hx, hy)

        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        cv2.circle(display, (cx, cy), 5, color, -1)
        cv2.putText(display, f"{label} {conf:.2f}", (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(display, f"real ({hx:.0f}, {hy:.0f}) mm", (x1, y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        cv2.putText(display, f"arm  ({x_arm:.0f}, {y_arm:.0f}) mm", (x1, y2 + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        # ── bolt 각도 (선택된 물체가 볼트일 때만) ──
        bolt_angle = 0.0
        is_bolt = (BOLT_ID is not None and cls_id == BOLT_ID)
        if is_bolt:
            if not prev_bolt_track:
                angle_history.clear()       # 새 볼트 추적 시작 → 이력 초기화
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
                            bolt_angle = np.rad2deg(np.arctan2(
                                np.mean(np.sin(angles_rad)),
                                np.mean(np.cos(angles_rad)))) % 180
                            cv2.putText(display, f"angle {bolt_angle:.1f} deg",
                                        (x1, y2 + 56),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        prev_bolt_track = is_bolt

        # ── 도달거리 D 표시 + IK 계산 ──
        r = np.sqrt(x_arm ** 2 + y_arm ** 2)
        z_w = GRASP_HEIGHT + L3 - ARM_H          # 어깨 기준 손목 높이 (ARM_H=바닥~어깨)
        D = np.sqrt(r ** 2 + z_w ** 2)
        # 최대(MAX_REACH)뿐 아니라 최소(MIN_REACH)도 체크 — 너무 가까우면 solve_ik가
        # ValueError를 던지므로, 표시와 실제 결과가 어긋나지 않게 둘 다 본다.
        reach_ok = MIN_REACH <= D <= MAX_REACH
        if D > MAX_REACH:
            reach_txt = "OUT (too far)"
        elif D < MIN_REACH:
            reach_txt = "OUT (too close)"
        else:
            reach_txt = "REACHABLE"
        cv2.putText(display, f"D={D:.0f}mm  {reach_txt}",
                    (x1, y2 + 74), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if reach_ok else (0, 0, 255), 1)

        try:
            cmd = solve_ik(x_arm, y_arm, bolt_angle)
            current_cmd = dict(label=label, conf=conf, hx=hx, hy=hy,
                               xa=x_arm, ya=y_arm, ba=bolt_angle,
                               D=D, is_bolt=is_bolt, cmd=cmd)
        except ValueError as e:
            cv2.putText(display, f"IK: {e}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        prev_bolt_track = False

    # ── 우측 상단 모터 각도 패널 ──
    if current_cmd:
        last_cmd = current_cmd
        cmd = current_cmd["cmd"]
        cv2.putText(display, "== Motor Cmd (IK) ==", (fw - 300, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        for mid in range(1, 7):
            cv2.putText(display, f"{labels6[mid]}: {cmd[mid]:6.1f}",
                        (fw - 300, 28 + mid * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(display, "[ C ] copy to clipboard", (fw - 300, 28 + 7 * 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    # ── FPS / 마우스 십자선 ──
    frame_count += 1
    if frame_count % 10 == 0:
        fps = 10.0 / (time.time() - fps_time)
        fps_time = time.time()
    cv2.line(display, (mouse_x - 15, mouse_y), (mouse_x + 15, mouse_y), (0, 255, 255), 1)
    cv2.line(display, (mouse_x, mouse_y - 15), (mouse_x, mouse_y + 15), (0, 255, 255), 1)
    cv2.putText(display, f"FPS {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    cv2.putText(display, "C: copy IK angles  |  Q: Quit", (10, fh - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.imshow("Detect & IK (calib)", display)

    key = cv2.waitKey(1)
    if key in (ord('q'), ord('Q')):
        break
    elif key in (ord('c'), ord('C')):
        if last_cmd:
            d = last_cmd
            c = d["cmd"]
            # RPi v1_9 의 k 입력에 그대로 붙여넣을 한 줄 (M1..M6 공백구분)
            one_line = " ".join(f"{c[m]:.1f}" for m in range(1, 7))
            print("\n" + "━" * 46)
            print(f"  감지: {d['label']} (신뢰도 {d['conf']:.2f})  D={d['D']:.0f}mm")
            print(f"  팔 좌표: ({d['xa']:.1f}, {d['ya']:.1f}) mm", end="")
            if d["is_bolt"]:
                print(f"   볼트각 {d['ba']:.1f}°")
            else:
                print()
            print("━" * 46)
            for mid in range(1, 7):
                print(f"  {labels6[mid]}: {c[mid]:7.1f}°")
            print("━" * 46)
            print(f"  ▶ RPi v1_9 k 입력용 한 줄:  {one_line}")
            ok = copy_clipboard(one_line)
            print(f"  ▶ 클립보드 복사 {'성공' if ok else '실패(수동 복사)'}")
            try:
                with open(IK_LOG_PATH, "a", encoding="utf-8") as f:
                    f.write(f"{time.strftime('%H:%M:%S')}  {d['label']}  "
                            f"arm({d['xa']:.1f},{d['ya']:.1f})  IK: {one_line}\n")
                print(f"  ▶ 파일 기록: {IK_LOG_PATH.name}")
            except Exception as e:
                print(f"  ▶ 파일 기록 실패: {e}")
            print("━" * 46 + "\n")
        else:
            print("[!] 감지된 물체 없음 — 물체를 카메라 앞에 놓으세요")

cap.release()
cv2.destroyAllWindows()
print("종료")
