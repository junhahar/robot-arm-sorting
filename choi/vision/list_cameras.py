# -*- coding: utf-8 -*-
"""
list_cameras.py — USB 캠 인덱스 찾기
====================================
인덱스 0~5를 열어보고, 실제로 프레임을 주는 카메라와 해상도를 표시한다.
Windows 기본 백엔드와 DSHOW 백엔드 둘 다 시도.

실행: python list_cameras.py
조작:
  · 열리는 카메라마다 미리보기 창이 뜬다. 어떤 게 'USB 캠 화면'인지 눈으로 확인.
  · 창 제목에 인덱스/백엔드가 적혀 있다 → USB 캠으로 보이는 창의 인덱스를 기억.
  · 아무 창에서 ESC 또는 q = 다음 카메라로, 전부 끝나면 종료.
  · 그 인덱스를 detect_and_ik_calib.py 의 CAMERA_INDEX 에 넣으면 됨.
     (DSHOW가 필요했으면 VideoCapture(N, cv2.CAP_DSHOW) 로 바꾸기)
"""
import cv2

MAX_INDEX = 5
BACKENDS = [("기본", cv2.CAP_ANY), ("DSHOW", cv2.CAP_DSHOW)]

print("=" * 56)
print(" 카메라 검색 시작 (인덱스 0~%d, 기본/DSHOW 백엔드)" % MAX_INDEX)
print("=" * 56)

found = []

for idx in range(MAX_INDEX + 1):
    for bname, bflag in BACKENDS:
        cap = cv2.VideoCapture(idx, bflag)
        if not cap.isOpened():
            cap.release()
            continue

        # 워밍업 (USB캠 초기 검은 프레임 버림)
        ok = False
        frame = None
        for _ in range(10):
            ret, f = cap.read()
            if ret and f is not None:
                ok, frame = True, f
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if ok:
            tag = f"index {idx} [{bname}]"
            print(f"  ✅ {tag}  →  프레임 OK, 해상도 {w}x{h}")
            found.append((idx, bname, w, h))

            win = f"cam {idx} ({bname}) - ESC/q = next"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            print(f"     미리보기 창 '{win}' — USB캠 화면인지 눈으로 확인, ESC/q로 다음")
            while True:
                ret, f = cap.read()
                if not ret or f is None:
                    break
                label = f"index={idx}  backend={bname}  {w}x{h}"
                cv2.putText(f, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 255), 2)
                cv2.imshow(win, f)
                k = cv2.waitKey(1)
                if k in (27, ord('q'), ord('Q')):
                    break
            cv2.destroyWindow(win)
        else:
            print(f"  ⚠ index {idx} [{bname}] : 열렸지만 프레임 못 받음 (해상도 {w}x{h})")
        cap.release()

print("=" * 56)
if found:
    print(" 사용 가능한 카메라:")
    for idx, bname, w, h in found:
        note = "" if bname == "기본" else "  ← VideoCapture(N, cv2.CAP_DSHOW) 필요"
        print(f"   index {idx} [{bname}]  {w}x{h}{note}")
    print("\n 위에서 'USB 캠 화면'이 보였던 인덱스를 detect_and_ik_calib.py 의")
    print("   CAMERA_INDEX = N  에 넣으세요.")
else:
    print(" ❌ 열리는 카메라가 없습니다. USB 연결/드라이버/다른 앱 점유 확인.")
print("=" * 56)
cv2.destroyAllWindows()
