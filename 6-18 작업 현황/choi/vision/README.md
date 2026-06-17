# vision — 노트북(PC) 카메라·YOLO·역기구학

물체 감지(YOLO best.pt) → 호모그래피 → 역기구학(IK)으로 모터 목표각을 계산하는
**노트북(Windows) 쪽** 코드. RPi 키보드 제어(`../rpi/`)와는 별개로 노트북에서 돈다.

## 파일
| 파일 | 용도 |
|---|---|
| `ik.py` | 역기구학 (좌표→모터각). 캘리(기어비) 반영본. `M*_OFFSET/M*_DIR`, `MOTOR_LIMITS` 포함 |
| `detect_and_ik_calib.py` | 카메라+YOLO 감지 → IK각 계산/표시. `C`=IK각 6개 출력+클립보드 복사 |
| `list_cameras.py` | USB 캠 인덱스 찾기 (인덱스 0~5, 기본/DSHOW 백엔드 시도) |

## ⚠️ 레포에 없는 것 — 따로 준비해야 함
같은 폴더(또는 `detect_and_ik_calib.py`의 `BASE_DIR`)에 아래 파일이 있어야 실행됨:
- **`best.pt`** — YOLO 학습 모델 (대용량이라 Git 제외, 별도 공유)
- **`camera_calib.npz`** — 카메라 내부보정 (mtx, dist)
- **`homography_64cm.npz`** — 호모그래피 행렬 (H)

## 실행 (노트북, 아나콘다 환경)
```
python list_cameras.py            # 먼저 USB 캠 인덱스 확인 → detect_and_ik_calib.py CAMERA_INDEX에 반영
python detect_and_ik_calib.py     # 감지 + IK각, C=출력, Q=종료
```
필요 패키지: `opencv-python ultralytics numpy pyperclip`

## 워크플로 (RPi와 함께)
1. 노트북 `detect_and_ik_calib.py` → 물체 감지 → `C` → IK각 6개
2. RPi `../rpi/rpi_keyboard_control_v1_11.py` → `7`(5개) 또는 `1~5`로 그 각도 실행

> 캘리(IK↔실물 보정)는 이 `ik.py` 한 곳에서만. RPi의 k/j/p 회귀는 중복이니 보정 끝났으면 쓰지 말 것(이중보정 주의).
> M2·M3 거울쌍은 명령각 360 대칭(M2+M3=360, 중점 180).
