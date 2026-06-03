import cv2
import os
import time

BOLT_LIE_CLASSES = [
    "bolt_lie_000_015",
    "bolt_lie_015_030",
    "bolt_lie_030_045",
    "bolt_lie_045_060",
    "bolt_lie_060_075",
    "bolt_lie_075_090",
    "bolt_lie_090_105",
    "bolt_lie_105_120",
    "bolt_lie_120_135",
    "bolt_lie_135_150",
    "bolt_lie_150_165",
    "bolt_lie_165_180",
]

def capture_dataset(class_name, target_count, interval=1.0):
    save_dir = f"dataset/raw/{class_name}"
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    count = len(os.listdir(save_dir))
    print(f"\n{'='*40}")
    print(f"클래스: {class_name}")
    print(f"현재: {count}장 / 목표: {target_count}장")
    print(f"Space: 시작/정지  Q: 다음 클래스")
    print(f"{'='*40}")

    capturing = False
    last_save = 0

    while count < target_count:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()
        now = time.time()

        if capturing and (now - last_save) >= interval:
            path = f"{save_dir}/{class_name}_{count:04d}.jpg"
            cv2.imwrite(path, frame)
            count += 1
            last_save = now
            print(f"  저장: {count}/{target_count}")

        status = "촬영 중" if capturing else "일시정지"
        color  = (0, 255, 0) if capturing else (0, 165, 255)

        cv2.putText(display, f"{class_name}",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(display, f"{count}/{target_count}  [{status}]",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.rectangle(display, (0, 0), (1279, 719), color, 2)
        cv2.imshow("Capture", display)

        key = cv2.waitKey(1)
        if key == ord(' '):
            capturing = not capturing
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[{class_name}] 완료: {count}장")


# ── 실행 ──────────────────────────────────────────
capture_dataset("nut", 200, interval=1.0)

for cls in BOLT_LIE_CLASSES:
    capture_dataset(cls, 50, interval=1.0)
