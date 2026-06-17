# -*- coding: utf-8 -*-
"""detect_pose_hailo.py — HailoRT 추론 버전 (RPi + Hailo-8L 전용).

detect_pose.py와 동일한 로직, YOLO 추론만 HailoRT(.hef)로 교체.
노트북 테스트는 detect_pose.py(ultralytics), RPi 배포는 이 파일 사용.

준비: calib/best_pose_512.hef  (hailo compiler 출력)
키: A=오토 | S=RPi전송(median) | C=IK각 복사 | Q=종료
"""
import sys
import json
import socket
import time
from collections import deque
from pathlib import Path

import platform
import threading
from queue import Queue

import cv2
import numpy as np

_IS_RPI = platform.system() == "Linux"

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE.parent))
from ik_pose import (solve_pick, check_limits, homography_to_arm,  # noqa: E402
                     _local_offset, GRASP_HEIGHT, L3, H as ARM_H, MAX_REACH, MIN_REACH, M1_GAIN)

MODEL_PATH = BASE / "best_pose_512.hef"
CALIB_PATH = BASE / "camera_calib.npz"
HOMO_PATH = BASE / "homography_64cm.npz"
import os  # noqa: E402
# 대시보드 연동 비전 상태(계약 스키마). 팀원이 SAMBO_VISION_STATE_PATH로 경로 지정 가능
VISION_STATE_PATH = Path(os.environ.get("SAMBO_VISION_STATE_PATH", str(BASE / "sambo_vision_state.json")))
BUSY_FLAG = VISION_STATE_PATH.parent / "robot_busy.flag"   # 브리지가 잡기 중이면 생성 → 인식 정지
CNC_BUSY_FLAG = VISION_STATE_PATH.parent / "cnc_busy.flag"  # CNC 가공중이면 생성 → 파이프 제외(볼트/너트만)
RUN_FLAG = VISION_STATE_PATH.parent / "run.flag"           # 있으면 자동탐색(없으면 sweep, 있으면 센터링). SWEEP버튼/START가 켬
GRASP_LOG = BASE / "grasp_log.jsonl"      # 잡으러 보낼 때마다 1줄 append
SCHEMA_V = 1                              # 계약 스키마 버전
STABLE_TOL_MM = 15.0                      # 안정 판정 위치 허용오차(mm)
STABLE_CONF = 0.5                         # 안정 판정 최소 신뢰도
RPI_HOST, RPI_PORT = ("127.0.0.1" if _IS_RPI else "172.30.1.26"), 5005
CONF = 0.4  # 임시(테스트). 검출 확인 후 0.6~0.7로 상향
CAMERA_INDEX = 0
ANG_OFFSET = 0.0
GRASP_T = 0.50
IMGSZ = 512

SCAN_M1 = 180.0
M1_STEP = 12.0
M1_ROT_SIGN = +1.0
CENTER_DEADBAND = 45
CENTER_KP = 0.035
CENTER_MAX_STEP = 18.0
SEARCH_STEP = 8.0
SEARCH_WAIT = 8
SWEEP_SEARCH = True
AUTO_COOLDOWN = 13.0

USE_CUSTOM_REGION = True
WORK_POLY_PX = np.array([[195, 55], [1043, 55], [1043, 645], [195, 645]], np.int32)
RELIABLE_R_MIN, RELIABLE_R_MAX = 180.0, 300.0
BOLT_ONLY = False
AUTO_ENABLED = False   # ★ 자동 잡기 비활성화. 키보드/웹 'a' 무시. 나중에 의도적으로 켤 때 True
AUTO_STABLE_N = 5
AUTO_GRAB_MIN = 0.6
AUTO_SIGMA_MAX = 2.0

# ── HailoRT 추론 셋업 ──
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams,  # noqa: E402
                            ConfigureParams, InputVStreamParams, OutputVStreamParams,
                            FormatType)

CLASS_NAMES = {0: "nut", 1: "bolt", 2: "pipe"}
NUM_CLASSES = 3
NUM_KPT = 2
IOU_THRESH = 0.45


# ── ultralytics 호환 래퍼 (기존 코드 수정 최소화) ──
class _NP:
    """numpy 배열을 .cpu().numpy() 체인으로 접근 가능하게."""
    def __init__(self, a):
        self._a = np.asarray(a)
    def cpu(self):
        return self
    def numpy(self):
        return self._a
    def __len__(self):
        return len(self._a)


class _Boxes:
    def __init__(self, xyxy, cls, conf):
        self.xyxy, self.cls, self.conf = _NP(xyxy), _NP(cls), _NP(conf)
    def __len__(self):
        return len(self.xyxy._a)


class _Kpts:
    def __init__(self, xy):
        self.xy = _NP(xy)
    def __len__(self):
        return len(self.xy._a)


class _HailoResult:
    def __init__(self, xyxy, cls, conf, kpt_xy):
        self.boxes = _Boxes(xyxy, cls, conf) if len(xyxy) > 0 else None
        self.keypoints = _Kpts(kpt_xy) if kpt_xy is not None and len(kpt_xy) > 0 else None


def _nms(boxes, scores, iou_thresh):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return np.array(keep, dtype=int)


def _letterbox(frame):
    """카메라 프레임 → 512×512 letterbox (비율 유지, 회색 패딩)."""
    h, w = frame.shape[:2]
    scale = IMGSZ / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    resized = cv2.resize(frame, (nw, nh))
    pad_w, pad_h = (IMGSZ - nw) // 2, (IMGSZ - nh) // 2
    padded = np.full((IMGSZ, IMGSZ, 3), 114, dtype=np.uint8)
    padded[pad_h:pad_h + nh, pad_w:pad_w + nw] = resized
    return padded, scale, pad_w, pad_h


def _build_anchors(imgsz=IMGSZ, strides=(8, 16, 32)):
    """YOLOv8 앵커 그리드(그리드 단위 중심) + 앵커별 stride 생성."""
    pts, strd = [], []
    for s in strides:
        n = imgsz // s
        sx = np.arange(n, dtype=np.float32) + 0.5
        sy = np.arange(n, dtype=np.float32) + 0.5
        gy, gx = np.meshgrid(sy, sx, indexing="ij")
        pts.append(np.stack([gx.ravel(), gy.ravel()], axis=1))
        strd.append(np.full(n * n, s, dtype=np.float32))
    return np.concatenate(pts, 0), np.concatenate(strd, 0)


_ANCHORS, _STRIDES = _build_anchors()  # (5376,2), (5376,)
_DFL_PROJ = np.arange(16, dtype=np.float32)


def _softmax(x, axis):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


_debug_first = True
_debug_box = True
_debug_all = True


def _postprocess(raw, orig_w, orig_h, scale, pad_w, pad_h):
    """HailoRT raw output → _HailoResult (ultralytics 호환).

    Hailo 출력 (채널 합 73 = YOLOv8-pose 헤드):
      concat14(64)=박스 DFL, activation1(3)=cls,
      mul_and_add1(4)+activation2(2)=키포인트(좌표4+신뢰도2).
    박스는 DFL을 직접 디코딩, 키포인트는 앵커+stride로 복원.
    """
    global _debug_first

    dfl_arr = cls_arr = kpt_xy_arr = kpt_cf_arr = None
    for info in _output_infos:
        t = np.squeeze(raw[info.name])
        ch = t.shape[-1]
        if ch == 64:
            dfl_arr = t.reshape(-1, 64)
        elif ch == NUM_CLASSES:
            cls_arr = t.reshape(-1, NUM_CLASSES)
        elif ch == NUM_KPT * 2:
            kpt_xy_arr = t.reshape(-1, NUM_KPT, 2)
        elif ch == NUM_KPT:
            kpt_cf_arr = t.reshape(-1, NUM_KPT)

    if _debug_first:
        _debug_first = False
        print(f"[DEBUG] dfl={None if dfl_arr is None else dfl_arr.shape} "
              f"cls={None if cls_arr is None else cls_arr.shape} "
              f"kptxy={None if kpt_xy_arr is None else kpt_xy_arr.shape}")
        if cls_arr is not None:
            print(f"[DEBUG] cls max={cls_arr.max():.3f}  >{CONF}: {(cls_arr.max(1) >= CONF).sum()}개")

    empty = _HailoResult(np.empty((0, 4)), np.empty(0), np.empty(0), np.empty((0, NUM_KPT, 2)))
    if dfl_arr is None or cls_arr is None:
        return empty

    cls_id = cls_arr.argmax(axis=1)
    cls_conf = cls_arr.max(axis=1)
    mask = cls_conf >= CONF
    if not mask.any():
        return empty

    ax = _ANCHORS[mask, 0]
    ay = _ANCHORS[mask, 1]
    st = _STRIDES[mask]
    cls_id, cls_conf = cls_id[mask], cls_conf[mask]

    # ── DFL 박스 디코딩 → 512 letterbox 픽셀 xyxy ──
    dist = (_softmax(dfl_arr[mask].reshape(-1, 4, 16), axis=2) * _DFL_PROJ).sum(axis=2)  # (N,4) ltrb
    x1 = (ax - dist[:, 0]) * st
    y1 = (ay - dist[:, 1]) * st
    x2 = (ax + dist[:, 2]) * st
    y2 = (ay + dist[:, 3]) * st
    boxes = np.stack([x1, y1, x2, y2], axis=1)

    # ── 키포인트 디코딩 (앵커+stride). ×2는 HEF의 mul_and_add 레이어가 이미 적용 → 중복금지 ──
    if kpt_xy_arr is not None:
        kr = kpt_xy_arr[mask]                       # (N,2,2)
        kx = (kr[:, :, 0] + (ax[:, None] - 0.5)) * st[:, None]
        ky = (kr[:, :, 1] + (ay[:, None] - 0.5)) * st[:, None]
        kpt512 = np.stack([kx, ky], axis=2)         # (N,2,2)
    else:
        kpt512 = None

    global _debug_box
    if _debug_box and len(boxes) > 0:
        _debug_box = False
        _bi = cls_conf.argmax()
        print(f"[BOX] top conf={cls_conf[_bi]:.3f} cls={cls_id[_bi]} "
              f"anchor=({ax[_bi]:.1f},{ay[_bi]:.1f}) stride={st[_bi]:.0f} "
              f"dist_ltrb={dist[_bi].round(2)}")
        print(f"[BOX] 512px xyxy={boxes[_bi].round(1)}  (이미지 0~512 범위여야 정상)")
        if kpt_xy_arr is not None:
            print(f"[KPT] raw={kpt_xy_arr[mask][_bi].round(3).tolist()}")
            print(f"[KPT] 512px={kpt512[_bi].round(1).tolist()}  (박스 양끝 근처여야 정상)")

    keep = _nms(boxes, cls_conf, IOU_THRESH)
    boxes, cls_id, cls_conf = boxes[keep], cls_id[keep], cls_conf[keep]
    if kpt512 is not None:
        kpt512 = kpt512[keep]

    # ── letterbox → 원본 프레임 좌표 ──
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, orig_w)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, orig_h)

    global _debug_all
    if _debug_all and len(boxes) > 0:
        _debug_all = False
        print(f"[ALL] NMS후 {len(boxes)}개 (frame {orig_w}x{orig_h}):")
        for j in range(len(boxes)):
            cx_d = (boxes[j, 0] + boxes[j, 2]) / 2
            cy_d = (boxes[j, 1] + boxes[j, 3]) / 2
            wd = boxes[j, 2] - boxes[j, 0]
            hd = boxes[j, 3] - boxes[j, 1]
            print(f"  cls={cls_id[j]} conf={cls_conf[j]:.2f} "
                  f"center=({cx_d:.0f},{cy_d:.0f}) wh=({wd:.0f}x{hd:.0f})")

    kpt_xy = np.zeros((len(boxes), NUM_KPT, 2))
    for j in range(len(boxes)):
        x1, y1, x2, y2 = boxes[j]
        use_decoded = False
        if kpt512 is not None:
            k = kpt512[j].copy()
            k[:, 0] = (k[:, 0] - pad_w) / scale
            k[:, 1] = (k[:, 1] - pad_h) / scale
            mx = 0.5 * (x2 - x1) + 20            # 박스 밖으로 너무 벗어나면 폐기
            my = 0.5 * (y2 - y1) + 20
            inside = ((k[:, 0] >= x1 - mx) & (k[:, 0] <= x2 + mx)
                      & (k[:, 1] >= y1 - my) & (k[:, 1] <= y2 + my)).all()
            if inside:
                kpt_xy[j] = k
                use_decoded = True
        if not use_decoded:                      # fallback: 박스 장축 양끝
            bw, bh = x2 - x1, y2 - y1
            cx_b, cy_b = (x1 + x2) / 2, (y1 + y2) / 2
            if bw >= bh:
                kpt_xy[j, 0], kpt_xy[j, 1] = [x1, cy_b], [x2, cy_b]
            else:
                kpt_xy[j, 0], kpt_xy[j, 1] = [cx_b, y1], [cx_b, y2]

    return _HailoResult(boxes, cls_id.astype(int), cls_conf, kpt_xy)


# ── HailoRT 초기화 ──
hef = HEF(str(MODEL_PATH))
_target = VDevice()
_cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
_network_group = _target.configure(hef, _cfg)[0]
_input_params = InputVStreamParams.make(_network_group, quantized=False, format_type=FormatType.FLOAT32)
_output_params = OutputVStreamParams.make(_network_group, quantized=False, format_type=FormatType.FLOAT32)
_input_infos = hef.get_input_vstream_infos()
_output_infos = hef.get_output_vstream_infos()
_input_name = _input_infos[0].name

print(f"HailoRT 모델 로드: {MODEL_PATH.name}")
print(f"  input: {_input_name}  shape: {_input_infos[0].shape}")
for info in _output_infos:
    print(f"  output: {info.name}  shape: {info.shape}")
print(f"클래스: {CLASS_NAMES}")

# ── 이미지 단독 테스트(박스 그려 저장): python3 detect_pose_hailo.py --image <경로> ──
if len(sys.argv) >= 3 and sys.argv[1] == "--image":
    _img = cv2.imread(sys.argv[2])
    if _img is None:
        print("이미지 못 읽음:", sys.argv[2]); sys.exit(1)
    _ih, _iw = _img.shape[:2]
    _lb, _sc, _pw, _ph = _letterbox(_img)
    _rgb = cv2.cvtColor(_lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    with _network_group.activate(), InferVStreams(_network_group, _input_params, _output_params) as _pp:
        _raw = _pp.infer({_input_name: np.expand_dims(_rgb, 0)})
    _res = _postprocess(_raw, _iw, _ih, _sc, _pw, _ph)
    if _res.boxes is not None:
        for _i in range(len(_res.boxes)):
            _b = _res.boxes.xyxy._a[_i].astype(int)
            _c = int(_res.boxes.cls._a[_i])
            cv2.rectangle(_img, (_b[0], _b[1]), (_b[2], _b[3]), (0, 255, 0), 2)
            cv2.putText(_img, CLASS_NAMES.get(_c, "?"), (_b[0], _b[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imwrite("test_out.jpg", _img)
    print(f"결과 저장: test_out.jpg  (입력 {_iw}x{_ih})")
    sys.exit(0)

BOLT_ID = 1
NUT_ID = 0
PIPE_ID = 2
INFER_EVERY = 3
infer_cnt = 0
_last_res = None

calib = np.load(CALIB_PATH)
mtx, dist = calib["mtx"], calib["dist"]
H_HOMO = np.load(HOMO_PATH)["H"]

USE_SIZE_GATE = True
SIZE_RANGE_MM = {PIPE_ID: (80.0, 250.0)}


def _px_to_arm_mm(px, py):
    real = cv2.perspectiveTransform(np.array([[[float(px), float(py)]]], np.float32), H_HOMO)[0][0]
    return homography_to_arm(float(real[0]), float(real[1]))


def circular_median_180(angles):
    if not len(angles):
        return 0.0
    a = np.deg2rad(np.asarray(angles, float) * 2.0)
    mean = np.arctan2(np.sin(a).mean(), np.cos(a).mean())
    dev = np.angle(np.exp(1j * (a - mean)))
    return float((np.rad2deg(mean + np.median(dev)) / 2.0) % 180.0)


def copy_clipboard(text):
    try:
        import pyperclip
        pyperclip.copy(text)
        return True
    except Exception:
        try:
            import subprocess
            subprocess.run("clip", input=text.encode("utf-16le"), check=False)
            return True
        except Exception:
            return False


def send_to_rpi(d, x=None, y=None, ba=None):
    req = {"x": round(float(d["xa"] if x is None else x), 1),
           "y": round(float(d["ya"] if y is None else y), 1),
           "bolt": round(float(d["ba"] if ba is None else ba), 1),
           "is_bolt": bool(d["is_bolt"]), "is_pipe": bool(d.get("is_pipe", False)), "near": bool(d["near"])}
    try:
        with socket.create_connection((RPI_HOST, RPI_PORT), timeout=8) as s:
            s.sendall(json.dumps(req).encode())
            print(f"\n  ▶ RPi 전송: {req}\n  ▶ 응답: {s.recv(4096).decode()}\n")
    except Exception as e:
        print(f"\n  ▶ 전송 실패: {e}  (RPI_HOST={RPI_HOST})\n")


# ── MJPEG 스트리밍 + 웹 제어 ──
from flask import Flask, Response, jsonify, request  # noqa: E402

_latest_frame = None
_frame_lock = threading.Lock()
_cmd_q = Queue()
_STREAM_PORT = 8091

_HTML = """\
<!DOCTYPE html><html><head><meta charset="utf-8"><title>Pose Detect</title>
<style>
body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:20px}
img{max-width:100%;border:1px solid #333;border-radius:8px}
.btn{padding:12px 24px;margin:6px;font-size:16px;border:none;border-radius:6px;cursor:pointer;color:#fff}
.g{background:#4CAF50} .b{background:#2196F3} .o{background:#FF9800} .r{background:#f44336} .m{background:#607D8B}
</style></head><body>
<h2>Pose Detect (HailoRT)</h2>
<img src="/video_feed"><br>
<div style="margin:16px">
 <button class="btn b" onclick="api('auto')">A: Auto</button>
 <button class="btn g" onclick="api('send')">S: Send</button>
 <button class="btn o" onclick="api('home')">0: Home</button>
 <button class="btn r" onclick="api('quit')">Q: Quit</button>
</div><div>
 <button class="btn m" onclick="api('m1left')">J: M1←</button>
 <button class="btn m" onclick="api('m1reset')">K: Reset</button>
 <button class="btn m" onclick="api('m1right')">L: M1→</button>
</div>
<script>function api(c){fetch('/api/'+c,{method:'POST'}).then(r=>r.json()).then(d=>console.log(d))}</script>
</body></html>"""

_flask_app = Flask(__name__)
import logging  # noqa: E402
logging.getLogger('werkzeug').setLevel(logging.ERROR)

def _mjpeg_response():
    def _gen():
        while True:
            with _frame_lock:
                f = _latest_frame
            if f is None:
                time.sleep(0.05)
                continue
            fs = cv2.resize(f, (640, 360))   # 스트림만 축소(WiFi 대역폭↓). 검출은 풀해상도 유지
            ok, jpg = cv2.imencode('.jpg', fs, [cv2.IMWRITE_JPEG_QUALITY, 55])
            if not ok:
                continue
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n'
            time.sleep(0.07)
    return Response(_gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@_flask_app.route('/')
def _index():
    if request.args.get('action') == 'stream':   # 대시보드가 기대하는 mjpg_streamer 호환 경로
        return _mjpeg_response()
    return _HTML

@_flask_app.route('/video_feed')
def _video_feed():
    return _mjpeg_response()

@_flask_app.route('/api/<cmd>', methods=['POST'])
def _api(cmd):
    _map = {'auto': ord('a'), 'send': ord('s'), 'home': ord('0'),
            'quit': ord('q'), 'm1left': ord('j'), 'm1right': ord('l'),
            'm1reset': ord('k'), 'copy': ord('c')}
    if cmd in _map:
        _cmd_q.put(_map[cmd])
        return jsonify(ok=True, cmd=cmd)
    return jsonify(ok=False), 400

threading.Thread(target=lambda: _flask_app.run(host='0.0.0.0', port=_STREAM_PORT, threaded=True),
                 daemon=True).start()
print(f"[웹] http://0.0.0.0:{_STREAM_PORT} 시작됨 (카메라 준비 중...)")


def _stdin_reader():
    """터미널에서 키 입력 → 명령 큐 (클릭 없이 키보드로 조작). 키 + Enter.
    안전상 오토(a)·M1회전은 키보드에서 제외 — 's'(전송) '0'(홈) 'q'(종료)만.
    """
    _keys = {'s', '0', 'q'}
    try:
        for _line in sys.stdin:
            ch = _line.strip().lower()[:1]
            if ch in _keys:
                _cmd_q.put(ord(ch))
                print(f"  [키] {ch}")
    except Exception:
        pass


threading.Thread(target=_stdin_reader, daemon=True).start()

# ── 카메라 (재시도: stuck 시 release 후 재오픈으로 자동복구) ──
def _open_cam():
    if _IS_RPI:
        c = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # ★YUYV 1280x720이 검증본(MJPG는 OpenCV 디코더가 검은화면 — 메모리 2026-06-14)
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        c = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not c.isOpened():
            c = cv2.VideoCapture(CAMERA_INDEX)
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return c

cap = None
for _try in range(5):
    cap = _open_cam()
    time.sleep(1.0)
    _warm_ok = sum(1 for _ in range(20) if cap.read()[0])
    print(f"[카메라] 시도{_try + 1}: open={cap.isOpened()} 정상프레임 {_warm_ok}/20")
    if _warm_ok >= 5:
        break
    cap.release()
    print("[카메라] 프레임 없음 → release 후 재시도")
    time.sleep(1.5)
fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
if fw <= 0 or fh <= 0:                       # GStreamer는 -1 반환 → 파이프라인 해상도로 고정
    fw, fh = 1280, 720
print(f"[카메라] 해상도 {fw}x{fh}")
import atexit  # noqa: E402
atexit.register(lambda: cap.release() if cap is not None else None)  # Ctrl+C 포함 종료 시 항상 release
# ★ 좀비 방지: HailoRT __exit__가 Ctrl+C 때 멈춰 프로세스가 안 죽고 8091·카메라를 계속 잡음.
#   → 시그널 받으면 즉시 os._exit → OS가 포트·카메라 fd 회수(좀비 안 남음).
import signal  # noqa: E402
signal.signal(signal.SIGINT, lambda *_: os._exit(0))
signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, mtx, (fw, fh), cv2.CV_16SC2)

_RC = WORK_POLY_PX.mean(axis=0)
_RS = float(np.max(np.linalg.norm(WORK_POLY_PX - _RC, axis=1)))

pos_history = deque(maxlen=15)
ang_hist = deque(maxlen=30)
ba_hist = deque(maxlen=15)
head_hist = deque(maxlen=7)
tip_hist = deque(maxlen=7)
last_cmd = None
m1_coupling_log = []
auto_mode = False
auto_stable = 0
grab_cooldown = 0.0
current_m1 = SCAN_M1
searching = False
centering = False
sweep_start_t = 0.0
SWEEP_TIMEOUT = 16.0
AUTO_SEARCH_WAIT = 12       # run.flag 시 물체 없을 때 자동 sweep 진입까지 대기 프레임(발작 방지)
_search_noobj = 0
_swept_idle = False         # 이번 무물체 구간에 좌우 1회 훑었나 (한 번만 훑고 스캔 대기)
center_dir = 1.0
prev_center_err = None
no_obj_count = 0
sweep_phase = "left"
labels6 = ["", "M1 Base ", "M2 Shldr", "M3 Mirror", "M4 Elbow", "M5 Wrist", "M6 Roll "]


def _clear_hist():
    for h in (pos_history, ang_hist, ba_hist, head_hist, tip_hist):
        h.clear()


def _write_json(path, obj):
    try:
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")
        tmp.replace(path)
    except Exception:
        pass


def _append_jsonl(path, obj):
    try:
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(obj, ensure_ascii=False) + "\n")
    except Exception:
        pass


def _send_median():
    mx = float(np.median([p[0] for p in pos_history]))
    my = float(np.median([p[1] for p in pos_history]))
    bam = circular_median_180(list(ba_hist)) if last_cmd["use_kpt"] and ba_hist else None
    send_to_rpi(last_cmd, mx, my, bam)
    _append_jsonl(GRASP_LOG, {                      # 잡으러 보낸 기록 1줄
        "ts": round(time.time(), 1),
        "target": CLASS_NAMES.get(last_cmd.get("cls"), "?").upper(),
        "conf": round(last_cmd.get("conf", 0.0), 3),
        "size_mm": round(last_cmd.get("size_mm", 0.0), 1),
        "sent_arm_mm": [round(mx, 1), round(my, 1)],
        "angle_deg": round(bam, 1) if bam is not None else None,
        "ik_deg": {f"M{k}": round(last_cmd["cmd"][k], 1) for k in range(1, 7)},
    })


def send_scan_to_rpi(m1):
    try:
        with socket.create_connection((RPI_HOST, RPI_PORT), timeout=8) as s:
            s.sendall(json.dumps({"action": "scan", "m1": round(float(m1), 1)}).encode())
            print(f"  ▶ M1 회전 {m1:.0f}  응답: {s.recv(2048).decode()}")
    except Exception as e:
        print(f"  ▶ M1 회전 실패: {e}")


def send_home_to_rpi():
    try:
        with socket.create_connection((RPI_HOST, RPI_PORT), timeout=8) as s:
            s.sendall(json.dumps({"action": "home"}).encode())
            print(f"  ▶ 원점복귀  응답: {s.recv(2048).decode()}")
    except Exception as e:
        print(f"  ▶ 원점복귀 실패: {e}")


def send_sweep_to_rpi():
    try:
        with socket.create_connection((RPI_HOST, RPI_PORT), timeout=8) as s:
            s.sendall(json.dumps({"action": "sweep"}).encode())
            s.recv(2048)
            print("  ▶ 연속 스윕 시작")
    except Exception as e:
        print(f"  ▶ 스윕 실패: {e}")


def send_stop_to_rpi():
    try:
        with socket.create_connection((RPI_HOST, RPI_PORT), timeout=8) as s:
            s.sendall(json.dumps({"action": "stop"}).encode())
            resp = json.loads(s.recv(2048).decode())
            print(f"  ▶ 스윕 정지  M1={resp.get('m1')}")
            return resp.get("m1")
    except Exception as e:
        print(f"  ▶ 정지 실패: {e}")
        return None


def _m1_rotate(ax, ay):
    if current_m1 == SCAN_M1:
        return ax, ay
    dth = np.radians((current_m1 - SCAN_M1) / M1_GAIN * M1_ROT_SIGN)
    c, s = np.cos(dth), np.sin(dth)
    return ax * c - ay * s, ax * s + ay * c


def _m1_rotate_angle(ang):
    if current_m1 == SCAN_M1:
        return ang
    dth = (current_m1 - SCAN_M1) / M1_GAIN * M1_ROT_SIGN
    return float((ang - dth) % 180)


print(f"=== pose detect (HailoRT) ===  웹: http://0.0.0.0:{_STREAM_PORT}")

# ── 메인 루프 (InferVStreams 컨텍스트 안에서 반복) ──
_diag_t = time.time()
_diag_loops = 0
_diag_readfail = 0
_diag_torn = 0
_diag_inf_ms = 0.0
_last_state_write = 0.0
_stable_count = 0
_stable_prev_cls = None
_stable_prev_xy = None
with _network_group.activate(), InferVStreams(_network_group, _input_params, _output_params) as _pipeline:
  _consec_fail = 0
  while True:
    ret, frame = cap.read()
    if not ret:
        _diag_readfail += 1
        _consec_fail += 1
        if _consec_fail >= 25:               # ≈1초 연속 실패 → 카메라 자동 재오픈 (뽑았다 꽂기 불필요)
            print("[카메라] 연속 실패 → release 후 재오픈")
            cap.release()
            time.sleep(1.0)
            cap = _open_cam()
            time.sleep(1.0)
            _consec_fail = 0
        time.sleep(0.05)
        continue
    _consec_fail = 0
    # 찢긴(마젠타) 프레임 폐기: G채널이 R·B보다 비정상적으로 낮으면 깨진 프레임
    _bm, _gm, _rm = float(frame[:, :, 0].mean()), float(frame[:, :, 1].mean()), float(frame[:, :, 2].mean())
    if _gm < 0.62 * min(_bm, _rm):
        _diag_torn += 1
        continue
    frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    disp = frame.copy()
    cv2.polylines(disp, [WORK_POLY_PX], True, (0, 255, 255), 2)
    if BUSY_FLAG.exists():                  # 잡기/이동 중 → 추론·기록 정지(스캔 정지 상태에서만 인식)
        cv2.putText(disp, "PICK in progress - detection paused", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        with _frame_lock:
            _latest_frame = disp
        time.sleep(0.03)
        continue
    infer_cnt += 1
    if searching or infer_cnt % INFER_EVERY == 0 or _last_res is None:
        lb, sc, pw, ph = _letterbox(frame)
        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0  # HEF는 0~1 입력
        _t0 = time.time()
        raw = _pipeline.infer({_input_name: np.expand_dims(rgb, 0)})
        _diag_inf_ms = (time.time() - _t0) * 1000.0
        _last_res = _postprocess(raw, fw, fh, sc, pw, ph)
    res = _last_res

    _diag_loops += 1
    if time.time() - _diag_t >= 1.0:
        print(f"[FPS] {_diag_loops}  infer={_diag_inf_ms:.0f}ms  readfail={_diag_readfail}  torn={_diag_torn}")
        _diag_t = time.time()
        _diag_loops = 0
        _diag_readfail = 0
        _diag_torn = 0

    # ── grasp점이 영역+reach 안, 최고 conf 1개 선택 ──
    _cnc_busy = CNC_BUSY_FLAG.exists()   # CNC 가공중 → 파이프 제외(볼트/너트만 잡기)
    best = None
    if res.boxes is not None and len(res.boxes) > 0:
        xyxy = res.boxes.xyxy.cpu().numpy()
        clss = res.boxes.cls.cpu().numpy().astype(int)
        confs = res.boxes.conf.cpu().numpy()
        kxy = res.keypoints.xy.cpu().numpy() if res.keypoints is not None else None
        for i in range(len(confs)):
            cls = int(clss[i])
            if BOLT_ONLY and cls != BOLT_ID:
                continue
            is_bolt = cls == BOLT_ID
            is_pipe = cls == PIPE_ID
            use_kpt = is_bolt or is_pipe
            x1, y1, x2, y2 = xyxy[i]
            head = np.array(kxy[i][0], float) if kxy is not None else np.array([(x1 + x2) / 2, (y1 + y2) / 2])
            tip = np.array(kxy[i][1], float) if kxy is not None else head
            if use_kpt:
                grasp = head + GRASP_T * (tip - head)
                ang_raw = float((np.degrees(np.arctan2(tip[1] - head[1], tip[0] - head[0])) + ANG_OFFSET) % 180)
                ang = _m1_rotate_angle(ang_raw)
            else:
                grasp = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                ang_raw = ang = 0.0
            gx, gy = float(grasp[0]), float(grasp[1])
            box_i = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.int32)
            if _cnc_busy and is_pipe:        # CNC 가공중 → 파이프는 후보 제외(드물게 표시만)
                cv2.polylines(disp, [box_i], True, (160, 160, 160), 1)
                cv2.putText(disp, "PIPE skip(CNC busy)", (int(x1), int(y1) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
                continue
            if USE_CUSTOM_REGION and cv2.pointPolygonTest(WORK_POLY_PX, (gx, gy), False) < 0:
                cv2.polylines(disp, [box_i], True, (130, 130, 130), 1)
                continue
            if use_kpt:
                ha, ta = _px_to_arm_mm(*head), _px_to_arm_mm(*tip)
                size_mm = float(np.hypot(ha[0] - ta[0], ha[1] - ta[1]))
            else:
                ca = [_px_to_arm_mm(cx, cy) for cx, cy in box_i]
                size_mm = float(max(np.hypot(ca[0][0] - ca[1][0], ca[0][1] - ca[1][1]),
                                    np.hypot(ca[1][0] - ca[2][0], ca[1][1] - ca[2][1])))
            lo, hi = SIZE_RANGE_MM.get(cls, (0.0, 1e9))
            if USE_SIZE_GATE and not (lo <= size_mm <= hi):
                cv2.polylines(disp, [box_i], True, (0, 0, 255), 1)
                cv2.putText(disp, f"{size_mm:.0f}mm", (int(x1), int(y1) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue
            real = cv2.perspectiveTransform(np.array([[[gx, gy]]], np.float32), H_HOMO)[0][0]
            axx, ayy = homography_to_arm(float(real[0]), float(real[1]))
            axx, ayy = _m1_rotate(axx, ayy)
            if not USE_CUSTOM_REGION:
                r = (axx * axx + ayy * ayy) ** 0.5
                if not (RELIABLE_R_MIN <= r <= RELIABLE_R_MAX):
                    cv2.polylines(disp, [box_i], True, (0, 200, 255), 2)
                    continue
            prio = 1 if is_pipe else 0
            if best is None or (prio, confs[i]) > (best["prio"], best["conf"]):
                best = dict(cls=cls, is_bolt=is_bolt, is_pipe=is_pipe, use_kpt=use_kpt, prio=prio,
                            conf=float(confs[i]), box=box_i, size_mm=size_mm,
                            head=head, tip=tip, grasp=grasp, ang=ang, ang_raw=ang_raw,
                            gx=gx, gy=gy, hx=float(real[0]), hy=float(real[1]), xa=axx, ya=ayy)

    auto_ready = False
    centered = False
    grab_direct = False
    if best:
        if best["use_kpt"]:
            h_new, t_new = np.array(best["head"], float), np.array(best["tip"], float)
            if head_hist:
                mh = np.median(np.array(head_hist), axis=0)
                mt = np.median(np.array(tip_hist), axis=0)
                if np.hypot(*(h_new - mt)) < np.hypot(*(h_new - mh)):
                    h_new, t_new = t_new, h_new
            head_hist.append(h_new)
            tip_hist.append(t_new)
            mh = np.median(np.array(head_hist), axis=0)
            mt = np.median(np.array(tip_hist), axis=0)
            grasp = mh + GRASP_T * (mt - mh)
            gx, gy = float(grasp[0]), float(grasp[1])
            real = cv2.perspectiveTransform(np.array([[[gx, gy]]], np.float32), H_HOMO)[0][0]
            _ht = cv2.perspectiveTransform(np.array([[mh, mt]], np.float32), H_HOMO)[0]
            _hax, _hay = _m1_rotate(*homography_to_arm(float(_ht[0][0]), float(_ht[0][1])))
            _tax, _tay = _m1_rotate(*homography_to_arm(float(_ht[1][0]), float(_ht[1][1])))
            axx = _hax + GRASP_T * (_tax - _hax)
            ayy = _hay + GRASP_T * (_tay - _hay)
            _raw = np.degrees(np.arctan2(mt[1] - mh[1], mt[0] - mh[0]))
            _ang_raw = float((_raw + ANG_OFFSET) % 180)
            best.update(head=mh, tip=mt, grasp=grasp, gx=gx, gy=gy,
                        hx=float(real[0]), hy=float(real[1]), xa=axx, ya=ayy,
                        ang_raw=_ang_raw, ang=_m1_rotate_angle(_ang_raw))
        near = bool(best["gy"] > fh // 2)
        centered = abs(best["gx"] - fw / 2) <= CENTER_DEADBAND
        if best["use_kpt"]:
            _hr = cv2.perspectiveTransform(np.array([[best["head"]]], np.float32), H_HOMO)[0][0]
            _tr = cv2.perspectiveTransform(np.array([[best["tip"]]], np.float32), H_HOMO)[0][0]
            _ha = _m1_rotate(*homography_to_arm(float(_hr[0]), float(_hr[1])))
            _ta = _m1_rotate(*homography_to_arm(float(_tr[0]), float(_tr[1])))
            grab_direct = (_ha[0] ** 2 + _ha[1] ** 2) < (_ta[0] ** 2 + _ta[1] ** 2)
        else:
            grab_direct = True
        ang_hist.append(best["ang"])
        _a = np.deg2rad(np.asarray(ang_hist, float) * 2.0)
        _R = abs(np.mean(np.exp(1j * _a)))
        sigma = (np.degrees(np.sqrt(max(0.0, -2.0 * np.log(max(_R, 1e-9))))) / 2.0
                 if len(ang_hist) > 3 else 0.0)
        pos_history.append((best["xa"], best["ya"]))
        ba_hist.append(best["ang"])
        cen = max(0.0, 1.0 - float(np.hypot(best["gx"] - _RC[0], best["gy"] - _RC[1])) / max(_RS, 1.0))
        grab = best["conf"] * cen

        # ── 안정 카운터(오검출 방지): 같은 클래스 + 위치 ±tol + conf≥th 연속 ──
        _cur_xy = (best["xa"], best["ya"])
        if (_stable_prev_cls == best["cls"] and _stable_prev_xy is not None
                and abs(_cur_xy[0] - _stable_prev_xy[0]) <= STABLE_TOL_MM
                and abs(_cur_xy[1] - _stable_prev_xy[1]) <= STABLE_TOL_MM
                and best["conf"] >= STABLE_CONF):
            _stable_count += 1
        else:
            _stable_count = 1 if best["conf"] >= STABLE_CONF else 0
        _stable_prev_cls = best["cls"]
        _stable_prev_xy = _cur_xy

        col = (255, 0, 0) if best["is_pipe"] else (0, 255, 0) if best["is_bolt"] else (0, 165, 255)
        cv2.polylines(disp, [best["box"]], True, col, 2)
        if best["use_kpt"]:
            hp = tuple(best["head"].astype(int)); tp = tuple(best["tip"].astype(int))
            cv2.line(disp, hp, tp, (0, 255, 255), 1)
            cv2.circle(disp, hp, 5, (0, 0, 255), -1)
            cv2.circle(disp, tp, 5, (255, 80, 0), -1)
        gp = (int(best["gx"]), int(best["gy"]))
        cv2.drawMarker(disp, gp, (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
        cv2.circle(disp, gp, 8, (0, 255, 0), 1)
        if best["use_kpt"]:
            d = best["tip"] - best["head"]; n = float(np.hypot(*d))
            if n > 1e-6:
                perp = np.array([-d[1], d[0]]) / n
                j1 = (np.array([best["gx"], best["gy"]]) - perp * 24).astype(int)
                j2 = (np.array([best["gx"], best["gy"]]) + perp * 24).astype(int)
                cv2.line(disp, tuple(j1), tuple(j2), (0, 140, 255), 2)
        bx, by = int(best["box"][:, 0].min()), int(best["box"][:, 1].min())
        gcol = (0, 255, 0) if grab >= 0.6 else (0, 200, 255) if grab >= 0.4 else (0, 0, 255)
        cv2.putText(disp, f"grab {grab:.2f}", (bx, by - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, gcol, 2)
        cv2.putText(disp, f"{CLASS_NAMES.get(best['cls'], '?')} {best['conf']:.2f} {best['size_mm']:.0f}mm", (bx, by - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

        try:
            cmd = solve_pick(best["xa"], best["ya"], best["ang"], best["use_kpt"])
            bad = check_limits(cmd)
            r = (best["xa"] ** 2 + best["ya"] ** 2) ** 0.5
            z_w = GRASP_HEIGHT + L3 - ARM_H
            D = (r ** 2 + z_w ** 2) ** 0.5
            reach = "REACHABLE" if MIN_REACH <= D <= MAX_REACH else "OUT"
            _ox, _oy = _local_offset(best['hx'], best['hy'])
            panel = [f"grasp real ({best['hx']:.0f}, {best['hy']:.0f}) mm",
                     f"arm  ({best['xa']:.0f}, {best['ya']:.0f}) mm",
                     f"RBF off ({_ox:+.1f}, {_oy:+.1f}) mm",
                     f"ang {best['ang']:.0f}  sig {sigma:.1f}  near={near}  {'DIRECT' if grab_direct else 'CENTER'}",
                     f"D={D:.0f}mm  r={r:.0f}mm  {reach}",
                     ""]
            panel += [f"{labels6[k]}: {cmd[k]:7.1f}" for k in range(1, 7)]
            pcol = (0, 0, 255) if bad else (0, 255, 0)
            for j, t in enumerate(panel):
                cv2.putText(disp, t, (10, 28 + j * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, pcol, 2)
            last_cmd = dict(cmd=cmd, xa=best["xa"], ya=best["ya"], ba=best["ang"],
                            is_bolt=best["is_bolt"], is_pipe=best["is_pipe"], use_kpt=best["use_kpt"], near=near,
                            cls=best["cls"], conf=best["conf"], size_mm=best["size_mm"])
            if time.time() - _last_state_write > 0.5:    # 계약 스키마로 비전상태 저장(대시보드 연동)
                _bx1, _by1 = float(best["box"][:, 0].min()), float(best["box"][:, 1].min())
                _bx2, _by2 = float(best["box"][:, 0].max()), float(best["box"][:, 1].max())
                _write_json(VISION_STATE_PATH, {"vision": {
                    "v": SCHEMA_V,
                    "target": CLASS_NAMES.get(best["cls"], "?").upper(),
                    "confidence": round(best["conf"], 3),
                    "stable_frames": int(_stable_count),
                    "arm_mm": [round(best["xa"], 1), round(best["ya"], 1)],
                    "angle_deg": round(best["ang"], 1),
                    "distance_mm": round(D, 1),
                    "near": bool(near),    # 가까운 물체(M5 클리어 분기). pick_runner가 그대로 사용
                    "ik": {f"M{k}": round(cmd[k], 1) for k in range(1, 7)},
                    "bbox": [round(_bx1 / fw, 4), round(_by1 / fh, 4),
                             round(_bx2 / fw, 4), round(_by2 / fh, 4)],
                    "correction": {"dx_mm": round(_ox, 1), "dy_mm": round(_oy, 1)},
                    "source": "detect_pose_hailo",
                    "ts": round(time.time(), 3),
                }})
                _last_state_write = time.time()
            at_scan = abs(current_m1 - SCAN_M1) < 0.5
            auto_ready = (best["conf"] >= AUTO_GRAB_MIN and sigma < AUTO_SIGMA_MAX
                          and (at_scan or centered))
        except ValueError:
            cv2.putText(disp, "IK: out of reach", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        pos_history.clear()
        ang_hist.clear()
        ba_hist.clear()
        head_hist.clear()
        tip_hist.clear()
        _stable_count = 0
        _stable_prev_cls = None
        _stable_prev_xy = None
        if time.time() - _last_state_write > 0.5:    # 무검출도 alive 유지(target=NONE)
            _write_json(VISION_STATE_PATH, {"vision": {
                "v": SCHEMA_V, "target": "NONE", "confidence": 0.0,
                "stable_frames": 0, "source": "detect_pose_hailo",
                "ts": round(time.time(), 3),
            }})
            _last_state_write = time.time()

    if auto_mode and auto_ready and time.time() >= grab_cooldown:
        auto_stable += 1
    else:
        auto_stable = 0
    acol = (0, 255, 0) if auto_mode else (160, 160, 160)
    cv2.putText(disp, f"AUTO {'ON' if auto_mode else 'OFF'}"
                + (f"  {auto_stable}/{AUTO_STABLE_N}" if auto_mode else "")
                + ("  [SEARCHING]" if searching else "")
                + ("  [CENTERING]" if centering else "")
                + f"    M1={current_m1:.0f}",
                (10, fh - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, acol, 2)
    cv2.putText(disp, "A:auto S:send C:copy  J/L:M1 K:reset  0:home P:angLog Q:quit", (10, fh - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    with _frame_lock:
        _latest_frame = disp
    k = 0
    try:
        k = _cmd_q.get_nowait()
    except Exception:
        pass
    time.sleep(0.01)
    if k in (ord('q'), ord('Q')):
        os._exit(0)    # HailoRT 정리 건너뛰고 즉시 종료 (좀비 포트·카메라 방지)
    if k in (ord('a'), ord('A')):
        if AUTO_ENABLED:
            auto_mode = not auto_mode
            auto_stable = 0
            print(f"  ▶ AUTO {'ON' if auto_mode else 'OFF'}")
        else:
            print("  ▶ AUTO 비활성화됨 (AUTO_ENABLED=False). 무시.")
    if k in (ord('s'), ord('S')):
        auto_mode = False
        if last_cmd and pos_history:
            _send_median()
            searching = False
            centering = False
            current_m1 = SCAN_M1
            _clear_hist()
        elif best is None:
            searching = True
            sweep_start_t = time.time()
            send_sweep_to_rpi()
    if k == ord('0'):
        send_home_to_rpi()
        current_m1 = SCAN_M1
    if k in (ord('c'), ord('C')) and last_cmd:
        one = " ".join(f"{last_cmd['cmd'][m]:.1f}" for m in range(1, 7))
        print("  ▶ IK각 복사:", one, "|", "성공" if copy_clipboard(one) else "실패")
    if k in (ord('j'), ord('J')):
        current_m1 = max(120.0, current_m1 - M1_STEP)
        send_scan_to_rpi(current_m1)
        _clear_hist()
    if k in (ord('l'), ord('L')):
        current_m1 = min(240.0, current_m1 + M1_STEP)
        send_scan_to_rpi(current_m1)
        _clear_hist()
    if k in (ord('k'), ord('K')):
        current_m1 = SCAN_M1
        send_scan_to_rpi(current_m1)
        _clear_hist()
    if k in (ord('p'), ord('P')) and best and best["is_bolt"]:
        m1_coupling_log.append((current_m1, best["ang_raw"], best["ang"]))
        print(f"  ▶ [M1커플링 {len(m1_coupling_log)}] M1={current_m1:.1f}  raw={best['ang_raw']:.1f}  보정후={best['ang']:.1f}")
        print("     (M1, raw):", [(round(a, 1), round(b, 1)) for a, b, _ in m1_coupling_log])

    # ── run.flag 자동탐색: 없으면 자동 sweep, 있으면 센터링 (SWEEP/START가 플래그 켬, 기존 로직 재사용) ──
    if RUN_FLAG.exists():
        if best is not None:
            # ★화면에 보이는 물체는 자동센터링 안 함(원래대로 회전0으로 바로 잡기=정확).
            #   센터링(M1 회전)은 스윕으로 찾은 물체에만(아래 searching→centering 경로, 원래 로직).
            _search_noobj = 0
            _swept_idle = False                   # 물체 봤음 → 다음 무물체때 다시 1회 훑기 허용
        elif not searching and not centering and not _swept_idle:
            _search_noobj += 1
            if _search_noobj >= AUTO_SEARCH_WAIT:  # 잠깐 없으면 좌우 1회 훑기(무한 X)
                searching = True
                sweep_start_t = time.time()
                send_sweep_to_rpi()
                _swept_idle = True                # 이번 무물체 구간엔 1회만 → 끝나면 스캔 대기
                _search_noobj = 0
    else:
        if searching or centering:                # 플래그 꺼짐 → 탐색/센터링 중단
            send_stop_to_rpi()
            searching = False
            centering = False
        if current_m1 != SCAN_M1:                 # ★run.flag OFF → detect 내부 M1만 스캔 동기(물리복귀는 브릿지 담당) + 옛 검출 클리어
            current_m1 = SCAN_M1                   #   send_scan 제거: 브릿지(END/STOP/SWEEP_STOP)가 M1 복귀 → detect rotate와 충돌(역동작) 방지
            _clear_hist()
        _search_noobj = 0
        _swept_idle = False                       # 정지 → 리셋

    if searching:
        if best is not None:
            m1 = send_stop_to_rpi()
            if m1 is not None:
                current_m1 = m1
            _clear_hist()
            searching = False
            centering = True
        elif time.time() - sweep_start_t > SWEEP_TIMEOUT:
            send_stop_to_rpi()
            send_scan_to_rpi(SCAN_M1)
            current_m1 = SCAN_M1
            searching = False

    if centering:
        if best is None:
            centering = False
        elif centered:
            centering = False
        else:
            err = best["gx"] - fw / 2
            step = max(-CENTER_MAX_STEP, min(CENTER_MAX_STEP, CENTER_KP * err))
            nm = max(120.0, min(240.0, current_m1 + step))
            if abs(nm - current_m1) > 0.5:
                current_m1 = nm
                send_scan_to_rpi(current_m1)
                _clear_hist()
                for _ in range(5):
                    cap.read()

    if auto_mode and time.time() < grab_cooldown:
        pass
    elif auto_mode and best is None:
        prev_center_err = None
        no_obj_count += 1
        if SWEEP_SEARCH and no_obj_count >= SEARCH_WAIT and current_m1 > 120.0:
            nm = max(120.0, current_m1 - SEARCH_STEP)
            current_m1 = nm
            send_scan_to_rpi(current_m1)
            _clear_hist()
            for _ in range(5):
                cap.read()
            no_obj_count = 0
    elif auto_mode and current_m1 != SCAN_M1 and not centered:
        no_obj_count = 0
        err = best["gx"] - fw / 2
        if prev_center_err is not None and abs(err) > abs(prev_center_err) + 6:
            center_dir = -center_dir
        prev_center_err = err
        step = max(-CENTER_MAX_STEP, min(CENTER_MAX_STEP, center_dir * CENTER_KP * err))
        nm = max(120.0, min(240.0, current_m1 + step))
        if abs(nm - current_m1) > 0.5:
            current_m1 = nm
            send_scan_to_rpi(current_m1)
            _clear_hist()
            auto_stable = 0
            for _ in range(5):
                cap.read()
    else:
        no_obj_count = 0
        prev_center_err = None

    if auto_mode and auto_stable >= AUTO_STABLE_N and last_cmd and pos_history:
        print(f"  ▶ [AUTO] 전송 (grab>={AUTO_GRAB_MIN}, {AUTO_STABLE_N}프레임 안정)")
        _send_median()
        grab_cooldown = time.time() + AUTO_COOLDOWN
        current_m1 = SCAN_M1
        auto_stable = 0
        pos_history.clear()
        ang_hist.clear()
        ba_hist.clear()
        last_cmd = None
        for _ in range(8):
            cap.read()

cap.release()
print("종료")
