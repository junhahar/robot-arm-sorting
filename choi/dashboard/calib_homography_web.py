# -*- coding: utf-8 -*-
"""calib_homography_web.py — 브라우저로 보면서 호모그래피 캘리브레이션 (웹캠).

라이브 영상에 체커보드 코너를 실시간 표시(FOUND/NOT) → 위치 잡고 [캘리브레이션 실행] 누르면
현재 프레임으로 호모그래피(H) 계산·저장(homography_webcam.npz) → 재투영 오차 확인 →
[homography_64cm 교체] 누르면 detect가 쓰는 homography_64cm.npz로 복사.

실행: detect 끄고(카메라 점유) → python3 calib_homography_web.py
보기: 브라우저  http://172.30.1.26:8094
   - 가로코너/세로코너/한변mm 입력 → [적용]  (내부 코너 수 = 사각형 수 - 1)
   - 체커보드를 테이블(물체 놓이는 평면)에 평평하게, 전체 보이게 → FOUND 뜨면
   - [캘리브레이션 실행] → 오차 평균 1~2mm면 좋음 → [교체]
종료: Ctrl+C
"""
import time
import json
import shutil
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import cv2

CAMERA_INDEX = 0
PORT = 8094
BASE = Path(__file__).resolve().parent
OUT = BASE / "homography_webcam.npz"
LIVE = BASE / "homography_64cm.npz"          # detect가 읽는 라이브 호모그래피

state = {"cols": 9, "rows": 6, "square": 25.0, "undistort": False}
_latest_raw = None
_latest_jpeg = None
_last_found = False
_lock = threading.Lock()

# 왜곡 보정용 카메라 내부파라미터(있으면). 웹캠용 아니면 undistort 끄는 게 안전.
_mtx = _dist = None
try:
    _cal = np.load(str(BASE / "camera_calib.npz"))
    _mtx, _dist = _cal["mtx"], _cal["dist"]
except Exception:
    pass


def open_camera():
    c = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return c


def _find(gray, cols, rows):
    return cv2.findChessboardCorners(
        gray, (cols, rows),
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)


def capture_loop():
    global _latest_raw, _latest_jpeg, _last_found
    cap = open_camera()
    if not cap.isOpened():
        print("카메라 못 엶 — detect 켜져 있으면 끄고(pkill -f detect_pose) 다시.")
        return
    print(f"캘리브레이션 서버 시작: http://172.30.1.26:{PORT}")
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        with _lock:
            _latest_raw = frame
            cols, rows = state["cols"], state["rows"]
        disp = frame.copy()
        n += 1
        if n % 3 == 0:   # 3프레임마다 검출(CPU 부담↓) — 보기용 피드백
            try:
                small = cv2.resize(frame, (640, 360))
                g = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
                found, corners = _find(g, cols, rows)
                _last_found = bool(found)
                if found:
                    cv2.drawChessboardCorners(
                        disp, (cols, rows), (corners * 2.0).astype(np.float32), True)
            except Exception:
                _last_found = False
        txt = "FOUND" if _last_found else "NOT FOUND"
        col = (0, 255, 0) if _last_found else (0, 0, 255)
        cv2.putText(disp, txt, (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)
        ok2, buf = cv2.imencode(".jpg", disp, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if ok2:
            with _lock:
                _latest_jpeg = buf.tobytes()


def do_calibrate():
    with _lock:
        frame = None if _latest_raw is None else _latest_raw.copy()
        cols, rows, square, undist = state["cols"], state["rows"], state["square"], state["undistort"]
    if frame is None:
        return {"ok": False, "msg": "카메라 프레임 없음"}
    img = frame
    if undist and _mtx is not None:
        img = cv2.undistort(img, _mtx, _dist)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = _find(gray, cols, rows)
    if not found:
        return {"ok": False, "msg": "체커보드 못 찾음 — 보드 전체 평평히/조명/내부코너수(cols·rows) 확인"}
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    obj = np.zeros((cols * rows, 2), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= square
    pix = corners.reshape(-1, 2)
    H, _ = cv2.findHomography(pix, obj, cv2.RANSAC, 3.0)
    if H is None:
        return {"ok": False, "msg": "호모그래피 계산 실패(코너 분포 확인)"}
    proj = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - obj, axis=1)
    np.savez(str(OUT), H=H)
    cv2.imwrite(str(BASE / "homo_board_webcam.jpg"), img)   # 캡처본 확인용
    return {"ok": True, "n": int(len(pix)),
            "mean": round(float(err.mean()), 2), "max": round(float(err.max()), 2)}


def do_apply():
    if not OUT.exists():
        return {"ok": False, "msg": "먼저 [캘리브레이션 실행]"}
    shutil.copy(str(OUT), str(LIVE))
    return {"ok": True, "msg": f"{LIVE.name} 교체 완료 (detect 재시작하면 적용)"}


PAGE = ("""<html><head><meta charset="utf-8"><title>호모그래피 캘리브</title></head>
<body style="margin:0;background:#0a0e14;color:#cfe;font-family:sans-serif;text-align:center">
<h3 style="color:#20d5e8">호모그래피 캘리브레이션 — 보드 맞추고 [캘리브레이션 실행]</h3>
<img src="/stream" style="max-width:90%"><br>
<div style="margin:10px;font-size:15px">
 가로코너 <input id="cols" type="number" value="9" style="width:60px">
 세로코너 <input id="rows" type="number" value="6" style="width:60px">
 한변mm <input id="square" type="number" value="25" style="width:70px">
 <button onclick="setp()">적용</button>
 <small style="color:#89a">(내부 코너 수 = 사각형 수 - 1)</small>
</div>
<button onclick="calib()" style="font-size:20px;padding:12px 28px;margin:8px">📐 캘리브레이션 실행</button>
<button onclick="apply()" id="applyBtn" disabled style="font-size:16px;padding:10px 20px;opacity:.5">homography_64cm 교체</button>
<div id="result" style="font-size:18px;margin:14px;white-space:pre;color:#9fe"></div>
<script>
function setp(){const q=`?cols=${cols.value}&rows=${rows.value}&square=${square.value}`;return fetch('/set'+q)}
function calib(){setp().then(()=>fetch('/calibrate')).then(r=>r.json()).then(d=>{
  if(d.ok){result.textContent=`✓ 코너 ${d.n}개\\n재투영 오차  평균 ${d.mean}mm  최대 ${d.max}mm\\n`+(d.mean<2?'좋음! 교체해도 됨':'오차 큼 — 보드 평평/숫자 확인');applyBtn.disabled=false;applyBtn.style.opacity=1}
  else{result.textContent='✗ '+d.msg;applyBtn.disabled=true;applyBtn.style.opacity=.5}})}
function apply(){fetch('/apply').then(r=>r.json()).then(d=>{result.textContent+='\\n'+(d.ok?'→ '+d.msg:d.msg)})}
</script>
</body></html>""").encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def _json(self, obj):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(obj).encode())

    def do_GET(self):
        p = urlparse(self.path)
        if p.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PAGE)
        elif p.path == "/set":
            q = parse_qs(p.query)
            try:
                with _lock:
                    if "cols" in q: state["cols"] = int(float(q["cols"][0]))
                    if "rows" in q: state["rows"] = int(float(q["rows"][0]))
                    if "square" in q: state["square"] = float(q["square"][0])
                self._json({"ok": True})
            except Exception as e:
                self._json({"ok": False, "msg": str(e)})
        elif p.path == "/calibrate":
            self._json(do_calibrate())
        elif p.path == "/apply":
            self._json(do_apply())
        elif p.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _latest_jpeg
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, *a):
        pass


def main():
    threading.Thread(target=capture_loop, daemon=True).start()
    try:
        ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
    except KeyboardInterrupt:
        print("\n종료.")


if __name__ == "__main__":
    main()
