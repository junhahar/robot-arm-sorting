# -*- coding: utf-8 -*-
"""calib_camera_web.py — 브라우저로 보면서 카메라 캘리브레이션 (렌즈 왜곡/내부파라미터).

체커보드를 각도·위치·거리 바꿔가며 [캡처]로 15~20장 모은 뒤 [캘리브레이션 실행] →
camera_calib.npz (mtx, dist) 저장. 이후 호모그래피/undistort에서 사용.

실행: detect 끄고(카메라 점유) → python3 calib_camera_web.py
보기: 브라우저  http://172.30.1.26:8095
   1) 가로코너/세로코너/한변mm 입력 → [적용]  (내부 코너 수 = 사각형 수 - 1)
   2) 체커보드를 각도·위치·거리 다르게 두고 FOUND 뜨면 [캡처] → 15~20장 반복
      (★다양할수록 좋음: 기울이기/모서리/가까이·멀리)
   3) [캘리브레이션 실행] → RMS 오차(px) 확인 (보통 <1px면 좋음) → camera_calib.npz 저장
종료: Ctrl+C
"""
import time
import json
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import numpy as np
import cv2

CAMERA_INDEX = 0
PORT = 8095
BASE = Path(__file__).resolve().parent
OUT = BASE / "camera_calib.npz"

state = {"cols": 7, "rows": 5, "square": 15.0}
shots = {"obj": [], "img": [], "size": None}
_latest_raw = None
_latest_jpeg = None
_last_found = False
_lock = threading.Lock()


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
    print(f"카메라 캘리브 서버 시작: http://172.30.1.26:{PORT}")
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
        if n % 3 == 0:
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
        with _lock:
            cnt = len(shots["img"])
        cv2.putText(disp, f"shots: {cnt}", (12, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        ok2, buf = cv2.imencode(".jpg", disp, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if ok2:
            with _lock:
                _latest_jpeg = buf.tobytes()


def do_capture():
    with _lock:
        frame = None if _latest_raw is None else _latest_raw.copy()
        cols, rows, square = state["cols"], state["rows"], state["square"]
    if frame is None:
        return {"ok": False, "msg": "프레임 없음", "n": len(shots["img"])}
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = _find(gray, cols, rows)
    if not found:
        return {"ok": False, "msg": "체커보드 못 찾음(이 장은 건너뜀)", "n": len(shots["img"])}
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square
    with _lock:
        shots["obj"].append(objp)
        shots["img"].append(corners)
        shots["size"] = gray.shape[::-1]   # (w, h)
        n = len(shots["img"])
    return {"ok": True, "n": n}


def do_calibrate():
    with _lock:
        obj = list(shots["obj"]); img = list(shots["img"]); size = shots["size"]
    if len(img) < 5:
        return {"ok": False, "msg": f"최소 5장(권장 15~20장) 필요 — 현재 {len(img)}장", "n": len(img)}
    ret, mtx, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None)
    np.savez(str(OUT), mtx=mtx, dist=dist)
    return {"ok": True, "rms": round(float(ret), 3), "n": len(img), "out": str(OUT)}


def do_reset():
    with _lock:
        shots["obj"].clear(); shots["img"].clear(); shots["size"] = None
    return {"ok": True, "n": 0}


PAGE = ("""<html><head><meta charset="utf-8"><title>카메라 캘리브</title></head>
<body style="margin:0;background:#0a0e14;color:#cfe;font-family:sans-serif;text-align:center">
<h3 style="color:#20d5e8">카메라 캘리브레이션(렌즈 왜곡) — 각도 바꿔가며 [캡처] 15~20장 → [캘리브레이션 실행]</h3>
<img src="/stream" style="max-width:88%"><br>
<div style="margin:8px;font-size:15px">
 가로코너 <input id="cols" type="number" value="7" style="width:55px">
 세로코너 <input id="rows" type="number" value="5" style="width:55px">
 한변mm <input id="square" type="number" value="15" style="width:65px">
 <button onclick="setp()">적용</button>
 <small style="color:#89a">(내부 코너 수 = 사각형 수 - 1)</small>
</div>
<button onclick="cap()" style="font-size:22px;padding:12px 30px;margin:6px">📷 캡처</button>
<button onclick="calib()" style="font-size:18px;padding:10px 22px">📐 캘리브레이션 실행</button>
<button onclick="reset()" style="font-size:14px;padding:8px 16px">초기화</button>
<div id="count" style="font-size:22px;margin:10px">캡처: 0장 (권장 15~20)</div>
<div id="result" style="font-size:18px;margin:10px;white-space:pre;color:#9fe"></div>
<script>
function setp(){const q=`?cols=${cols.value}&rows=${rows.value}&square=${square.value}`;return fetch('/set'+q)}
function show(n){count.textContent='캡처: '+n+'장 (권장 15~20)'}
function cap(){setp().then(()=>fetch('/capture')).then(r=>r.json()).then(d=>{show(d.n);result.textContent=d.ok?'✓ 캡처됨':'✗ '+d.msg})}
function calib(){fetch('/calibrate').then(r=>r.json()).then(d=>{
  if(d.ok){result.textContent=`✓ 캘리브 완료 (${d.n}장)\\nRMS 재투영 오차 ${d.rms} px\\n`+(d.rms<1?'좋음! camera_calib.npz 저장됨':'오차 큼 — 더 다양한 각도로 추가 캡처 권장')}
  else{result.textContent='✗ '+d.msg}})}
function reset(){fetch('/reset').then(r=>r.json()).then(d=>{show(d.n);result.textContent='초기화됨'})}
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
        elif p.path == "/capture":
            self._json(do_capture())
        elif p.path == "/calibrate":
            self._json(do_calibrate())
        elif p.path == "/reset":
            self._json(do_reset())
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
