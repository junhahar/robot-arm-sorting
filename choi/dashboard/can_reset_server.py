# -*- coding: utf-8 -*-
"""can_reset_server.py — CAN(can0) 재연결 미니 서비스 (root로 실행).

대시보드 "통신 재연결" 버튼이 GET /reset 호출 → can0 down→설정→up (bus-off/에러 해제).
브리지와 완전 독립(브리지가 죽어있어도 작동). systemd로 root 실행해야 ip link 권한 있음.

엔드포인트:
  GET /reset           → can0 재연결 실행(down→설정→up), JSON {ok, state, log}
  GET /bridge-restart  → 브리지 서비스 재시작(systemctl restart sambo-ws-bridge), "서버 재연결"
  GET /status          → 현재 can0 상태 JSON
포트: 8096  (CORS 허용 — 대시보드 :8090에서 fetch)
"""
import re
import json
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = 8096
IFACE = "can0"
BITRATE = "125000"
SAMPLE_POINT = "0.625"


def _run(args):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=10)
        return p.returncode, (p.stdout + p.stderr).strip()
    except Exception as e:
        return -1, str(e)


def can_state():
    rc, out = _run(["ip", "-details", "link", "show", IFACE])
    if rc != 0:
        return "unknown"
    m = re.search(r"can state ([A-Z-]+)", out)
    return m.group(1) if m else "unknown"


def do_reset():
    log = []
    for args in (["ip", "link", "set", IFACE, "down"],
                 ["ip", "link", "set", IFACE, "type", "can",
                  "bitrate", BITRATE, "sample-point", SAMPLE_POINT],
                 ["ip", "link", "set", IFACE, "up"]):
        rc, out = _run(args)
        log.append({"cmd": " ".join(args), "rc": rc, "out": out})
        if rc != 0:
            return {"ok": False, "step": " ".join(args), "log": log, "state": can_state()}
    return {"ok": True, "log": log, "state": can_state()}


def do_bridge_restart():
    rc, out = _run(["systemctl", "restart", "sambo-ws-bridge"])
    if rc != 0:
        return {"ok": False, "msg": out[:300]}
    rc2, st = _run(["systemctl", "is-active", "sambo-ws-bridge"])
    return {"ok": rc2 == 0, "state": st.strip()}


class Handler(BaseHTTPRequestHandler):
    def _send(self, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/reset"):
            self._send(do_reset())
        elif self.path.startswith("/bridge-restart"):
            self._send(do_bridge_restart())
        elif self.path.startswith("/status") or self.path == "/":
            self._send({"ok": True, "state": can_state()})
        else:
            self.send_response(404)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

    def log_message(self, *a):
        pass


def main():
    print(f"CAN reset server: http://0.0.0.0:{PORT}  (GET /reset)")
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
