"""Local GPT bridge for the Sambo robot-arm dashboard.

Run this file from a new PowerShell window after OPENAI_API_KEY is configured.
The server binds to localhost only and never exposes the API key to the browser.
"""

from __future__ import annotations

import json
import os
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


HOST = "127.0.0.1"
PORT = 8766
MODEL = os.environ.get("OPENAI_MODEL", "gpt-5.4-mini")
DASHBOARD_DIR = Path(__file__).resolve().parent

SYSTEM_INSTRUCTIONS = """\
너는 삼보모터스 로봇팔 자동 분류 시스템의 작업자용 AI 설명 도우미다.
현장 작업자가 5초 안에 이해할 수 있도록 쉽고 짧은 한국어로 답변한다.
대시보드 JSON 상태를 근거로 지금 상태와 바로 해야 할 일만 우선 설명한다.
기본 답변은 7줄 이내로 작성한다. 사용자가 상세 설명을 요청한 경우에만 길게 설명한다.
첫 줄에는 반드시 "상태: 정상", "상태: 주의", "상태: 위험" 중 하나를 표시한다.
DEMO MODE라면 첫 문장 다음에 "현재 화면은 연습용 샘플 데이터입니다."라고 한 번만 알린다.
사람 팔 MPU 값은 추정값이며 정확한 절대 관절각이라고 표현하지 않는다.
JSON 키, true/false, 영문 상태 코드를 그대로 나열하지 않는다.
PRE_GRASP는 "집기 전 접근", WARN은 "주의", BOLT는 "볼트"처럼 작업자가 이해하기 쉬운 말로 바꾼다.
정상인 항목을 전부 나열하지 않는다. 문제가 있거나 작업자가 바로 확인할 항목만 말한다.
주의 상태에서는 점검 방법을 안내한다. 즉시 위험한 상황이 아니라면 E-STOP을 무조건 누르라고 안내하지 않는다.
사람 접근, 제어 불능 움직임, 심한 걸림, 위험 온도, 긴급 오류처럼 즉시 위험한 상황에서는 하드웨어 E-STOP과 주변 안전 확인을 가장 먼저 안내한다.
로봇 제어 명령을 실행했다고 말하지 않는다. 이 챗봇은 설명만 하고 제어하지 않는다.
기본 답변은 아래 형식을 사용한다.

상태: 정상 / 주의 / 위험
지금: 작업자가 바로 이해할 수 있는 한 문장
확인:
1. 가장 먼저 할 일
2. 필요한 경우 두 번째 할 일

사용자가 이유를 묻거나 상세 설명을 요청했을 때만 "이유:"를 추가한다.
"""


def extract_output_text(response: dict) -> str:
    """Extract assistant text from a raw Responses API response."""
    if isinstance(response.get("output_text"), str):
        return response["output_text"].strip()

    parts: list[str] = []
    for item in response.get("output", []):
        if item.get("type") != "message":
            continue
        for content in item.get("content", []):
            if content.get("type") == "output_text" and content.get("text"):
                parts.append(content["text"])
    return "\n".join(parts).strip()


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DASHBOARD_DIR), **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.path = "/sambo_robot_arm_dashboard.html"
        if self.path == "/api/health":
            return self.send_json(
                HTTPStatus.OK,
                {"ok": True, "model": MODEL, "api_key_configured": bool(os.environ.get("OPENAI_API_KEY"))},
            )
        return super().do_GET()

    def do_POST(self):
        if self.path != "/api/chat":
            return self.send_json(HTTPStatus.NOT_FOUND, {"error": "지원하지 않는 경로입니다."})

        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            return self.send_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {"error": "OPENAI_API_KEY 환경변수가 없습니다. 새 PowerShell 창에서 서버를 다시 실행하세요."},
            )

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1_000_000:
                raise ValueError("요청 크기가 올바르지 않습니다.")
            request_body = json.loads(self.rfile.read(length).decode("utf-8"))
            question = str(request_body.get("question", "")).strip()
            dashboard = request_body.get("dashboard", {})
            if not question:
                raise ValueError("질문을 입력하세요.")

            input_text = (
                "작업자 질문:\n"
                f"{question}\n\n"
                "현재 대시보드 상태 JSON:\n"
                f"{json.dumps(dashboard, ensure_ascii=False, separators=(',', ':'))}"
            )
            payload = {
                "model": MODEL,
                "instructions": SYSTEM_INSTRUCTIONS,
                "input": input_text,
                "max_output_tokens": 360,
            }
            upstream = Request(
                "https://api.openai.com/v1/responses",
                data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json",
                },
                method="POST",
            )
            with urlopen(upstream, timeout=50) as response:
                api_response = json.loads(response.read().decode("utf-8"))
            reply = extract_output_text(api_response)
            if not reply:
                raise RuntimeError("GPT 응답에서 텍스트를 찾지 못했습니다.")
            return self.send_json(HTTPStatus.OK, {"reply": reply, "model": MODEL})
        except HTTPError as error:
            detail = error.read().decode("utf-8", errors="replace")
            print(f"[OpenAI HTTP {error.code}] {detail}")
            return self.send_json(
                HTTPStatus.BAD_GATEWAY,
                {"error": f"OpenAI API 요청 실패: HTTP {error.code}"},
            )
        except URLError as error:
            print(f"[Network Error] {error}")
            return self.send_json(HTTPStatus.BAD_GATEWAY, {"error": "OpenAI API 네트워크 연결에 실패했습니다."})
        except (ValueError, json.JSONDecodeError) as error:
            return self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
        except Exception as error:  # Keep local server alive and return a readable message.
            print(f"[Server Error] {type(error).__name__}: {error}")
            return self.send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "GPT 응답 처리 중 오류가 발생했습니다."})

    def send_json(self, status: HTTPStatus, payload: dict):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string: str, *args):
        print(f"[Dashboard] {self.address_string()} - {format_string % args}")


if __name__ == "__main__":
    print("=" * 68)
    print("Sambo Robot Arm Dashboard GPT Server")
    print(f"URL   : http://{HOST}:{PORT}/")
    print(f"Model : {MODEL}")
    print(f"API key configured: {'YES' if os.environ.get('OPENAI_API_KEY') else 'NO'}")
    print("종료하려면 Ctrl+C를 누르세요.")
    print("=" * 68)
    ThreadingHTTPServer((HOST, PORT), DashboardHandler).serve_forever()
