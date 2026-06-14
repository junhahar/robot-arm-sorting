(function () {
  "use strict";

  const DEFAULT_OLLAMA_URL = "http://localhost:11434/api/chat";
  const DEFAULT_OLLAMA_MODEL = "exaone3.5:7.8b";
  const CHAT_TIMEOUT_MS = 60000;

  function getEl(id) {
    return document.getElementById(id);
  }

  function getOllamaUrl() {
    return localStorage.getItem("samboOllamaUrl") || DEFAULT_OLLAMA_URL;
  }

  function getOllamaModel() {
    return localStorage.getItem("samboOllamaModel") || DEFAULT_OLLAMA_MODEL;
  }

  function appendBotMessage(text, note) {
    if (typeof window.addChatMessage === "function") {
      window.addChatMessage(text, "bot", note || "");
    }
  }

  function appendUserMessage(text) {
    if (typeof window.addChatMessage === "function") {
      window.addChatMessage(text, "user");
    }
  }

  function getDashboardSnapshot() {
    const base =
      typeof window.getAiDashboardState === "function"
        ? window.getAiDashboardState()
        : {};

    try {
      if (typeof state !== "undefined") {
        base.life = state.life || null;
        base.mode = state.mode;
        base.demo = state.demo;
      }
    } catch {
      // The inline dashboard state is optional for this external adapter.
    }

    return base;
  }

  function fallbackAnswer(question) {
    if (typeof window.localEasyAssistant === "function") {
      return window.localEasyAssistant(question);
    }
    return "현재 화면 값을 기준으로 답변하려 했지만 AI 연결을 확인해야 합니다. PC에서 Ollama가 실행 중인지 먼저 확인하세요.";
  }

  const SYSTEM_PROMPT = [
    "당신은 삼보모터스 로봇팔 자동 분류 시스템 대시보드 전용 AI 작업 도우미입니다.",
    "대시보드 JSON에 포함된 로봇팔, STS3215 모터, CAN 통신, ToF, 비전, 수명예측 상태만 설명합니다.",
    "절대 로봇 제어 명령을 실행하거나 사용자가 실행하도록 유도하지 않습니다.",
    "답변은 한국어로, 비전공자도 이해할 수 있게 7줄 이내로 간결하게 작성합니다.",
    "첫 줄은 '상태: 정상', '상태: 주의', '상태: 경고' 중 하나로 시작합니다.",
    "수명예측은 남은 잔여량, 누적 cyc, 상태 이유를 쉬운 말로 설명합니다.",
    "DEMO MODE이면 실제 장비값이 아니라고 먼저 알립니다."
  ].join("\n");

  async function askOllama(question, dashboard) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), CHAT_TIMEOUT_MS);
    try {
      const response = await fetch(getOllamaUrl(), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          model: getOllamaModel(),
          messages: [
            { role: "system", content: SYSTEM_PROMPT },
            {
              role: "user",
              content:
                "작업자 질문:\n" +
                question +
                "\n\n현재 대시보드 상태 JSON:\n" +
                JSON.stringify(dashboard, null, 2)
            }
          ],
          stream: false,
          options: { temperature: 0.2 }
        }),
        signal: controller.signal
      });
      clearTimeout(timer);
      if (!response.ok) throw new Error("HTTP " + response.status);
      const data = await response.json();
      return data && data.message && data.message.content
        ? data.message.content
        : "";
    } catch (error) {
      clearTimeout(timer);
      throw error;
    }
  }

  window.submitAiChat = async function submitAiChat(event) {
    if (event && typeof event.preventDefault === "function") {
      event.preventDefault();
    }

    const input = getEl("aiChatInput");
    if (!input) return;

    const question = input.value.trim();
    if (!question) return;

    input.value = "";
    appendUserMessage(question);

    const oldPlaceholder = input.placeholder;
    input.placeholder = "PC Ollama 응답 대기 중...";
    input.disabled = true;

    try {
      const dashboard = getDashboardSnapshot();
      const answer = await askOllama(question, dashboard);
      appendBotMessage(answer || fallbackAnswer(question), "PC Ollama 응답");
    } catch (error) {
      const reason =
        error && error.name === "AbortError"
          ? "응답 시간 초과"
          : "PC Ollama 미연결 또는 CORS 확인";
      appendBotMessage(fallbackAnswer(question), reason + " · 로컬 안내 fallback");
    } finally {
      input.disabled = false;
      input.placeholder = oldPlaceholder;
      setTimeout(() => input.focus(), 40);
    }
  };

  window.askQuick = function askQuick(question) {
    if (typeof window.toggleAiChat === "function") {
      window.toggleAiChat(true);
    }
    const input = getEl("aiChatInput");
    if (input) input.value = question;
    window.submitAiChat({ preventDefault() {} });
  };

  document.addEventListener("DOMContentLoaded", function () {
    const note = document.querySelector(".ai-chat-note");
    if (note) {
      note.textContent =
        "현재 대시보드 값을 쉬운 말로 설명합니다. AI는 라즈베리파이가 아니라 이 화면을 연 PC의 Ollama(localhost:11434)를 사용합니다. 실제 제어 명령은 보내지 않습니다.";
    }
  });
})();
