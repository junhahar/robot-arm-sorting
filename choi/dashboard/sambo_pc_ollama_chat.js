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
    "반드시 한국어로만 답합니다. 사용자가 영어로 질문하거나 JSON 키가 영어여도 최종 답변은 한국어로 번역해서 말합니다.",
    "영어 제목, 영어 문단, 'Based on the provided JSON' 같은 영어 안내 문구를 절대 쓰지 않습니다.",
    "답변은 비전공자도 이해할 수 있게 7줄 이내로 간결하게 작성합니다.",
    "첫 줄은 '상태: 정상', '상태: 주의', '상태: 경고' 중 하나로 시작합니다.",
    "수명예측은 남은 잔여량, 누적 cyc, 상태 이유를 쉬운 말로 설명합니다.",
    "DEMO MODE이면 실제 장비값이 아니라고 먼저 알립니다."
  ].join("\n");

  function hasEnoughKorean(text) {
    const hangul = (String(text || "").match(/[가-힣]/g) || []).length;
    const latin = (String(text || "").match(/[A-Za-z]/g) || []).length;
    return hangul >= 8 || hangul >= latin * 0.25;
  }

  function looksLikeEnglishReport(text) {
    return /based on|joint summary|additional information|current angle|target angle|temperature|fatigue adjusted/i.test(
      String(text || "")
    );
  }

  function jointLabel(joint, index) {
    const id = joint && joint.id ? joint.id : "J" + (index + 1);
    const name = joint && joint.name ? joint.name : "";
    return name ? `${id} ${name}` : id;
  }

  function buildKoreanSummary(dashboard) {
    const joints = Array.isArray(dashboard?.joints)
      ? dashboard.joints
      : Array.isArray(dashboard?.robot?.joints)
        ? dashboard.robot.joints
        : [];
    if (!joints.length) return "상태: 정상\n현재 대시보드 값을 한국어로 정리하려 했지만 모터 데이터가 충분하지 않습니다.";

    const warn = joints.filter((j) => {
      const label = String(j?.label || j?.status || "").toUpperCase();
      const severity = Number(j?.severity || 0);
      return severity > 0 || /WARN|주의|경고|위험/.test(label);
    });
    const hottest = joints.reduce((a, b) => (Number(a?.temp || 0) >= Number(b?.temp || 0) ? a : b), joints[0]);
    const maxCycle = joints.reduce(
      (a, b) => (Number(a?.fatigueAdjustedCycles || a?.fatigue_adjusted_cycles || 0) >= Number(b?.fatigueAdjustedCycles || b?.fatigue_adjusted_cycles || 0) ? a : b),
      joints[0]
    );
    const lines = [
      `상태: ${warn.length ? "주의" : "정상"}`,
      warn.length
        ? `${warn.map((j, i) => jointLabel(j, i)).join(", ")}에서 평소보다 높은 신호가 있어 확인이 필요합니다.`
        : "현재 6개 모터 모두 정상 범위로 보이며 즉시 조치가 필요한 경고는 없습니다.",
      `가장 높은 온도는 ${jointLabel(hottest, joints.indexOf(hottest))}의 ${Math.round(Number(hottest?.temp || 0))}°C입니다.`,
      `가장 많이 누적된 보정 사용량은 ${jointLabel(maxCycle, joints.indexOf(maxCycle))}의 ${Number(maxCycle?.fatigueAdjustedCycles || maxCycle?.fatigue_adjusted_cycles || 0).toFixed(1)} cyc입니다.`,
      "수명예측 값은 기본 이동량에 온도, 전류, 부하 같은 부담 신호를 반영해 보수적으로 계산한 값입니다."
    ];
    return lines.join("\n");
  }

  function normalizeKoreanAnswer(answer, question, dashboard) {
    const text = String(answer || "").trim();
    if (!text) return fallbackAnswer(question);
    if (hasEnoughKorean(text) && !looksLikeEnglishReport(text)) return text;
    return buildKoreanSummary(dashboard);
  }

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
                JSON.stringify(dashboard, null, 2) +
                "\n\n위 JSON을 보고 반드시 한국어로만 답하세요. 영어 제목이나 영어 요약은 쓰지 마세요."
            }
          ],
          stream: false,
          options: { temperature: 0.1 }
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
      appendBotMessage(normalizeKoreanAnswer(answer, question, dashboard), "PC Ollama 응답");
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
