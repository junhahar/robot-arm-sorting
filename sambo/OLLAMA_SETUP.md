# 대시보드 AI 챗봇 (Ollama) 설정 가이드

대시보드의 AI 작업 도우미는 **로컬 Ollama**를 사용합니다.  
외부 API 키 없이 PC에서 직접 실행되며, 데이터가 외부로 나가지 않습니다.

---

## 사용 모델

| 항목 | 내용 |
|------|------|
| 모델명 | `exaone3.5:7.8b` |
| 개발사 | LG AI Research |
| 특징 | 한국어·영어 이중언어 특화, 7.8B 파라미터 |
| 용량 | 약 4.4 GB |

---

## Step 1. Ollama 설치

1. [https://ollama.com/download](https://ollama.com/download) 접속
2. **Windows** 선택 후 설치 파일 다운로드
3. 설치 완료 후 터미널(PowerShell)에서 확인

```powershell
ollama --version
```

---

## Step 2. 모델 다운로드

```powershell
ollama pull exaone3.5:7.8b
```

> 약 4.7 GB 다운로드 — 네트워크 속도에 따라 수 분 소요

다운로드 완료 확인:

```powershell
ollama list
```

`exaone3.5:7.8b` 가 목록에 보이면 완료입니다.

---

## Step 3. Ollama 실행

> **Windows에 Ollama를 설치하면 시스템 트레이에서 자동으로 백그라운드 실행됩니다.**  
> 아래 두 경우 중 하나를 선택하세요.

---

### 케이스 A — 로컬에서 대시보드를 열 때 (`localhost`)

자동 실행된 Ollama를 그대로 사용하면 됩니다. 별도 실행 불필요.

설치 후 정상 실행 중인지 확인:

```powershell
ollama list
```

---

### 케이스 B — Pi에서 대시보드를 서빙할 때 (`http://sambo.local` 등)

브라우저 origin이 `localhost`가 아니므로 CORS 설정이 필요합니다.  
자동 실행된 Ollama를 종료하고 CORS를 허용하여 재실행해야 합니다.

```powershell
# 기존 자동 실행 프로세스 종료
Stop-Process -Name "ollama" -Force -ErrorAction SilentlyContinue

# CORS 허용 후 재실행
$env:OLLAMA_ORIGINS="*"
ollama serve
```

> 이 터미널 창은 대시보드를 사용하는 동안 **닫으면 안 됩니다.**

---

## Step 4. 대시보드 실행

### 방법 A — VS Code Live Server

1. VS Code에서 `sambo_robot_arm_dashboard.html` 열기
2. 우하단 `Go Live` 버튼 클릭
3. 브라우저 자동 오픈

### 방법 B — Python HTTP 서버

```powershell
cd C:\anaconda\envs\bigdata\PROJECT1\sambo
python -m http.server 8080
```

브라우저에서 `http://localhost:8080/sambo_robot_arm_dashboard.html` 접속

---

## Step 5. 챗봇 사용

1. 대시보드 우하단 **AI 작업 도우미** 버튼 클릭
2. 채팅창에 질문 입력 (예: `지금 로봇 상태 괜찮아?`, `J3 부하 확인해줘`)
3. Ollama가 현재 대시보드 데이터를 기반으로 답변

> 챗봇은 **대시보드 데이터 관련 질문에만** 답변합니다.  
> 무관한 질문은 정중히 거절합니다.

---

## 상태별 메시지

| 채팅창 표시 | 의미 | 조치 |
|-------------|------|------|
| `Ollama 응답` | 정상 작동 중 | — |
| `로컬 안내 · Ollama 미연결 fallback` | Ollama 미실행 | Step 3 확인 |
| `응답 시간이 초과됐습니다` | 60초 내 응답 없음 | Ollama 실행 여부 및 모델 로드 확인 |

---

## 문제 해결

**Q. `ollama serve` 실행 시 "port already in use" 오류**

Ollama가 이미 백그라운드에서 실행 중입니다. 별도 실행 없이 바로 사용 가능합니다.

**Q. CORS 오류가 날 때**

Step 3 케이스 B를 참고하세요. Pi 서빙 환경에서는 반드시 `OLLAMA_ORIGINS="*"` 설정 후 재실행해야 합니다.

**Q. 모델 응답이 느릴 때**

- GPU가 있으면 Ollama가 자동으로 GPU를 사용합니다
- CPU 전용 실행 시 응답에 10~30초 소요될 수 있습니다 (정상)

---

## 아키텍처 요약

```
[RPi 5]                          [노트북]
 ├─ 로봇 제어                    ├─ 브라우저 (대시보드)
 ├─ WebSocket 브릿지  ─────────→  │   └─ 로봇 데이터 수신
 └─ HTML 파일 서빙               └─ Ollama (localhost:11434)
                                      └─ 챗봇 질문/답변 처리
```
