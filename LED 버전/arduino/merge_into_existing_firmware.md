# 기존 Nano 펌웨어에 LED 기능 병합하기

이 문서는 같은 Arduino Nano가 STS3215/URT, MCP2515, TOF, E-Stop까지 담당하는 경우를 위한 병합 안내입니다. 실제 로봇 Nano에 `cnc_led_status_nano.ino`를 통째로 올리면 기존 서보 제어 코드가 사라지므로, 아래 LED 수신부만 합칩니다.

## 1. 기존 CAN 속도 확인

현재 프로젝트의 최종 CAN 기준은 다음입니다.

```text
125 kbps
Standard 11-bit
sample-point 62.5%
```

Arduino 코드에서는 보통 `CAN_125KBPS`로 설정합니다. 이미 운용 중인 펌웨어가 다른 속도를 쓰고 있으면 RPi `can0` 설정과 반드시 맞춰야 합니다.

MCP2515 크리스털은 `MCP_8MHZ` 또는 `MCP_16MHZ` 중 실제 보드에 맞게 선택해야 합니다. 사양 확인 필요.

## 2. 핀 상수 추가

기존 펌웨어 상단에 LED 핀과 프레임 ID를 추가합니다.

```cpp
#define USE_D4_D5_LED_PINS 0

#if USE_D4_D5_LED_PINS
const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 5;
#else
const uint8_t RED_LED_PIN = A0;
const uint8_t GREEN_LED_PIN = A1;
#endif

const uint32_t CAN_ID_CNC_LED = 0x410;
const uint8_t LED_CMD_CNC_STATE = 0x20;
const bool LED_ACTIVE_HIGH = true;
```

A0/A1 기본값을 권장합니다. D4~D9는 리미트 스위치 확장 후보 핀이므로 D4/D5는 확실히 비어 있을 때만 사용합니다.

## 3. 상태 enum과 변수 추가

```cpp
enum CncLedState : uint8_t {
  LED_IDLE = 0,
  LED_LOADING = 1,
  LED_MACHINING = 2,
  LED_DONE_WAIT_UNLOAD = 3,
  LED_UNLOADING = 4,
  LED_RACK_FULL = 5,
  LED_ERROR = 6,
  LED_CAN_LOST = 255
};

uint8_t currentLedState = LED_CAN_LOST;
uint8_t ledRackCount = 0;
uint8_t ledRackCapacity = 0;
uint8_t ledRemainingSec = 0;
uint8_t ledLastSeq = 0;

unsigned long lastLedCanRxMs = 0;
const unsigned long LED_CAN_TIMEOUT_MS = 2500;
```

기존 펌웨어에 `currentState` 같은 이름이 있으면 충돌을 피하려고 `currentLedState`처럼 LED 접두어를 붙입니다.

## 4. LED 함수 추가

```cpp
void writeLed(uint8_t pin, bool on) {
  digitalWrite(pin, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

void setCncLed(bool redOn, bool greenOn) {
  writeLed(RED_LED_PIN, redOn);
  writeLed(GREEN_LED_PIN, greenOn);
}

bool ledBlinkPhase(unsigned long now, unsigned long halfPeriodMs) {
  return ((now / halfPeriodMs) % 2UL) == 0UL;
}

void updateCncLedPattern() {
  unsigned long now = millis();
  uint8_t stateToShow = currentLedState;

  if (lastLedCanRxMs == 0 || now - lastLedCanRxMs > LED_CAN_TIMEOUT_MS) {
    stateToShow = LED_CAN_LOST;
  }

  switch (stateToShow) {
    case LED_IDLE:
      setCncLed(false, true);
      break;
    case LED_MACHINING:
      setCncLed(true, false);
      break;
    case LED_DONE_WAIT_UNLOAD: {
      bool on = ledBlinkPhase(now, 500);
      setCncLed(false, on);
      break;
    }
    case LED_LOADING:
    case LED_UNLOADING: {
      bool phase = ledBlinkPhase(now, 250);
      setCncLed(phase, !phase);
      break;
    }
    case LED_RACK_FULL: {
      unsigned long t = now % 1000UL;
      bool redOn = (t < 120UL) || (t >= 220UL && t < 340UL);
      setCncLed(redOn, false);
      break;
    }
    case LED_ERROR: {
      bool on = ledBlinkPhase(now, 125);
      setCncLed(on, on);
      break;
    }
    case LED_CAN_LOST:
    default: {
      bool phase = ledBlinkPhase(now, 700);
      setCncLed(phase, !phase);
      break;
    }
  }
}
```

## 5. CAN 수신 처리 추가

기존 CAN receive switch 전에 아래 함수를 호출하고, LED 프레임이면 기존 서보 명령 switch로 내려가지 않게 처리합니다.

```cpp
bool handleCncLedFrame(unsigned long rxId, uint8_t rxLen, uint8_t *rxBuf) {
  if ((rxId & 0x80000000UL) != 0UL) return false;
  if ((rxId & 0x7FFUL) != CAN_ID_CNC_LED) return false;
  if (rxLen < 2) return true;
  if (rxBuf[0] != LED_CMD_CNC_STATE) return true;

  currentLedState = rxBuf[1];

  if (rxLen >= 5) {
    ledRackCount = rxBuf[2];
    ledRackCapacity = rxBuf[3];
    ledRemainingSec = rxBuf[4];
  }
  if (rxLen >= 7) {
    ledLastSeq = rxBuf[6];
  }

  lastLedCanRxMs = millis();
  return true;
}
```

기존 loop의 CAN 수신부 예시:

```cpp
if (CAN.readMsgBuf(&canId, &len, buf) != CAN_OK) return;

if (handleCncLedFrame(canId, len, buf)) {
  return;
}

switch (canId) {
  // 기존 0x100 서보 명령, TOF, E-Stop 처리 유지
}
```

## 6. setup과 loop에 연결

`setup()`에 LED 출력 초기화를 추가합니다.

```cpp
pinMode(RED_LED_PIN, OUTPUT);
pinMode(GREEN_LED_PIN, OUTPUT);
setCncLed(false, false);
```

`loop()`에서는 E-Stop으로 조기 return하기 전에도 LED 패턴을 갱신하는 편이 좋습니다.

```cpp
void loop() {
  updateCncLedPattern();

  // 기존 E-Stop, CAN receive, servo control 로직
}
```

E-Stop 상태에서 서보 전원은 물리적으로 차단되어야 하지만, Nano 5V가 살아 있다면 LED는 오류 또는 통신 상태를 계속 보여줄 수 있습니다.
