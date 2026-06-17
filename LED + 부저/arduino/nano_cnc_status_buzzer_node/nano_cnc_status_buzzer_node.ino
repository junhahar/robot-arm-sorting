#include <SPI.h>
#include <mcp_can.h>

const uint8_t CAN_CS_PIN = 10;
const uint8_t CAN_INT_PIN = 2;

const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 5;
const uint8_t BUZZER_PIN = 6;
const uint8_t RESET_BTN_PIN = 7;

// Most low-cost MCP2515 modules are 8 MHz, but some are 16 MHz.
// Change to MCP_16MHZ if the board crystal is 16 MHz.
#define MCP_CLOCK MCP_8MHZ

// 0 = active buzzer, 1 = passive piezo buzzer driven by tone().
#define PASSIVE_BUZZER 0

const uint32_t CAN_ID_CNC_STATUS = 0x410;  // RPi -> display Nano
const uint32_t CAN_ID_RACK_RESET = 0x411;  // display Nano -> RPi

const uint8_t CMD_CNC_STATUS = 0x20;
const uint8_t CMD_RACK_RESET = 0x31;
const bool LED_ACTIVE_HIGH = true;
const bool BUZZER_ACTIVE_HIGH = true;
const uint16_t BUZZER_FREQ_HZ = 2400;

enum CncStatus : uint8_t {
  ST_IDLE = 0,
  ST_MACHINING = 1,
  ST_DONE_WAIT = 2,
  ST_RACK_FULL = 3,
  ST_ERROR = 4,
  ST_CAN_LOST = 255
};

MCP_CAN CAN0(CAN_CS_PIN);

uint8_t currentState = ST_CAN_LOST;
uint8_t rackCount = 0;
uint8_t rackCapacity = 0;
uint8_t remainingSec = 0;
uint8_t lastRxSeq = 0;
uint8_t txSeq = 0;

unsigned long lastCanRxMs = 0;
unsigned long lastResetTxMs = 0;
const unsigned long CAN_TIMEOUT_MS = 2500;
const unsigned long RESET_REPEAT_BLOCK_MS = 1000;
const unsigned long BUTTON_DEBOUNCE_MS = 30;

void writeLed(uint8_t pin, bool on) {
  digitalWrite(pin, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

void setBuzzer(bool on) {
#if PASSIVE_BUZZER
  if (on) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
  } else {
    noTone(BUZZER_PIN);
  }
#else
  digitalWrite(BUZZER_PIN, on == BUZZER_ACTIVE_HIGH ? HIGH : LOW);
#endif
}

void setOutputs(bool redOn, bool greenOn, bool buzzerOn) {
  writeLed(RED_LED_PIN, redOn);
  writeLed(GREEN_LED_PIN, greenOn);
  setBuzzer(buzzerOn);
}

bool phase(unsigned long now, unsigned long halfPeriodMs) {
  return ((now / halfPeriodMs) % 2UL) == 0UL;
}

bool doublePulse(unsigned long now, unsigned long cycleMs) {
  unsigned long t = now % cycleMs;
  return (t < 120UL) || (t >= 220UL && t < 340UL);
}

void updateOutputPattern() {
  unsigned long now = millis();
  uint8_t stateToShow = currentState;

  if (lastCanRxMs == 0 || now - lastCanRxMs > CAN_TIMEOUT_MS) {
    stateToShow = ST_CAN_LOST;
  }

  switch (stateToShow) {
    case ST_IDLE:
      setOutputs(false, true, false);
      break;

    case ST_MACHINING:
      setOutputs(true, false, false);
      break;

    case ST_DONE_WAIT: {
      bool greenBlink = phase(now, 500);
      setOutputs(false, greenBlink, false);
      break;
    }

    case ST_RACK_FULL: {
      bool redPulse = doublePulse(now, 1200);
      bool buzzerPulse = doublePulse(now, 1800);
      setOutputs(redPulse, false, buzzerPulse);
      break;
    }

    case ST_ERROR: {
      bool blink = phase(now, 120);
      setOutputs(blink, blink, blink);
      break;
    }

    case ST_CAN_LOST:
    default: {
      bool p = phase(now, 650);
      setOutputs(p, !p, false);
      break;
    }
  }
}

void handleCncStatusFrame(unsigned long rxId, uint8_t rxLen, uint8_t *rxBuf) {
  if ((rxId & 0x80000000UL) != 0UL) return;
  if ((rxId & 0x7FFUL) != CAN_ID_CNC_STATUS) return;
  if (rxLen < 2) return;
  if (rxBuf[0] != CMD_CNC_STATUS) return;

  currentState = rxBuf[1];

  if (rxLen >= 5) {
    rackCount = rxBuf[2];
    rackCapacity = rxBuf[3];
    remainingSec = rxBuf[4];
  }
  if (rxLen >= 7) {
    lastRxSeq = rxBuf[6];
  }

  lastCanRxMs = millis();
}

void pollCanRx() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8] = {0};

    if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
      handleCncStatusFrame(rxId, rxLen, rxBuf);
    }
  }
}

void sendRackResetRequest() {
  uint8_t buf[8] = {
    CMD_RACK_RESET,
    1,
    txSeq++,
    0, 0, 0, 0, 0
  };

  CAN0.sendMsgBuf(CAN_ID_RACK_RESET, 0, 8, buf);
}

void pollResetButton() {
  unsigned long now = millis();

  static bool lastStablePressed = false;
  static bool lastRawPressed = false;
  static unsigned long lastChangeMs = 0;

  bool rawPressed = (digitalRead(RESET_BTN_PIN) == LOW);

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastChangeMs = now;
  }

  if (now - lastChangeMs < BUTTON_DEBOUNCE_MS) return;

  bool stablePressed = rawPressed;

  if (stablePressed && !lastStablePressed) {
    if (now - lastResetTxMs > RESET_REPEAT_BLOCK_MS) {
      sendRackResetRequest();
      lastResetTxMs = now;
    }
  }

  lastStablePressed = stablePressed;
}

void waitForCanInit() {
  while (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_CLOCK) != CAN_OK) {
    setOutputs(true, true, false);
    delay(250);
    setOutputs(false, false, false);
    delay(250);
  }
  CAN0.setMode(MCP_NORMAL);
}

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  setOutputs(false, false, false);
  waitForCanInit();

  currentState = ST_CAN_LOST;
  lastCanRxMs = 0;

  Serial.println(F("Sambo CNC LED+buzzer display Nano ready"));
}

void loop() {
  pollCanRx();
  pollResetButton();
  updateOutputPattern();
}
