#include <SPI.h>
#include <mcp_can.h>

// Sambo display-only Nano.
// Raspberry Pi decides the workflow state and sends CAN 0x410.
// This Nano only renders LEDs and the buzzer.

const uint8_t CAN_CS_PIN = 10;
const uint8_t CAN_INT_PIN = 2;

const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 5;
const uint8_t BUZZER_PIN = 6;
const uint8_t BLUE_LED_PIN = 7;

// Most low-cost MCP2515 modules are 8 MHz, but some are 16 MHz.
// Change to MCP_16MHZ if the board crystal is 16 MHz.
#define MCP_CLOCK MCP_8MHZ

// 0 = active buzzer, 1 = passive piezo buzzer driven by tone().
#define PASSIVE_BUZZER 1

const uint32_t CAN_ID_CNC_STATUS = 0x410;  // RPi -> display Nano
const uint8_t CMD_CNC_STATUS = 0x20;

const bool LED_ACTIVE_HIGH = true;
const bool BUZZER_ACTIVE_HIGH = true;
const uint16_t BUZZER_FREQ_HZ = 2400;

enum CncStatus : uint8_t {
  ST_CNC_EMPTY = 0,
  ST_CNC_OCCUPIED = 1,
  ST_CNC_DONE_WAIT = 2,
  ST_RACK_LOADED = 3,
  ST_RACK_FULL = 4,
  ST_ERROR = 5,
  ST_CAN_LOST = 255
};

MCP_CAN CAN0(CAN_CS_PIN);

uint8_t currentState = ST_CAN_LOST;
uint8_t rackCount = 0;
uint8_t rackCapacity = 0;
uint8_t remainingSec = 0;
uint8_t lastRxSeq = 0;

unsigned long lastCanRxMs = 0;
unsigned long stateStartMs = 0;
const unsigned long CAN_TIMEOUT_MS = 2500;

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

void setOutputs(bool redOn, bool greenOn, bool blueOn, bool buzzerOn) {
  writeLed(RED_LED_PIN, redOn);
  writeLed(GREEN_LED_PIN, greenOn);
  writeLed(BLUE_LED_PIN, blueOn);
  setBuzzer(buzzerOn);
}

bool phase(unsigned long now, unsigned long halfPeriodMs) {
  return ((now / halfPeriodMs) % 2UL) == 0UL;
}

bool triplePulseOnce(unsigned long now) {
  unsigned long t = now - stateStartMs;
  return (t < 180UL) ||
         (t >= 350UL && t < 530UL) ||
         (t >= 700UL && t < 880UL);
}

bool doublePulseRepeat(unsigned long now, unsigned long cycleMs) {
  unsigned long t = (now - stateStartMs) % cycleMs;
  return (t < 150UL) || (t >= 300UL && t < 450UL);
}

bool fastPulseRepeat(unsigned long now) {
  unsigned long t = (now - stateStartMs) % 300UL;
  return t < 120UL;
}

void setState(uint8_t nextState) {
  if (nextState == currentState) return;
  currentState = nextState;
  stateStartMs = millis();

  Serial.print(F("state="));
  Serial.print(currentState);
  Serial.print(F(" rack="));
  Serial.print(rackCount);
  Serial.print(F("/"));
  Serial.print(rackCapacity);
  Serial.print(F(" remain="));
  Serial.println(remainingSec);
}

void updateOutputPattern() {
  unsigned long now = millis();
  uint8_t stateToShow = currentState;

  if (lastCanRxMs == 0 || now - lastCanRxMs > CAN_TIMEOUT_MS) {
    stateToShow = ST_CAN_LOST;
  }

  switch (stateToShow) {
    case ST_CNC_EMPTY:
      setOutputs(false, false, true, false);
      break;

    case ST_CNC_OCCUPIED:
      setOutputs(true, false, false, false);
      break;

    case ST_CNC_DONE_WAIT: {
      bool greenBlink = phase(now, 700);
      setOutputs(false, greenBlink, false, false);
      break;
    }

    case ST_RACK_LOADED:
      setOutputs(false, false, true, triplePulseOnce(now));
      break;

    case ST_RACK_FULL:
      setOutputs(false, false, false, doublePulseRepeat(now, 1400));
      break;

    case ST_ERROR: {
      bool blink = phase(now, 150);
      setOutputs(blink, blink, blink, fastPulseRepeat(now));
      break;
    }

    case ST_CAN_LOST:
    default: {
      unsigned long step = (now / 350UL) % 4UL;
      setOutputs(step == 0, step == 1, step == 2, false);
      break;
    }
  }
}

void handleCncStatusFrame(unsigned long rxId, uint8_t rxLen, uint8_t *rxBuf) {
  if ((rxId & 0x80000000UL) != 0UL) return;
  if ((rxId & 0x7FFUL) != CAN_ID_CNC_STATUS) return;
  if (rxLen < 2) return;
  if (rxBuf[0] != CMD_CNC_STATUS) return;

  if (rxLen >= 5) {
    rackCount = rxBuf[2];
    rackCapacity = rxBuf[3];
    remainingSec = rxBuf[4];
  }
  if (rxLen >= 7) {
    lastRxSeq = rxBuf[6];
  }

  lastCanRxMs = millis();
  setState(rxBuf[1]);
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

void waitForCanInit() {
  while (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_CLOCK) != CAN_OK) {
    setOutputs(true, true, true, false);
    delay(250);
    setOutputs(false, false, false, false);
    delay(250);
  }
  CAN0.setMode(MCP_NORMAL);
}

void startupTest() {
  setOutputs(false, false, false, false);
  delay(300);

  setOutputs(true, false, false, false);
  delay(350);
  setOutputs(false, true, false, false);
  delay(350);
  setOutputs(false, false, true, false);
  delay(350);
  setOutputs(false, false, false, true);
  delay(200);
  setOutputs(false, false, false, false);
  delay(300);
}

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  startupTest();
  waitForCanInit();

  currentState = ST_CAN_LOST;
  stateStartMs = millis();
  lastCanRxMs = 0;

  Serial.println(F("Sambo CNC RGB LED+buzzer display Nano ready"));
}

void loop() {
  pollCanRx();
  updateOutputPattern();
}
