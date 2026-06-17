#include <SPI.h>
#include <mcp_can.h>

const uint8_t CAN_CS_PIN = 10;
const uint8_t CAN_INT_PIN = 2;

// Default is A0/A1 to avoid the D4-D9 limit-switch reservation.
// Set this to 1 only when D4/D5 are confirmed unused.
#define USE_D4_D5_LED_PINS 0

#if USE_D4_D5_LED_PINS
const uint8_t RED_LED_PIN = 4;
const uint8_t GREEN_LED_PIN = 5;
#else
const uint8_t RED_LED_PIN = A0;
const uint8_t GREEN_LED_PIN = A1;
#endif

// Most low-cost MCP2515 modules are 8 MHz, but some are 16 MHz.
// Change to MCP_16MHZ if the board crystal is 16 MHz.
#define MCP_CLOCK MCP_8MHZ

const uint32_t CAN_ID_CNC_LED = 0x410;
const uint8_t LED_CMD_CNC_STATE = 0x20;
const bool LED_ACTIVE_HIGH = true;

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

MCP_CAN CAN0(CAN_CS_PIN);

uint8_t currentState = LED_CAN_LOST;
uint8_t rackCount = 0;
uint8_t rackCapacity = 0;
uint8_t remainingSec = 0;
uint8_t lastSeq = 0;

unsigned long lastCanRxMs = 0;
const unsigned long CAN_TIMEOUT_MS = 2500;

void writeLed(uint8_t pin, bool on) {
  digitalWrite(pin, on == LED_ACTIVE_HIGH ? HIGH : LOW);
}

void setLed(bool redOn, bool greenOn) {
  writeLed(RED_LED_PIN, redOn);
  writeLed(GREEN_LED_PIN, greenOn);
}

bool blinkPhase(unsigned long now, unsigned long halfPeriodMs) {
  return ((now / halfPeriodMs) % 2UL) == 0UL;
}

void updateLedPattern() {
  unsigned long now = millis();
  uint8_t stateToShow = currentState;

  if (lastCanRxMs == 0 || now - lastCanRxMs > CAN_TIMEOUT_MS) {
    stateToShow = LED_CAN_LOST;
  }

  switch (stateToShow) {
    case LED_IDLE:
      setLed(false, true);
      break;

    case LED_MACHINING:
      setLed(true, false);
      break;

    case LED_DONE_WAIT_UNLOAD: {
      bool on = blinkPhase(now, 500);
      setLed(false, on);
      break;
    }

    case LED_LOADING:
    case LED_UNLOADING: {
      bool phase = blinkPhase(now, 250);
      setLed(phase, !phase);
      break;
    }

    case LED_RACK_FULL: {
      unsigned long t = now % 1000UL;
      bool redOn = (t < 120UL) || (t >= 220UL && t < 340UL);
      setLed(redOn, false);
      break;
    }

    case LED_ERROR: {
      bool on = blinkPhase(now, 125);
      setLed(on, on);
      break;
    }

    case LED_CAN_LOST:
    default: {
      bool phase = blinkPhase(now, 700);
      setLed(phase, !phase);
      break;
    }
  }
}

void handleCanFrame(unsigned long rxId, uint8_t rxLen, uint8_t *rxBuf) {
  if ((rxId & 0x80000000UL) != 0UL) return;
  if ((rxId & 0x7FFUL) != CAN_ID_CNC_LED) return;
  if (rxLen < 2) return;
  if (rxBuf[0] != LED_CMD_CNC_STATE) return;

  currentState = rxBuf[1];

  if (rxLen >= 5) {
    rackCount = rxBuf[2];
    rackCapacity = rxBuf[3];
    remainingSec = rxBuf[4];
  }
  if (rxLen >= 7) {
    lastSeq = rxBuf[6];
  }

  lastCanRxMs = millis();
}

void pollCanRx() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId = 0;
    uint8_t rxLen = 0;
    uint8_t rxBuf[8] = {0};

    if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
      handleCanFrame(rxId, rxLen, rxBuf);
    }
  }
}

void waitForCanInit() {
  while (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_CLOCK) != CAN_OK) {
    setLed(true, true);
    delay(250);
    setLed(false, false);
    delay(250);
  }
  CAN0.setMode(MCP_NORMAL);
}

void setup() {
  Serial.begin(115200);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  setLed(false, false);
  waitForCanInit();

  currentState = LED_CAN_LOST;
  lastCanRxMs = 0;
}

void loop() {
  pollCanRx();
  updateLedPattern();
}
