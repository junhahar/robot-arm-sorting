// ============================================================
// Arduino Nano + VL53L0X(ToF) + MCP2515  CAN 송신기
//   ★ 변형: HIGH ACCURACY (정밀) 모드  ─ 적분시간 200ms
//   ★ ToF 라이브러리: Adafruit_VL53L0X
//
//   [VL53L0X(TOF200C)] ──I2C(A4/A5)── [Nano] ──SPI(D10~D13)── [MCP2515] ──CAN── [RPi5]
//
// [이 버전의 특징]
//   - configSensor(HIGH_ACCURACY) = 측정 적분시간 200ms
//   - VCSEL period를 안 건드려서 TOF200C 클론에서도 안전하게 적용됨
//   - 장점: 측정값 지터 감소(정밀도↑), 긴 적분으로 사거리도 기본보다 약간↑
//   - 단점: 한 번 측정에 ~200ms → 송신 주기 5Hz로 느려짐
//
// [CAN 프레임]  ID=0x300, DLC=8, little-endian  (DEFAULT/LONG 버전과 동일)
//   [0..1] 거리 uint16(mm), [2] status(0 OK/1 범위초과/2 센서오류),
//   [3..5] 예약0, [6..7] 카운터 uint16
//   → RPi tof_can_rx.py 그대로 사용
//
// [핀맵]
//   MCP2515 : CS=D10, SI=D11, SO=D12, SCK=D13, INT=D2, VCC=5V, GND=GND
//   VL53L0X : VIN=5V(TOF200C), GND=GND, SDA=A4, SCL=A5
//
// [라이브러리]  Adafruit_VL53L0X (+BusIO), coryjfowler mcp_can
// ============================================================

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define LOOPBACK_TEST   0

const uint8_t  CAN_CS_PIN     = 10;
const uint8_t  CAN_INT_PIN    = 2;

const uint32_t CAN_ID_TOF     = 0x300;
const uint32_t CAN_ID_PING_RX = 0x37F;
const uint32_t CAN_ID_PING_TX = 0x3FF;
const uint8_t  CMD_PING       = 0x7F;
const uint16_t SEND_PERIOD_MS = 200;     // 정밀 모드 측정시간(200ms)에 맞춰 5Hz

const uint16_t TOF_OUT_OF_RANGE = 8190;
const uint8_t  AVG_SAMPLES      = 3;
const uint8_t  RANGE_OUT_STATUS = 4;     // Adafruit RangeStatus 4 = 범위초과

const uint8_t  ST_OK        = 0;
const uint8_t  ST_OUTRANGE  = 1;
const uint8_t  ST_SENSORERR = 2;

MCP_CAN          CAN0(CAN_CS_PIN);
Adafruit_VL53L0X tof;

uint16_t counter      = 0;
uint32_t lastSendMs   = 0;
uint32_t lastDiagMs   = 0;
byte     lastSendStat = 0;
bool     tofReady     = false;

uint16_t avgBuf[AVG_SAMPLES];
uint8_t  avgIdx  = 0;
uint8_t  avgFill = 0;


const char* sendStatStr(byte s) {
  switch (s) {
    case 0:  return "OK";
    case 1:  return "FAILINIT";
    case 2:  return "FAILTX";
    case 5:  return "CTRLERROR";
    case 6:  return "GETTXBFTIMEOUT (=ACK 없음)";
    case 7:  return "SENDMSGTIMEOUT";
    default: return "?";
  }
}


uint16_t readToF(uint8_t &status) {
  if (!tofReady) { status = ST_SENSORERR; return TOF_OUT_OF_RANGE; }

  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);   // 정밀 모드: 블로킹 ~200ms

  if (m.RangeStatus == RANGE_OUT_STATUS) { status = ST_OUTRANGE; return TOF_OUT_OF_RANGE; }

  uint16_t mm = m.RangeMilliMeter;
  avgBuf[avgIdx] = mm;
  avgIdx = (avgIdx + 1) % AVG_SAMPLES;
  if (avgFill < AVG_SAMPLES) avgFill++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < avgFill; i++) sum += avgBuf[i];

  status = ST_OK;
  return (uint16_t)(sum / avgFill);
}


void printDiagnostics() {
  if (millis() - lastDiagMs < 1000) return;
  lastDiagMs = millis();

  Serial.print(F("  [DIAG] TEC=")); Serial.print(CAN0.errorCountTX());
  Serial.print(F(" REC="));         Serial.print(CAN0.errorCountRX());
  Serial.print(F(" EFLG=0x"));      Serial.print(CAN0.getError(), HEX);
  Serial.print(F(" lastStat="));    Serial.print(lastSendStat);
  Serial.print(F(" ("));            Serial.print(sendStatStr(lastSendStat));
  Serial.println(F(")"));
}


void sendToFFrame() {
  uint8_t  status;
  uint16_t dist = readToF(status);

  uint8_t buf[8];
  buf[0] = (uint8_t)( dist           & 0xFF);
  buf[1] = (uint8_t)((dist    >> 8)  & 0xFF);
  buf[2] = status;
  buf[3] = 0; buf[4] = 0; buf[5] = 0;
  buf[6] = (uint8_t)( counter        & 0xFF);
  buf[7] = (uint8_t)((counter >> 8)  & 0xFF);

  byte stat = CAN0.sendMsgBuf(CAN_ID_TOF, 0, 8, buf);
  lastSendStat = stat;

  if (stat == CAN_OK) {
    Serial.print(F("TX OK  cnt=")); Serial.print(counter);
    Serial.print(F("  dist="));     Serial.print(dist);
    Serial.print(F("mm  st="));      Serial.print(status);
    if (status == ST_OUTRANGE)  Serial.print(F(" (범위초과)"));
    if (status == ST_SENSORERR) Serial.print(F(" (센서오류)"));
    Serial.println();
  } else {
    Serial.print(F("TX FAIL stat=")); Serial.print(stat);
    Serial.print(F(" ("));            Serial.print(sendStatStr(stat));
    Serial.println(F(")"));
  }
  counter++;
}


void pollCanRx() {
  if (CAN0.checkReceive() != CAN_MSGAVAIL) return;
  long unsigned int rxId = 0;
  uint8_t rxLen = 0, rxBuf[8];
  if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) != CAN_OK) return;
  if (rxLen > 8) return;
  if (rxId == CAN_ID_PING_RX && rxLen >= 1 && rxBuf[0] == CMD_PING) {
    uint8_t ack[3] = { CMD_PING, 0, 0x00 };
    CAN0.sendMsgBuf(CAN_ID_PING_TX, 0, 3, ack);
    Serial.println(F("PING ACK 송신"));
  }
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== Nano ToF CAN sender (HIGH ACCURACY) ==="));

  // --- VL53L0X 초기화: 기본 begin() 후 정밀 프로파일 적용 ---
  Wire.begin();
  if (tof.begin()) {                 // 클론에서 검증된 기본 초기화
    tofReady = true;
    // HIGH_ACCURACY = 적분 200ms. VCSEL 미변경 → 클론 안전.
    if (tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
      Serial.println(F("VL53L0X init OK (HIGH_ACCURACY, 적분 200ms)"));
    } else {
      Serial.println(F("VL53L0X init OK, 단 HIGH_ACCURACY 미적용 -> 기본 유지"));
    }
  } else {
    tofReady = false;
    Serial.println(F("VL53L0X init FAIL <- USB 완전히 뽑았다 꽂기/배선(SDA=A4,SCL=A5,VIN) 확인"));
  }

  // --- MCP2515 ---
  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  while (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("MCP2515 init fail, retry..."));
    delay(1000);
  }
  Serial.println(F("MCP2515 init OK (125kbps, 8MHz)"));

#if LOOPBACK_TEST
  CAN0.setMode(MCP_LOOPBACK);
  Serial.println(F(">>> LOOPBACK MODE <<<"));
#else
  CAN0.setMode(MCP_NORMAL);
  Serial.println(F("NORMAL MODE"));
#endif

  Serial.println(F("Sender ready. ToF ID=0x300, period=200ms (5Hz)"));
}

void loop() {
  pollCanRx();
  uint32_t now = millis();
  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendToFFrame();
  }
  printDiagnostics();
}
