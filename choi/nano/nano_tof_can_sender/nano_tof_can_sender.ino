// ============================================================
// Arduino Nano (ATmega328P 5V) + VL53L0X(ToF) + MCP2515  CAN 송신기
//   ★ ToF 라이브러리: Adafruit_VL53L0X 사용 (Pololu 대신)
//
//   [VL53L0X(TOF200C)] ──I2C(A4/A5)── [Arduino Nano] ──SPI(D10~D13)── [MCP2515] ──CAN── [RPi5]
//
// [버스 토폴로지]
//   RPi5 (CANable Pro / gs_usb)
//        └── CAN ── STM32(NUCLEO-F103RB)
//        └── CAN ── Nano (MCP2515 8MHz)  ← 이 노드 (Nano #1, ToF)
//
// [확정 버스 설정]  (CAN_통신_규약.md 기준)
//   비트레이트  125 kbps,  샘플포인트 62.5%(라이브러리 고정),  종단 60Ω,  크리스털 8MHz
//   ⚠ MCP2515 모듈 종단 점퍼 J1 = OFF (중간 노드)
//
// [CAN ID 할당]
//   0x300  Nano → RPi   ToF 거리          ← 이 코드 주력
//   0x37F  RPi → Nano   PING 요청          (진단)
//   0x3FF  Nano → RPi   PING ACK           (진단)
//
// [ToF 프레임]  ID=0x300, DLC=8, little-endian, 주기 100ms (10Hz)
//   [0..1] 거리 uint16 (mm)        ← 8190 이상이면 측정 실패/범위초과
//   [2]    status  0=OK, 1=범위초과, 2=센서초기화실패
//   [3]    (예약 0)
//   [4..5] (예약 0)
//   [6..7] 카운터 uint16           ← RPi에서 프레임 유실 검출용
//   ※ 프레임 포맷은 Pololu 버전과 동일 → RPi tof_can_rx.py 그대로 사용
//
// [핀맵]
//   MCP2515 : VCC=5V, GND=GND, CS=D10, SI=D11(MOSI), SO=D12(MISO), SCK=D13, INT=D2
//   VL53L0X : VIN=5V(TOF200C, 레귤레이터/레벨시프터 내장), GND=GND, SDA=A4, SCL=A5
//             XSHUT/GPIO1 = 미사용
//
// [필요 라이브러리]
//   - Adafruit  "Adafruit_VL53L0X"  (의존성: Adafruit BusIO 자동 설치)
//   - coryjfowler "mcp_can"
// ============================================================

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ── 진단 모드 ────────────────────────────────────────────────
// 0 = NORMAL (실제 버스 송신, ACK 필요)
// 1 = LOOPBACK (MCP2515 내부 순환, ACK 불필요)
#define LOOPBACK_TEST   0

// ── 핀/상수 ─────────────────────────────────────────────────
const uint8_t  CAN_CS_PIN     = 10;
const uint8_t  CAN_INT_PIN    = 2;

const uint32_t CAN_ID_TOF     = 0x300;
const uint32_t CAN_ID_PING_RX = 0x37F;
const uint32_t CAN_ID_PING_TX = 0x3FF;
const uint8_t  CMD_PING       = 0x7F;
const uint16_t SEND_PERIOD_MS = 100;     // 10Hz

// ToF 상수
const uint16_t TOF_OUT_OF_RANGE = 8190;  // 범위초과/실패 시 보낼 값
const uint8_t  AVG_SAMPLES      = 3;      // 이동평균 샘플 수 (노이즈 완화)
const uint8_t  RANGE_OUT_STATUS = 4;      // Adafruit RangeStatus 4 = phase fail(범위초과)

// CAN status 코드
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

// 이동평균 링버퍼
uint16_t avgBuf[AVG_SAMPLES];
uint8_t  avgIdx  = 0;
uint8_t  avgFill = 0;


// ── coryjfowler 라이브러리 리턴 코드 ─────────────────────────
const char* sendStatStr(byte s) {
  switch (s) {
    case 0:  return "OK";
    case 1:  return "FAILINIT";
    case 2:  return "FAILTX";
    case 5:  return "CTRLERROR";
    case 6:  return "GETTXBFTIMEOUT (=ACK 없음, 버스 한쪽 침묵)";
    case 7:  return "SENDMSGTIMEOUT";
    default: return "?";
  }
}


// ── ToF 1회 측정 + 이동평균 (Adafruit API) ──────────────────
// 반환: 평균 거리(mm). status는 out 파라미터.
uint16_t readToF(uint8_t &status) {
  if (!tofReady) { status = ST_SENSORERR; return TOF_OUT_OF_RANGE; }

  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);   // 단발 측정 (블로킹 ~30ms)

  if (m.RangeStatus == RANGE_OUT_STATUS) {   // 범위초과
    status = ST_OUTRANGE;
    return TOF_OUT_OF_RANGE;
  }

  uint16_t mm = m.RangeMilliMeter;

  // 정상값만 이동평균에 반영
  avgBuf[avgIdx] = mm;
  avgIdx = (avgIdx + 1) % AVG_SAMPLES;
  if (avgFill < AVG_SAMPLES) avgFill++;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < avgFill; i++) sum += avgBuf[i];

  status = ST_OK;
  return (uint16_t)(sum / avgFill);
}


// ── 진단 출력 (1초마다) ─────────────────────────────────────
void printDiagnostics() {
  if (millis() - lastDiagMs < 1000) return;
  lastDiagMs = millis();

  uint8_t tec  = CAN0.errorCountTX();
  uint8_t rec  = CAN0.errorCountRX();
  uint8_t eflg = CAN0.getError();

  Serial.print(F("  [DIAG] TEC=")); Serial.print(tec);
  Serial.print(F(" REC="));         Serial.print(rec);
  Serial.print(F(" EFLG=0x"));      Serial.print(eflg, HEX);
  Serial.print(F(" lastStat="));    Serial.print(lastSendStat);
  Serial.print(F(" ("));            Serial.print(sendStatStr(lastSendStat));
  Serial.println(F(")"));

  if (eflg & 0x04) Serial.println(F("    TXWAR  : TX 에러 >=96  <- ACK 못 받는 중"));
  if (eflg & 0x10) Serial.println(F("    TXEP   : TX 패시브   <- 거의 버스에 단독"));
  if (eflg & 0x20) Serial.println(F("    TXBO   : Bus-Off!    <- 배선/타이밍 심각"));
}


// ── ToF 프레임 송신 ─────────────────────────────────────────
void sendToFFrame() {
  uint8_t  status;
  uint16_t dist = readToF(status);

  uint8_t buf[8];
  buf[0] = (uint8_t)( dist           & 0xFF);
  buf[1] = (uint8_t)((dist    >> 8)  & 0xFF);
  buf[2] = status;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = 0;
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


// ── ping 응답 (checkReceive 폴링) ───────────────────────────
void pollCanRx() {
  if (CAN0.checkReceive() != CAN_MSGAVAIL) return;

  long unsigned int rxId = 0;
  uint8_t rxLen = 0;
  uint8_t rxBuf[8];

  if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) != CAN_OK) return;
  if (rxLen > 8) return;  // 깨진 프레임 방어

  if (rxId == CAN_ID_PING_RX && rxLen >= 1 && rxBuf[0] == CMD_PING) {
    uint8_t ack[3] = { CMD_PING, 0, 0x00 };
    CAN0.sendMsgBuf(CAN_ID_PING_TX, 0, 3, ack);
    Serial.println(F("PING ACK 송신"));
  }
}


// ── setup / loop ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== Nano ToF(VL53L0X/Adafruit) CAN sender ==="));

  // --- VL53L0X (I2C, Adafruit) 초기화 ---
  // ※ LONG_RANGE 프로파일은 VCSEL period를 바꿔 재캘리브레이션을 유발하는데,
  //   TOF200C 클론 모듈이 이를 거부해 begin()이 실패함(st=2). → 기본 begin() 사용.
  Wire.begin();
  if (tof.begin()) {            // 기본 주소 0x29, 검증된 호출
    tofReady = true;
    Serial.println(F("VL53L0X init OK (Adafruit, addr=0x29)"));
  } else {
    tofReady = false;
    Serial.println(F("VL53L0X init FAIL <- I2C 배선/주소/전원 확인"));
  }

  // --- MCP2515 (SPI/CAN) 초기화 ---
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

  Serial.println(F("Sender ready. ToF ID=0x300, period=100ms"));
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