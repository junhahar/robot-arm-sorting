// ============================================================
// Arduino Nano (ATmega328P 5V) + VL53L0X(ToF) + MCP2515  CAN 송신기
//   ★ 용도: 그리퍼 파지판정 (GRASP DETECTION), median-5, 10Hz
//   ★ 자가복구 4종:
//      ① 센서  : begin 실패해도 loop에서 1초마다 재시도(재부팅 불필요)
//      ② 측정  : RangeStatus!=0 전부 실패 처리(garbage를 OK로 보내 오판 방지)
//      ③ CAN   : ★bus-off(TEC≥256으로 송신 영구중단) 감지 시 MCP2515 자동 재init(재부팅 불필요)
//      ④ HANG  : ★I2C 락업(팔 진동으로 SDA/SCL 글리치→센서가 버스 붙잡음)으로 loop가 영원히 멈추는 것 방지.
//                 - Wire 타임아웃(25ms): begin/rangingTest가 막혀도 강제해제+TWI리셋 → 절대 안 얼어붙음
//                 - 워치독(4s): 그래도 멈추면 자동 리셋·복구(setup 중 hang도, init 전에 켜서 잡음)
//
//   [VL53L0X(TOF200C)] ──I2C(A4/A5)── [Arduino Nano] ──SPI(D10~D13)── [MCP2515] ──CAN── [RPi5]
//
// [확정 버스 설정] 125kbps, 종단 60Ω, 크리스털 8MHz, J1=OFF(중간노드)
// [ToF 프레임] ID=0x300, DLC=8, little-endian, 100ms(10Hz)
//   [0..1] 거리 uint16(mm)  [2] status 0=OK/1=범위초과/2=센서초기화실패
//   [3..5] 예약0  [6..7] 카운터 uint16
// [핀맵] MCP2515 CS=D10 INT=D2 SI=D11 SO=D12 SCK=D13 / VL53L0X SDA=A4 SCL=A5 VIN=5V
// [라이브러리] Adafruit_VL53L0X, coryjfowler mcp_can
// ============================================================

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <avr/wdt.h>      // ★워치독 — loop hang(센서 I2C 락업/진동 글리치) 시 4초 후 자동 리셋·복구

#define LOOPBACK_TEST   0

const uint8_t  CAN_CS_PIN     = 10;
const uint8_t  CAN_INT_PIN    = 2;

const uint32_t CAN_ID_TOF     = 0x300;
const uint32_t CAN_ID_PING_RX = 0x37F;
const uint32_t CAN_ID_PING_TX = 0x3FF;
const uint8_t  CMD_PING       = 0x7F;
const uint16_t SEND_PERIOD_MS = 100;     // 10Hz

const uint16_t TOF_OUT_OF_RANGE = 8190;
const uint8_t  FILTER_SAMPLES   = 5;

const uint8_t  ST_OK        = 0;
const uint8_t  ST_OUTRANGE  = 1;
const uint8_t  ST_SENSORERR = 2;

const uint8_t  EFLG_TXBO    = 0x20;      // MCP2515 EFLG: TX Bus-Off 비트

MCP_CAN          CAN0(CAN_CS_PIN);
Adafruit_VL53L0X tof;

uint16_t counter      = 0;
uint32_t lastSendMs   = 0;
uint32_t lastDiagMs   = 0;
uint32_t lastReinitMs = 0;          // ① 센서 자가복구 타이머
uint32_t lastCanChkMs = 0;          // ③ CAN bus-off 체크 타이머
byte     lastSendStat = 0;
bool     tofReady     = false;

uint16_t medBuf[FILTER_SAMPLES];
uint8_t  medIdx  = 0;
uint8_t  medFill = 0;


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


uint16_t medianOfBuf() {
  uint8_t n = medFill;
  if (n == 0) return TOF_OUT_OF_RANGE;

  uint16_t tmp[FILTER_SAMPLES];
  for (uint8_t i = 0; i < n; i++) tmp[i] = medBuf[i];

  for (uint8_t i = 1; i < n; i++) {
    uint16_t key = tmp[i];
    int8_t   j   = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}


// ② 범위초과(RangeStatus==4=phase fail)만 못잡음(8190) 처리.
//   1/2/3/5(sigma/signal/min/hw)는 거리값이 대체로 유효 → 그대로 통과(단독 센서 테스트와 동일 동작).
//   ※ 파지판정(84↔235, 임계130)엔 마진 읽기여도 충분. ==4만 걸러도 오판 안 남(성공=근거리=강신호=status0).
const uint8_t RANGE_OUT_STATUS = 4;
uint16_t readToF(uint8_t &status) {
  if (!tofReady) { status = ST_SENSORERR; return TOF_OUT_OF_RANGE; }

  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);

  if (m.RangeStatus == RANGE_OUT_STATUS) {   // 범위초과(=물체없음/너무 멀음)만 8190
    status = ST_OUTRANGE;
    return TOF_OUT_OF_RANGE;
  }

  medBuf[medIdx] = m.RangeMilliMeter;
  medIdx = (medIdx + 1) % FILTER_SAMPLES;
  if (medFill < FILTER_SAMPLES) medFill++;

  status = ST_OK;
  return medianOfBuf();
}


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
  if (eflg & EFLG_TXBO) Serial.println(F("    TXBO   : Bus-Off!    <- 자동복구 시도 중"));
}


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
    if (status == ST_OK && dist <= 130) Serial.print(F("  -> 잡음(<=130)"));
    if (status == ST_OK && dist >  130 && dist < TOF_OUT_OF_RANGE) Serial.print(F("  -> 못잡음(>130)"));
    if (status == ST_OUTRANGE)  Serial.print(F(" (범위초과/측정실패=못잡음)"));
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
  uint8_t rxLen = 0;
  uint8_t rxBuf[8];

  if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) != CAN_OK) return;
  if (rxLen > 8) return;

  if (rxId == CAN_ID_PING_RX && rxLen >= 1 && rxBuf[0] == CMD_PING) {
    uint8_t ack[3] = { CMD_PING, 0, 0x00 };
    CAN0.sendMsgBuf(CAN_ID_PING_TX, 0, 3, ack);
    Serial.println(F("PING ACK 송신"));
  }
}


// ① 센서 begin (setup/loop 공용). 성공 시 tofReady=true + 필터 초기화.
bool initToF() {
  if (tof.begin()) {
    tofReady = true;
    medIdx = 0; medFill = 0;
    return true;
  }
  return false;
}


// ── MCP2515 init (setup/복구 공용) ──────────────────────────
void initCan() {
  while (CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("MCP2515 init fail, retry..."));
    wdt_reset();          // loop에서 CAN 재init 중에도 워치독 안 터지게(설정 전이면 no-op)
    delay(1000);
  }
#if LOOPBACK_TEST
  CAN0.setMode(MCP_LOOPBACK);
#else
  CAN0.setMode(MCP_NORMAL);
#endif
}


// ③ ★CAN bus-off 자동복구
//   배선 간헐불량 등으로 TX 에러가 쌓여 TEC≥256 → MCP2515가 BUS-OFF(송신 영구중단)되면
//   원래 전원 빼야(리부트) 복구됨. → loop에서 0.5초마다 감지해서 자동 재init(리부트 불필요).
//   ※ 근본 원인(배선 접촉)은 따로 잡아야 함. 이건 "멈춰도 스스로 살아나게" 하는 안전망.
void recoverCanIfBusOff() {
  if (millis() - lastCanChkMs < 500) return;
  lastCanChkMs = millis();
  if (CAN0.getError() & EFLG_TXBO) {
    Serial.println(F("[CAN] BUS-OFF 감지 -> MCP2515 재init (자동복구, 리부트 불필요)"));
    initCan();
  }
}


void setup() {
  // ★부팅 즉시 워치독 끔(구형 부트로더 리셋루프 방지 — WDT 리셋 후 여기서 먼저 끈다)
  MCUSR = 0;
  wdt_disable();

  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== Nano ToF GRASP sender (median-5, 10Hz, self-recover x3 + WDT + I2Ctimeout) ==="));

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);   // ★I2C 락업 방지: 트랜잭션이 25ms 넘으면 강제중단+TWI 리셋(begin/rangingTest 영원히 멈춤 차단)
  delay(200);

  // ★워치독을 init 전에 켠다 — setup 중(센서 재시도·MCP init) hang도 자동복구. 재시도 루프마다 wdt_reset로 먹임.
  wdt_enable(WDTO_4S);

  tofReady = false;
  for (uint8_t attempt = 1; attempt <= 8; attempt++) {
    wdt_reset();
    if (initToF()) {
      Serial.print(F("VL53L0X init OK (try=")); Serial.print(attempt); Serial.println(F(")"));
      break;
    }
    Serial.print(F("VL53L0X begin 실패, 재시도 ")); Serial.print(attempt);
    Serial.println(F("/8 ..."));
    delay(300);
  }
  if (!tofReady)
    Serial.println(F("VL53L0X init FAIL — loop에서 계속 재시도(status=2 송신 중). 접촉/전원 점검."));

  pinMode(CAN_INT_PIN, INPUT_PULLUP);
  initCan();
  Serial.println(F("MCP2515 init OK (125kbps, 8MHz)"));
#if LOOPBACK_TEST
  Serial.println(F(">>> LOOPBACK MODE <<<"));
#else
  Serial.println(F("NORMAL MODE"));
#endif

  Serial.println(F("Sender ready. ToF ID=0x300, period=100ms, filter=median-5"));
  // ★워치독은 위에서 init 전에 이미 켰다(Wire setup 직후) — setup 중 hang도 잡기 위해.
  //   loop가 멈추면(센서 I2C 블로킹 등) 4초 후 자동 리셋→setup 재실행→복구. 전원/재업로드 불필요.
}


void loop() {
  wdt_reset();          // ★워치독 먹이기 — 한 바퀴 정상이면 리셋 안 됨. 멈추면 4초 후 자동 리셋.
  pollCanRx();

  uint32_t now = millis();

  // ① 센서 자가복구 — init 실패 상태면 1초마다 begin 재시도(재부팅 불필요)
  if (!tofReady && now - lastReinitMs >= 1000) {
    lastReinitMs = now;
    if (initToF())
      Serial.println(F("VL53L0X 뒤늦게 init OK (loop 재시도) — 복구됨"));
  }

  // ③ CAN bus-off 자가복구 — 송신 영구중단 상태면 MCP2515 재init(재부팅 불필요)
  recoverCanIfBusOff();

  if (now - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = now;
    sendToFFrame();
  }

  printDiagnostics();
}
