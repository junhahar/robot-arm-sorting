/*
 * Arduino Uno 펌웨어 — CAN ↔ STS3215 브릿지
 *
 * 역할: RPi5에서 CAN으로 받은 명령을 STS3215 서보에 UART로 전달
 *       서보 위치 피드백, ToF 거리, 그리퍼(SG90) 제어
 *
 * 배선:
 *   MCP2515  → SPI (10=CS, 11=MOSI, 12=MISO, 13=SCK)
 *   STS3215  → SoftwareSerial (RX=2, TX=3) + DIR=4
 *   SG90     → PWM pin 9
 *   VL53L0X  → I2C (A4=SDA, A5=SCL)
 *
 * 필요 라이브러리:
 *   - mcp_can (Seeed Studio 또는 coryjfowler)
 *   - SCServo (Feetech)
 *   - Servo (Arduino 기본)
 *   - VL53L0X (Pololu) — ToF 사용 시
 *
 * STS3215 보드레이트:
 *   기본 1Mbps지만 SoftwareSerial은 최대 115200.
 *   → 서보 보드레이트를 115200으로 변경 후 사용 (아래 SERVO_BAUD)
 *   → 변경 방법: Feetech 매니저 소프트웨어 또는 별도 스크립트
 */

#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>
#include <SCServo.h>
#include <Servo.h>
#include <Wire.h>

// ─── Pin 정의 ───────────────────────────────────────────────
#define CAN_CS_PIN   10
#define STS_RX_PIN   2
#define STS_TX_PIN   3
#define STS_DIR_PIN  4   // half-duplex 방향 제어
#define GRIPPER_PIN  9

// ─── 설정 ───────────────────────────────────────────────────
#define SERVO_BAUD   115200  // STS3215 보드레이트 (기본 1M → 115200 변경 필요)
#define CAN_SPEED    CAN_500KBPS
#define MCP_CLOCK    MCP_8MHZ  // MCP2515 크리스탈 (8MHz or 16MHz 확인)

// ─── CAN Message ID (config.py와 동일) ──────────────────────
#define CAN_ID_SERVO_MOVE  0x10
#define CAN_ID_SERVO_READ  0x11
#define CAN_ID_GRIPPER     0x12
#define CAN_ID_TOF_READ    0x13
#define CAN_ID_ESTOP       0xFF

#define CAN_ID_SERVO_POS   0x20  // 응답
#define CAN_ID_TOF_DIST    0x21  // 응답

// ─── 객체 생성 ──────────────────────────────────────────────
MCP_CAN CAN(CAN_CS_PIN);
SoftwareSerial stsSerial(STS_RX_PIN, STS_TX_PIN);
SMS_STS sms_sts;
Servo gripper;

// ─── ToF (VL53L0X) ─────────────────────────────────────────
#define TOF_ADDR  0x29
bool tofReady = false;

uint16_t readToF() {
    if (!tofReady) return 9999;

    // VL53L0X 단일 측정 요청
    Wire.beginTransmission(TOF_ADDR);
    Wire.write(0x00);
    Wire.write(0x01);  // 측정 시작
    Wire.endTransmission();

    delay(30);  // 측정 대기

    // 결과 읽기 (레지스터 0x1E-0x1F)
    Wire.beginTransmission(TOF_ADDR);
    Wire.write(0x1E);
    Wire.endTransmission();
    Wire.requestFrom(TOF_ADDR, (uint8_t)2);

    if (Wire.available() >= 2) {
        uint16_t dist = Wire.read() << 8;
        dist |= Wire.read();
        return dist;
    }
    return 9999;
}

// ─── Half-duplex 방향 제어 ──────────────────────────────────
void setTX() {
    digitalWrite(STS_DIR_PIN, HIGH);
    delayMicroseconds(10);
}

void setRX() {
    delayMicroseconds(10);
    digitalWrite(STS_DIR_PIN, LOW);
}

// ─── CAN 응답 전송 ─────────────────────────────────────────
void canSend(uint32_t id, uint8_t *data, uint8_t len) {
    CAN.sendMsgBuf(id, 0, len, data);
}

// ─── 명령 처리 ─────────────────────────────────────────────

void handleServoMove(uint8_t *buf) {
    uint8_t servoId = buf[0];
    uint16_t pos = ((uint16_t)buf[1] << 8) | buf[2];

    setTX();
    sms_sts.WritePosEx(servoId, (int)pos, 0, 0);
    setRX();
}

void handleServoRead(uint8_t *buf) {
    uint8_t servoId = buf[0];

    setRX();
    int pos = sms_sts.ReadPos(servoId);
    if (pos < 0) pos = 0;

    uint8_t resp[3];
    resp[0] = servoId;
    resp[1] = (uint8_t)(pos >> 8);
    resp[2] = (uint8_t)(pos & 0xFF);
    canSend(CAN_ID_SERVO_POS, resp, 3);
}

void handleGripper(uint8_t *buf) {
    uint8_t angle = buf[0];
    gripper.write(angle);
}

void handleToFRead() {
    uint16_t dist = readToF();
    uint8_t resp[2];
    resp[0] = (uint8_t)(dist >> 8);
    resp[1] = (uint8_t)(dist & 0xFF);
    canSend(CAN_ID_TOF_DIST, resp, 2);
}

void handleEStop() {
    // 모든 서보 토크 해제
    setTX();
    for (int id = 1; id <= 5; id++) {
        sms_sts.EnableTorque(id, 0);
    }
    setRX();
    gripper.write(90);  // 그리퍼 열기
}

// ─── Setup ──────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("[FW] Arduino CAN-STS3215 Bridge");

    // Direction pin
    pinMode(STS_DIR_PIN, OUTPUT);
    digitalWrite(STS_DIR_PIN, LOW);  // 기본 RX 모드

    // STS3215 시리얼
    stsSerial.begin(SERVO_BAUD);
    sms_sts.pSerial = &stsSerial;
    Serial.println("[FW] STS3215 serial OK");

    // 그리퍼
    gripper.attach(GRIPPER_PIN);
    gripper.write(90);
    Serial.println("[FW] Gripper OK");

    // CAN 초기화
    if (CAN.begin(MCP_ANY, CAN_SPEED, MCP_CLOCK) == CAN_OK) {
        Serial.println("[FW] CAN OK");
    } else {
        Serial.println("[FW] CAN FAIL");
    }
    CAN.setMode(MCP_NORMAL);

    // ToF 초기화
    Wire.begin();
    Wire.beginTransmission(TOF_ADDR);
    if (Wire.endTransmission() == 0) {
        tofReady = true;
        Serial.println("[FW] ToF OK");
    } else {
        Serial.println("[FW] ToF not found");
    }

    Serial.println("[FW] Ready");
}

// ─── Main Loop ──────────────────────────────────────────────

void loop() {
    // CAN 메시지 수신 대기
    if (CAN.checkReceive() != CAN_MSGAVAIL) return;

    long unsigned int canId;
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN.readMsgBuf(&canId, &len, buf) != CAN_OK) return;

    switch (canId) {
        case CAN_ID_SERVO_MOVE:
            if (len >= 3) handleServoMove(buf);
            break;
        case CAN_ID_SERVO_READ:
            if (len >= 1) handleServoRead(buf);
            break;
        case CAN_ID_GRIPPER:
            if (len >= 1) handleGripper(buf);
            break;
        case CAN_ID_TOF_READ:
            handleToFRead();
            break;
        case CAN_ID_ESTOP:
            handleEStop();
            Serial.println("[FW] E-STOP!");
            break;
    }
}
