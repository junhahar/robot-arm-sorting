/*
 * Arduino Uno 펌웨어 — CAN ↔ STS3215 브릿지
 *
 * 통신 경로:
 *   RPi5 → USB → CANable Pro → CAN → MCP2515 → SPI → Arduino Uno
 *   Arduino Uno → Hardware UART (D0/D1) → Bus Servo Driver → STS3215 x6
 *
 * 배선:
 *   MCP2515  → SPI (10=CS, 11=MOSI, 12=MISO, 13=SCK)
 *   Bus Servo Driver → Hardware Serial (D0=RX, D1=TX)
 *   SG90     → PWM pin 9
 *   VL53L0X  → I2C (A4=SDA, A5=SCL)
 *   E-STOP   → pin 2 (INPUT_PULLUP)
 *
 * 주의:
 *   D0/D1이 USB와 공유 → 펌웨어 업로드 시 Bus Servo Driver 분리
 *
 * 필요 라이브러리:
 *   - mcp_can (Seeed Studio 또는 coryjfowler)
 *   - SCServo (Feetech)
 *   - Servo (Arduino 기본)
 *   - VL53L0X (Pololu) — ToF 사용 시
 */

#include <SPI.h>
#include <mcp_can.h>
#include <SCServo.h>
#include <Servo.h>
#include <Wire.h>

// ─── Pin 정의 ───────────────────────────────────────────────
#define CAN_CS_PIN   10
#define GRIPPER_PIN  9
#define ESTOP_PIN    2
#define FSR_PIN      A0

// ─── 설정 ───────────────────────────────────────────────────
#define SERVO_BAUD   1000000  // STS3215 기본 1Mbps (Hardware Serial)
#define CAN_SPEED    CAN_500KBPS
#define MCP_CLOCK    MCP_8MHZ

// ─── CAN Message ID (config.py와 동일) ──────────────────────
#define CAN_ID_SERVO_MOVE  0x10
#define CAN_ID_SERVO_READ  0x11
#define CAN_ID_GRIPPER     0x12
#define CAN_ID_TOF_READ    0x13
#define CAN_ID_FSR_READ    0x14
#define CAN_ID_ESTOP       0xFF

#define CAN_ID_SERVO_POS   0x20
#define CAN_ID_TOF_DIST    0x21
#define CAN_ID_FSR_VAL     0x22

// ─── 객체 생성 ──────────────────────────────────────────────
MCP_CAN CAN(CAN_CS_PIN);
SMS_STS sms_sts;
Servo gripper;

bool estop_active = false;

// ─── ToF (VL53L0X) ─────────────────────────────────────────
#define TOF_ADDR  0x29
bool tofReady = false;

uint16_t readToF() {
    if (!tofReady) return 9999;

    Wire.beginTransmission(TOF_ADDR);
    Wire.write(0x00);
    Wire.write(0x01);
    Wire.endTransmission();

    delay(30);

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

// ─── CAN 응답 ───────────────────────────────────────────────
void canSend(uint32_t id, uint8_t *data, uint8_t len) {
    CAN.sendMsgBuf(id, 0, len, data);
}

// ─── 명령 처리 ──────────────────────────────────────────────

void handleServoMove(uint8_t *buf) {
    uint8_t servoId = buf[0];
    uint16_t pos = ((uint16_t)buf[1] << 8) | buf[2];
    sms_sts.WritePosEx(servoId, (int)pos, 0, 0);
}

void handleServoRead(uint8_t *buf) {
    uint8_t servoId = buf[0];
    int pos = sms_sts.ReadPos(servoId);
    if (pos < 0) pos = 0;

    uint8_t resp[3];
    resp[0] = servoId;
    resp[1] = (uint8_t)(pos >> 8);
    resp[2] = (uint8_t)(pos & 0xFF);
    canSend(CAN_ID_SERVO_POS, resp, 3);
}

void handleGripper(uint8_t *buf) {
    gripper.write(buf[0]);
}

void handleToFRead() {
    uint16_t dist = readToF();
    uint8_t resp[2];
    resp[0] = (uint8_t)(dist >> 8);
    resp[1] = (uint8_t)(dist & 0xFF);
    canSend(CAN_ID_TOF_DIST, resp, 2);
}

void handleFSRRead() {
    uint16_t val = analogRead(FSR_PIN);
    uint8_t resp[2];
    resp[0] = (uint8_t)(val >> 8);
    resp[1] = (uint8_t)(val & 0xFF);
    canSend(CAN_ID_FSR_VAL, resp, 2);
}

void handleEStop() {
    estop_active = true;
    for (int id = 1; id <= 5; id++) {
        sms_sts.EnableTorque(id, 0);
    }
    gripper.write(90);
}

// ─── Setup ──────────────────────────────────────────────────

void setup() {
    // E-STOP 버튼
    pinMode(ESTOP_PIN, INPUT_PULLUP);

    // STS3215 — Hardware Serial (D0/D1) via Bus Servo Driver
    Serial.begin(SERVO_BAUD);
    sms_sts.pSerial = &Serial;

    // 그리퍼
    gripper.attach(GRIPPER_PIN);
    gripper.write(90);

    // CAN 초기화
    while (CAN.begin(MCP_ANY, CAN_SPEED, MCP_CLOCK) != CAN_OK) {
        delay(100);
    }
    CAN.setMode(MCP_NORMAL);

    // ToF 초기화
    Wire.begin();
    Wire.beginTransmission(TOF_ADDR);
    tofReady = (Wire.endTransmission() == 0);
}

// ─── Main Loop ──────────────────────────────────────────────

void loop() {
    // 물리 E-STOP 체크
    if (digitalRead(ESTOP_PIN) == LOW && !estop_active) {
        handleEStop();
        uint8_t data[1] = {0xFF};
        canSend(CAN_ID_SERVO_POS, data, 1);
    }

    if (estop_active) {
        delay(100);
        return;
    }

    // CAN 메시지 수신
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
        case CAN_ID_FSR_READ:
            handleFSRRead();
            break;
        case CAN_ID_ESTOP:
            handleEStop();
            break;
    }
}
