/*
 * Nucleo-F103RB 펌웨어 — CAN <-> STS3215 브릿지
 *
 * 통신 경로:
 *   RPi5 -> USB -> CANable Pro -> CAN -> STM32 (내장 bxCAN) -> UART -> STS3215
 *
 * 배선:
 *   CAN:        PA11(RX), PA12(TX) + 외부 트랜시버(MCP2551/TJA1050)
 *   STS3215:    Serial1 PA9(TX), PA10(RX) -> Bus Servo Driver
 *   SG90:       PWM PA8
 *   VL53L0X:    I2C PB6(SCL), PB7(SDA)
 *   E-STOP:     PB4 (INPUT_PULLUP)
 *   Debug:      Serial (USART2, ST-Link USB) 115200bps
 *
 * Arduino Uno 대비 변경:
 *   - MCP2515/SPI 제거 -> 내장 CAN (eXoCAN)
 *   - FSR 제거
 *   - Serial/Serial1 분리 -> 업로드 시 서보 드라이버 분리 불필요
 *
 * 필요 라이브러리:
 *   - eXoCAN (exomodular)
 *   - SCServo (Feetech) — lib/ 폴더에 수동 추가
 *   - Servo (Arduino 기본)
 *   - Wire (Arduino 기본)
 */

#include <Arduino.h>
#include <eXoCAN.h>
#include <SCServo.h>
#include <Servo.h>
#include <Wire.h>

// ─── Pin ────────────────────────────────────────────────────
#define GRIPPER_PIN  PA8
#define ESTOP_PIN    PB4

// ─── STS3215 ────────────────────────────────────────────────
#define SERVO_BAUD   1000000

// ─── CAN Message ID (config.py와 동일) ──────────────────────
#define CAN_ID_SERVO_MOVE  0x10
#define CAN_ID_SERVO_READ  0x11
#define CAN_ID_GRIPPER     0x12
#define CAN_ID_TOF_READ    0x13
#define CAN_ID_ESTOP       0xFF

#define CAN_ID_SERVO_POS   0x20
#define CAN_ID_TOF_DIST    0x21

// ─── 객체 ───────────────────────────────────────────────────
eXoCAN can(STD_ID_LEN, BR125K, PORTA_11_12_XCVR);
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

// ─── CAN 송신 ───────────────────────────────────────────────
void canSend(uint32_t id, uint8_t *data, uint8_t len) {
    can.transmit(id, data, len);
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

void handleEStop() {
    estop_active = true;
    for (int id = 1; id <= 6; id++) {
        sms_sts.EnableTorque(id, 0);
    }
    gripper.write(90);
    Serial.println("[E-STOP] ACTIVE");
}

// ─── Setup ──────────────────────────────────────────────────

void setup() {
    // Debug (ST-Link USB)
    Serial.begin(115200);
    Serial.println("[STM32] Boot...");

    // E-STOP
    pinMode(ESTOP_PIN, INPUT_PULLUP);

    // STS3215 — Serial1 (USART1, PA9/PA10)
    Serial1.begin(SERVO_BAUD);
    sms_sts.pSerial = &Serial1;

    // Gripper
    gripper.attach(GRIPPER_PIN);
    gripper.write(90);

    // CAN (내장 bxCAN, PA11/PA12)
    can.filterMask16Init(0, 0, 0x7FF, 0, 0);

    // ToF (I2C1, PB6/PB7)
    Wire.begin();
    Wire.beginTransmission(TOF_ADDR);
    tofReady = (Wire.endTransmission() == 0);

    Serial.print("[STM32] ToF: ");
    Serial.println(tofReady ? "OK" : "NOT FOUND");
    Serial.println("[STM32] Ready");
}

// ─── Main Loop ──────────────────────────────────────────────

void loop() {
    // E-STOP
    if (digitalRead(ESTOP_PIN) == LOW && !estop_active) {
        handleEStop();
        uint8_t data[1] = {0xFF};
        canSend(CAN_ID_SERVO_POS, data, 1);
    }

    if (estop_active) {
        delay(100);
        return;
    }

    // CAN 수신 (polling)
    int rxId;
    int fltIdx;
    uint8_t buf[8];
    int len = can.receive(rxId, fltIdx, buf);

    if (len < 0) return;

    switch ((uint32_t)rxId) {
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
            break;
    }
}
