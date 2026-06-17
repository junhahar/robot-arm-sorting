/*
 * MPU6050 ×3 티칭 센서 — Arduino Nano
 *
 * 배선:
 *   Nano A4(SDA), A5(SCL) → TCA9548A SDA, SCL
 *   TCA9548A CH0 → MPU#1 (상완)
 *   TCA9548A CH1 → MPU#2 (전완)
 *   TCA9548A CH2 → MPU#3 (손등)
 *   5V, GND → TCA + MPU 3개 공통
 *
 * 출력 (Serial 115200):
 *   yaw1,pitch1,roll1,yaw2,pitch2,roll2,yaw3,pitch3,roll3\n
 *   50Hz 주기
 */

#include <Wire.h>
#include <MPU6050_light.h>

#define TCA_ADDR 0x70
#define NUM_SENSORS 3

MPU6050 mpu(Wire);
float angles[NUM_SENSORS][3];  // [sensor][yaw, pitch, roll]

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool initSensor(uint8_t channel) {
    tcaSelect(channel);
    byte status = mpu.begin();
    if (status != 0) {
        Serial.print("[ERR] MPU CH");
        Serial.print(channel);
        Serial.print(" fail: ");
        Serial.println(status);
        return false;
    }
    Serial.print("[OK] MPU CH");
    Serial.println(channel);
    return true;
}

void calibrateSensor(uint8_t channel) {
    tcaSelect(channel);
    Serial.print("[CAL] CH");
    Serial.print(channel);
    Serial.println("...");
    mpu.calcOffsets();
}

void readSensor(uint8_t channel, uint8_t idx) {
    tcaSelect(channel);
    mpu.update();
    angles[idx][0] = mpu.getAngleZ();  // yaw
    angles[idx][1] = mpu.getAngleY();  // pitch
    angles[idx][2] = mpu.getAngleX();  // roll
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Serial.println("[NANO] MPU Teaching Sensor");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (!initSensor(i)) {
            Serial.println("[ERR] Init failed — check wiring");
            while (1) delay(1000);
        }
    }

    Serial.println("[CAL] Hold arm still for 3 seconds...");
    delay(1000);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        calibrateSensor(i);
    }
    Serial.println("[RDY] Streaming...");
}

void loop() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        readSensor(i, i);
    }

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(angles[i][0], 2);
        Serial.print(",");
        Serial.print(angles[i][1], 2);
        Serial.print(",");
        Serial.print(angles[i][2], 2);
        if (i < NUM_SENSORS - 1) Serial.print(",");
    }
    Serial.println();

    delay(20);  // 50Hz
}
