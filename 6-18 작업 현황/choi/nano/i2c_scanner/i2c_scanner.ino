// ============================================================
// I2C 스캐너 - VL53L0X(TOF200C) 배선/전원 확진용
//
//   VL53L0X(TOF200C) ── I2C ── Arduino Nano
//     VIN → 5V,  GND → GND,  SDA → A4,  SCL → A5
//
// 업로드 후 시리얼 모니터(115200)에서:
//   "found 0x29"  → 센서 인식됨. 배선/전원 정상 (VL53L0X 기본주소 0x29)
//   "no device"   → 배선/전원/풀업 문제. SDA·SCL 스왑, VIN 5V, GND 공통 확인
//
// ※ 이 스케치엔 MCP2515/CAN 코드 없음. 순수 I2C 확인 전용.
//   확인 끝나면 다시 nano_tof_can_sender.ino 업로드.
// ============================================================

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();
  Serial.println(F("=== I2C Scanner ==="));
  Serial.println(F("VL53L0X(TOF200C) 기대 주소: 0x29"));
}

void loop() {
  byte count = 0;

  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("found 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x29) Serial.print(F("  <- VL53L0X!"));
      Serial.println();
      count++;
    }
  }

  if (count == 0) {
    Serial.println(F("no device  <- 배선/전원/풀업 확인 (SDA=A4, SCL=A5, VIN=5V, GND공통)"));
  }
  Serial.println(F("---"));
  delay(2000);
}
