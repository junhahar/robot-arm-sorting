// ============================================================
// VL53L0X 센서 단독 테스트 — CAN/MCP2515 전부 제외.
//   센서만 읽어서 시리얼 모니터로 거리(mm) 출력.
//   "센서가 살아있나?"만 확인하는 용도. (CAN 배선 문제와 분리)
//
// [배선] VIN=5V, GND=GND, SDA=A4, SCL=A5  (XSHUT 미사용)
// [라이브러리] Adafruit "Adafruit_VL53L0X" (의존성 Adafruit BusIO 자동)
//
// 시리얼 모니터 115200:
//   - "found 0x29 <- VL53L0X!"  → 센서 전기적으로 연결됨(I2C OK)
//   - "no device"               → 배선/전원 문제 (SDA/SCL/VIN/GND)
//   - "init OK" 후 "거리 = NNN mm" 줄줄  → 센서 정상 작동!
//   - "init FAIL" 반복           → 0x29 보이면 콜드부팅, 안 보이면 배선
// ============================================================

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X tof;
bool ready = false;


// I2C 버스 스캔 — 0x29(VL53L0X)가 전기적으로 보이는지 확인
void i2cScan() {
  Serial.println(F("[I2C 스캔]"));
  byte found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found 0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if (a == 0x29) Serial.print(F("  <- VL53L0X!"));
      Serial.println();
      found++;
    }
  }
  if (!found)
    Serial.println(F("  no device <- 배선/전원 문제 (SDA=A4, SCL=A5, VIN=5V, GND 공통 확인)"));
}


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== VL53L0X 센서 단독 테스트 (CAN 없음) ==="));

  Wire.begin();
  Wire.setClock(100000);   // 표준 100kHz (클론 안정)
  delay(200);

  i2cScan();               // 먼저 0x29 보이는지

  // begin() 최대 10회 재시도 (클론은 첫 begin이 자주 실패)
  for (uint8_t i = 1; i <= 10; i++) {
    if (tof.begin()) {
      ready = true;
      Serial.print(F("VL53L0X init OK (try="));
      Serial.print(i); Serial.println(F(")"));
      break;
    }
    Serial.print(F("begin 실패, 재시도 ")); Serial.print(i); Serial.println(F("/10"));
    delay(300);
  }

  if (!ready) {
    Serial.println(F("init FAIL. 위 I2C 스캔 결과로 판단:"));
    Serial.println(F("  0x29 보임  → 센서 살아있음, begin만 실패 → USB 완전히 뽑고 10초 후 재연결(콜드부팅)"));
    Serial.println(F("  no device → 배선/전원 문제 또는 센서 사망"));
  }
}


void loop() {
  // 아직 init 안 됐으면 1초마다 재시도(콜드부팅 등으로 살아나면 잡음)
  if (!ready) {
    delay(1000);
    if (tof.begin()) {
      ready = true;
      Serial.println(F("VL53L0X 뒤늦게 init OK!"));
    } else {
      Serial.println(F("...센서 대기 중 (아직 init 안 됨)"));
    }
    return;
  }

  // 측정해서 거리 출력
  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);   // 단발 측정(블로킹 ~30ms)

  if (m.RangeStatus != 4) {     // 4 = 범위초과(phase fail)
    Serial.print(F("거리 = "));
    Serial.print(m.RangeMilliMeter);
    Serial.print(F(" mm"));
    if (m.RangeMilliMeter <= 130) Serial.print(F("   (<=130: 잡음 영역)"));
    else                          Serial.print(F("   (>130: 못잡음 영역)"));
    Serial.println();
  } else {
    Serial.println(F("범위초과 (물체 없음 / 너무 멀음)"));
  }

  delay(200);   // 5Hz, 눈으로 보기 편하게
}
