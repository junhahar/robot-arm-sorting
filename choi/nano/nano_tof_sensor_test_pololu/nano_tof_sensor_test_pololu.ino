// ============================================================
// VL53L0X 센서 단독 테스트 — Pololu 라이브러리 버전 (CAN 없음).
//   ★ Adafruit begin()이 "0x29는 보이는데 계속 실패"할 때(TOF200C 클론) 사용.
//   ★ Pololu 라이브러리는 init이 더 관대해서 Adafruit이 거부하는 클론도 잡는다.
//
// [라이브러리 설치] Arduino IDE → 라이브러리 매니저 → "VL53L0X" 검색
//   → 저자 **Pololu** 것 설치. (Adafruit_VL53L0X 아님! 다른 라이브러리)
//
// [배선] VIN=5V, GND=GND, SDA=A4, SCL=A5
//
// 시리얼 115200:
//   - "init OK" 후 "거리 = NNN mm" 줄줄  → 센서 정상! (Adafruit만 문제였던 것)
//                                          → 운전용 펌웨어도 Pololu로 바꿔야 함
//   - "init FAIL (Pololu도 실패)"         → 센서 사망 가능성 큼 → 교체
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>     // ★ Pololu 라이브러리 (Adafruit 아님)

VL53L0X sensor;
bool ready = false;


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("=== VL53L0X 센서 테스트 (Pololu 라이브러리) ==="));

  Wire.begin();
  Wire.setClock(100000);   // 표준 100kHz (클론 안정)
  delay(200);

  sensor.setTimeout(500);

  // init 최대 10회 재시도
  for (uint8_t i = 1; i <= 10; i++) {
    if (sensor.init()) {
      ready = true;
      Serial.print(F("init OK (try="));
      Serial.print(i); Serial.println(F(")"));
      break;
    }
    Serial.print(F("init 실패, 재시도 ")); Serial.print(i); Serial.println(F("/10"));
    delay(300);
  }

  if (ready) {
    sensor.startContinuous();    // 연속 측정 모드
    Serial.println(F("연속측정 시작 — 손으로 가렸다 떼며 숫자 변하는지 보세요"));
  } else {
    Serial.println(F("init FAIL (Pololu도 실패)."));
    Serial.println(F("  배선은 OK(0x29 응답)인데 Adafruit·Pololu 둘 다 init 실패 → 센서 칩 사망 가능성 큼 → 교체 권장."));
  }
}


void loop() {
  if (!ready) {
    delay(1000);
    Serial.println(F("...센서 init 안 됨 (대기)"));
    return;
  }

  uint16_t mm = sensor.readRangeContinuousMillimeters();

  Serial.print(F("거리 = "));
  Serial.print(mm);
  Serial.print(F(" mm"));
  if (sensor.timeoutOccurred())      Serial.print(F("   TIMEOUT(통신끊김)"));
  else if (mm >= 8000)               Serial.print(F("   (범위초과/물체없음)"));
  else if (mm <= 130)                Serial.print(F("   (<=130: 잡음 영역)"));
  else                               Serial.print(F("   (>130: 못잡음 영역)"));
  Serial.println();

  delay(200);   // 5Hz
}
