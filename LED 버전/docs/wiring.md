# LED 버전 배선

## 권장 핀

| 부품 | Arduino Nano 핀 | 비고 |
|---|---|---|
| MCP2515 CS | D10 | SPI CS |
| MCP2515 MOSI | D11 | SPI |
| MCP2515 MISO | D12 | SPI |
| MCP2515 SCK | D13 | SPI |
| MCP2515 INT | D2 | 인터럽트 입력 |
| 빨강 LED | A0 | 기본값, 디지털 출력으로 사용 |
| 초록 LED | A1 | 기본값, 디지털 출력으로 사용 |

첨부 원안의 D4/D5도 사용할 수 있습니다. 하지만 D4~D9는 리미트 스위치 확장 후보 핀이므로 최종 하드웨어에서는 A0/A1을 권장합니다.

```text
USE_D4_D5_LED_PINS = 0:
  A0 = 빨강 LED
  A1 = 초록 LED

USE_D4_D5_LED_PINS = 1:
  D4 = 빨강 LED
  D5 = 초록 LED
```

## 일반 LED 직접 연결

5V용 일반 LED라면 각 LED마다 220~330옴 직렬 저항을 넣습니다.

```text
Arduino A0/D4 -> 220~330옴 -> 빨강 LED 애노드
빨강 LED 캐소드 -> GND

Arduino A1/D5 -> 220~330옴 -> 초록 LED 애노드
초록 LED 캐소드 -> GND
```

코드는 active HIGH 기준입니다.

## 12V/24V 패널 LED 또는 표시등

12V/24V 패널 LED, 타워램프, 소비 전류가 큰 표시등은 Arduino 핀에 직접 연결하지 않습니다.

```text
Arduino 출력 -> 게이트/베이스 저항 -> MOSFET 또는 트랜지스터
외부 LED 전원 + -> 표시등 +
표시등 - -> MOSFET drain 또는 트랜지스터 collector
MOSFET source 또는 트랜지스터 emitter -> GND
외부 전원 GND -> Arduino GND와 공통
```

정확한 저항값, 구동 전류, 절연 방식은 실제 표시등 사양 확인 필요.

## CAN 배선

```text
Raspberry Pi 5 USB -> CANable
CANable CANH -> MCP2515 CANH
CANable CANL -> MCP2515 CANL
CANable GND -> MCP2515/Arduino GND 권장
```

- CANable과 MCP2515가 버스 양 끝이면 양 끝에만 120옴 종단을 둡니다.
- MCP2515 모듈에 120옴 저항이 이미 납땜되어 있는지 사양 확인 필요.
- CANable TERM 스위치 또는 점퍼 상태 사양 확인 필요.
- CANH/CANL은 꼬임쌍을 권장합니다.

## 전원과 GND

- Raspberry Pi 5는 전용 USB-C 5V 어댑터를 사용합니다.
- Arduino Nano는 PC USB 또는 별도 5V 전원을 사용할 수 있습니다.
- MCP2515 VCC는 사용하는 모듈의 로직 전압 사양 확인 필요. 일반 Nano 조합은 보통 5V 모듈을 사용합니다.
- SMPS GND, Arduino GND, MCP2515 GND, URT GND, STS3215 GND, VL53L1X GND, CANable terminal GND는 공통 GND로 묶는 것을 권장합니다.

## 안전

- E-Stop은 STS3215 +7.4~+7.5V 서보 전원을 물리적으로 차단해야 합니다.
- LED 상태 표시는 안전장치가 아니라 작업자 알림입니다.
- `RACK_FULL`은 LED만 켜는 것으로 끝내면 안 됩니다. Raspberry Pi 로직에서 새 CNC 투입과 회수 동작을 막아야 합니다.
- STS3215 서보 전원은 Arduino 5V, Raspberry Pi 5V, CANable 5V OUT에서 공급하지 않습니다.
