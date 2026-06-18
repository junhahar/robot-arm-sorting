# LED + 부저 배선

이 배선은 **표시 전용 Arduino Nano** 기준입니다. 기존 TOF용 Nano나 로봇팔 제어 Nano의 배선을 바꾸지 않습니다.

## 기본 핀맵

| 부품 | Arduino Nano 핀 | 비고 |
|---|---:|---|
| MCP2515 INT | D2 | CAN 수신 인터럽트 입력 |
| 빨간 LED | D4 | CNC 점유/가공 중 |
| 초록 LED | D5 | 가공 완료/회수 대기 |
| 압전 부저 | D6 | 적재 완료/만재/에러 |
| 파란 LED | D7 | CNC 비어 있음 |
| MCP2515 CS | D10 | SPI CS |
| MCP2515 MOSI | D11 | SPI MOSI |
| MCP2515 MISO | D12 | SPI MISO |
| MCP2515 SCK | D13 | SPI SCK |
| MCP2515 VCC | 5V | 사용하는 MCP2515 모듈 전압 사양 확인 필요 |
| MCP2515 GND | GND | Nano GND와 공통 |

## LED 연결

일반 5V LED 기준으로 각 LED마다 220~330옴 저항을 넣습니다.

```text
D4 -> 220~330옴 -> 빨간 LED +
빨간 LED - -> GND

D5 -> 220~330옴 -> 초록 LED +
초록 LED - -> GND

D7 -> 220~330옴 -> 파란 LED +
파란 LED - -> GND
```

LED는 CNC 박스에 5mm LED 홀더 또는 베젤로 고정하는 것을 권장합니다. LED 다리에 점퍼선을 임시로 감는 방식은 테스트용으로만 사용하고, 최종은 납땜 또는 단자 처리 후 수축튜브로 절연합니다.

## 부저 연결

작은 5V 압전 부저 기준입니다.

```text
D6 -> 부저 +
부저 - -> GND
```

수동 압전 부저면 Arduino 코드의 기본값을 그대로 사용합니다.

```cpp
#define PASSIVE_BUZZER 1
```

능동 부저면 아래처럼 바꿉니다.

```cpp
#define PASSIVE_BUZZER 0
```

12V/24V 부저, 큰 전류 부저, 산업용 경광 부저는 Nano 핀에 직접 연결하지 않습니다. MOSFET 또는 트랜지스터 드라이버를 사용해야 합니다. 부저 사양 확인 필요.

## MCP2515 연결

```text
Arduino Nano D2  -> MCP2515 INT
Arduino Nano D10 -> MCP2515 CS
Arduino Nano D11 -> MCP2515 SI / MOSI
Arduino Nano D12 -> MCP2515 SO / MISO
Arduino Nano D13 -> MCP2515 SCK
Arduino Nano 5V  -> MCP2515 VCC
Arduino Nano GND -> MCP2515 GND
```

MCP2515 모듈 크리스털을 확인합니다.

```text
8MHz  -> Arduino 코드의 MCP_CLOCK = MCP_8MHZ 그대로 사용
16MHz -> MCP_CLOCK을 MCP_16MHZ로 변경
```

## CAN 배선

```text
Raspberry Pi 5 USB -> CANable
CANable CANH -> MCP2515 CANH
CANable CANL -> MCP2515 CANL
CANable GND -> 표시용 Nano/MCP2515 GND 권장
```

- CANable과 MCP2515가 버스 양 끝이면 양 끝에만 120옴 종단을 둡니다.
- 기존 로봇 제어/TOF CAN 노드가 같은 버스에 있으면 전체 버스 양 끝만 종단합니다.
- 중간 노드에 종단저항을 추가하면 안 됩니다.
- CANH/CANL은 트위스트 페어를 권장합니다.
- MCP2515 모듈 내장 120옴 저항 여부와 CANable TERM 상태는 사양 확인 필요.

## 전원과 GND

```text
표시용 Nano = PC USB 또는 별도 5V 전원
Raspberry Pi 5 = 전용 USB-C 5V 어댑터
STS3215 서보 = 별도 7.4~7.5V SMPS
```

- 표시용 Nano GND, MCP2515 GND, CANable terminal GND는 공통 GND로 묶는 것을 권장합니다.
- STS3215 서보 전원은 Arduino 5V, Raspberry Pi 5V, CANable 5V OUT에서 공급하지 않습니다.
- 브레드보드는 표시용 Nano/MCP2515/저항/부저 같은 저전류 회로에만 사용합니다.
- 서보 전원 같은 고전류 전원은 브레드보드로 분배하지 않습니다.

## 안전

- 표시용 Nano는 안전장치가 아니라 알림장치입니다.
- E-Stop은 반드시 STS3215 +7.4~+7.5V 서보 전원을 물리적으로 차단해야 합니다.
- `RACK_FULL` 상태에서는 RPi 로직에서 다음 CNC 투입을 막아야 합니다.
- 적재함을 비운 뒤 `rack_count = 0` 초기화는 RPi 대시보드나 운영 스크립트에서 처리합니다.
