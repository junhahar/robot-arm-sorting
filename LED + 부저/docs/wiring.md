# LED + 부저 배선

이 배선은 **별도 표시용 Arduino Nano** 기준입니다. 기존 TOF용 Nano에 이 코드를 올리거나 배선을 섞지 않습니다.

## 기본 핀맵

| 부품 | Arduino Nano 핀 | 비고 |
|---|---|---|
| MCP2515 CS | D10 | SPI CS |
| MCP2515 MOSI | D11 | SPI |
| MCP2515 MISO | D12 | SPI |
| MCP2515 SCK | D13 | SPI |
| MCP2515 INT | D2 | 인터럽트 입력 |
| 빨강 LED | D4 | 직렬 저항 필요 |
| 초록 LED | D5 | 직렬 저항 필요 |
| 부저 | D6 | 능동형 부저 기본 |
| 랙 리셋 버튼 | D7 | `INPUT_PULLUP`, 버튼 반대쪽은 GND |

## LED 직접 연결

일반 5V LED라면 각 LED마다 220~330옴 직렬 저항을 넣습니다.

```text
D4 -> 220~330옴 -> 빨강 LED 애노드
빨강 LED 캐소드 -> GND

D5 -> 220~330옴 -> 초록 LED 애노드
초록 LED 캐소드 -> GND
```

## 부저 연결

작은 5V 능동형 부저이고 소비 전류가 Nano 핀 허용 범위 안이면 직접 구동할 수 있습니다. 전류 사양 확인 필요.

```text
D6 -> 부저 +
부저 - -> GND
```

수동형 피에조 부저이면 Arduino 코드에서 아래를 바꿉니다.

```cpp
#define PASSIVE_BUZZER 1
```

12V/24V 부저, 타워램프, 패널 표시등, 소비 전류가 큰 장치는 Nano 핀에 직접 연결하지 않습니다.

```text
Arduino D6 -> 게이트/베이스 저항 -> MOSFET 또는 트랜지스터
외부 부저 전원 + -> 부저 +
부저 - -> MOSFET drain 또는 트랜지스터 collector
MOSFET source 또는 트랜지스터 emitter -> GND
외부 전원 GND -> Arduino GND와 공통
```

정확한 저항값, 드라이버 방식, 역기전력 보호 필요 여부는 부저/램프 사양 확인 필요.

## 리셋 버튼

```text
Arduino D7 -> 버튼 한쪽
버튼 반대쪽 -> GND
```

코드는 `INPUT_PULLUP` 기준입니다. 평상시 HIGH, 누르면 LOW입니다. 이 버튼은 Nano에서 랙 카운트를 직접 초기화하지 않고 RPi로 `0x411` 리셋 요청만 보냅니다.

## CAN 배선

```text
Raspberry Pi 5 USB -> CANable
CANable CANH -> MCP2515 CANH
CANable CANL -> MCP2515 CANL
CANable GND -> 표시용 Nano/MCP2515 GND 권장
```

- CANable과 표시용 MCP2515가 버스 양 끝이면 양 끝에만 120옴 종단을 둡니다.
- 기존 로봇 제어/TOF CAN 노드가 이미 같은 버스에 있으면 전체 버스 양 끝만 종단합니다. 중간 노드에 종단저항을 추가하면 안 됩니다.
- MCP2515 모듈에 120옴 저항이 이미 납땜되어 있는지 사양 확인 필요.
- CANable TERM 스위치 또는 점퍼 상태 사양 확인 필요.
- CANH/CANL은 꼬임쌍을 권장합니다.

## 전원과 GND

- 표시용 Nano는 PC USB, 별도 5V 어댑터, 또는 검증된 5V DC-DC 전원으로 구동합니다.
- Raspberry Pi 5는 전용 USB-C 5V 어댑터를 사용합니다.
- MCP2515 VCC는 사용하는 모듈의 로직 전압 사양 확인 필요. 일반 Nano 조합은 보통 5V MCP2515 모듈을 사용합니다.
- 표시용 Nano GND, MCP2515 GND, CANable terminal GND는 공통 GND로 묶는 것을 권장합니다.
- STS3215 서보 전원은 Arduino 5V, Raspberry Pi 5V, CANable 5V OUT에서 공급하지 않습니다.

## 안전

- 표시용 Nano와 부저는 안전장치가 아니라 알림장치입니다.
- E-Stop은 STS3215 +7.4~+7.5V 서보 전원을 물리적으로 차단해야 합니다.
- `RACK_FULL` 상태는 RPi 로직에서 새 PIPE 투입과 CNC 완료품 회수를 막아야 합니다.
- 부저가 너무 자주 울리면 작업자가 무시하게 되므로, `RACK_FULL`은 짧은 두 번 삐삐 반복, `ERROR`만 빠른 반복으로 구분합니다.
