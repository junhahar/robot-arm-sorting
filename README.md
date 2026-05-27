# Robot Arm Sorting

## 하드웨어 구성

### 메인 컴퓨팅

| 장치 | 역할 | 전원 |
|------|------|------|
| 라즈베리파이5 + Hailo 8L | AI 추론(팔 인식, 볼트/너트 분류), 위치 추정, 메인 제어 | 공식 27W USB-C PD 충전기 |
| 노트북 | 학습 데이터 수집, 모델 학습, 영상 촬영, 디버깅 | 자체 |

### MCU

| 장치 | 역할 | 전원 |
|------|------|------|
| Arduino Uno | ToF·SG90·E-stop·서보 명령 중계 | 노트북 USB |

### 통신 백본 (CAN)

```
RPi5 ──USB── CANable Pro ══CAN══ MCP2515 ──SPI── Arduino Uno
```

- RPi5 ↔ CANable Pro: USB
- CANable Pro ↔ MCP2515: CAN-H / CAN-L 2선
- MCP2515 ↔ Arduino: SPI (D10 CS, D11 MOSI, D12 MISO, D13 SCK, D2 INT)
- 종단저항 120Ω: CANable Pro 및 MCP2515 모듈 내장 점퍼캡으로 활성화

### 카메라

| 카메라 | 연결 | 용도 |
|--------|------|------|
| ArduCam USB 글로벌 셔터 | RPi5 USB3 | 그리퍼 부착, 실시간 볼트/너트 인식 |
| 웹캠 1 | 노트북 USB | 영상 촬영 |
| 웹캠 2 | 노트북 USB | 영상 촬영 |

### 센서 / 액추에이터 (Arduino Uno 연결)

| 장치 | 인터페이스 | 핀 |
|------|-----------|-----|
| VL53L0X (ToF) | I2C | A4 SDA, A5 SCL |
| E-stop 버튼 | GPIO 인터럽트 | D3 (INT1) |
| SG90 그리퍼 서보 | PWM | D9 |
| Bus Servo Adapter | UART | D0 RX, D1 TX |

### 서보 제어 라인

```
Arduino ──UART── Bus Servo Adapter ──── Servo #1 ──┬── #2 ──┬── #3 ──┬── #4 ──┬── #5
   D1(TX) → RXD                       (베이스)    │       │       │       │   (그리퍼축)
   D0(RX) ← TXD                                  (어깨) (팔꿈치) (손목)
   GND   ↔ GND
```

- **서보**: Feetech STS3215 × 5축
- **연결 방식**: Bus Servo Adapter의 서보 포트 #1에 첫 서보 연결 후 데이지체인으로 5축 확장
- **방향 전환**: Bus Servo Adapter 내부에서 half-duplex 자동 처리 (외부 회로 불필요)
- **속도**: 하드웨어 UART 사용으로 1Mbps 풀속도 가능

### 전원 분배

| 전원 | 출력 | 공급 대상 |
|------|------|-----------|
| LW-K3010D 벤치 파워서플라이 | 7.4V (또는 9V), CC 8A | Bus Servo Adapter DC 잭 → STS3215 5축 |
| RPi5 27W USB-C 충전기 | 5V/5A | RPi5 + Hailo 8L |
| 노트북 USB | 5V | Arduino Uno (5V 핀에서 SG90, VL53L0X 분기) |

**공통 GND**: WAGO 분배기로 노트북 GND ↔ 파워서플라이 GND ↔ Arduino GND ↔ Bus Servo Adapter GND를 한 노드로 통합.

---

## Arduino 핀 배치 요약

| 핀 | 용도 |
|----|------|
| D0 | UART RX (Bus Servo Adapter TXD) |
| D1 | UART TX (Bus Servo Adapter RXD) |
| D2 | MCP2515 INT |
| D3 | E-stop 인터럽트 |
| D9 | SG90 PWM |
| D10 | MCP2515 CS |
| D11 | MCP2515 MOSI |
| D12 | MCP2515 MISO |
| D13 | MCP2515 SCK |
| A4 | I2C SDA (VL53L0X) |
| A5 | I2C SCL (VL53L0X) |
| D4~D8, A0~A3 | 예비 (리미트 스위치, 상태 LED 등) |

---

## 시스템 전체 신호 흐름

```
[노트북] ──USB── [Arduino Uno] ──SPI── [MCP2515] ═CAN═ [CANable Pro] ──USB── [RPi5 + Hailo 8L]
              │                                                                    │
              ├─ I2C ─── [VL53L0X (ToF)]                                          │
              ├─ GPIO ── [E-stop 버튼]                                            │
              ├─ PWM ─── [SG90 그리퍼]                                       ──USB── [ArduCam 글로벌 셔터]
              └─ UART ── [Bus Servo Adapter] ──── [STS3215 #1 → #2 → #3 → #4 → #5]
                                ▲
                                │ DC 잭
                       [LW-K3010D 파워서플라이]

[노트북] ──USB── [웹캠 1, 웹캠 2]
```

---

## 역할 분담

| 컴포넌트 | 책임 |
|----------|------|
| **RPi5 + Hailo 8L** | 카메라 영상 처리, AI 추론, 위치 추정, 경로 계획, 메인 제어 루프 |
| **CAN 통신** | RPi5와 Arduino 간 명령/상태 실시간 교환 |
| **Arduino Uno** | 저수준 I/O 처리 (서보 명령 중계, 센서 읽기, E-stop, 그리퍼) |
| **Bus Servo Adapter** | UART → STS3215 half-duplex 변환, 전원 분배 |
| **STS3215 × 5** | 5축 관절 구동 (피드백 포함) |
| **SG90** | 그리퍼 개폐 |
| **VL53L0X** | 그리퍼 ↔ 대상물 거리 측정 |
| **글로벌 셔터 카메라** | 볼트/너트 실시간 판별 |
