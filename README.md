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
| Nucleo-F103RB | STS3215 6축(Bus Servo Adapter UART) + MG90 그리퍼(PWM) 제어 | 5V (XL4015 → WAGO) |
| Arduino Nano #1 | VL53L0X ToF 센서 처리 | 5V (XL4015 → WAGO) |
| Arduino Nano #2 | E-stop 처리 | 5V (XL4015 → WAGO) |

### 통신 백본 (CAN)

```
                     ┌── CANable Pro ── RPi5
                     │
   WAGO (CAN-H) ─────┼── SN65HVD230 ── Nucleo-F103RB
                     │
   WAGO (CAN-L) ─────┼── MCP2515 ── Arduino Nano #1 (ToF)
                     │
                     └── MCP2515 ── Arduino Nano #2 (E-stop)
```

- CAN 버스는 WAGO 2개(CAN-H, CAN-L)로 공통 분기
- RPi5 ↔ CANable Pro: USB
- Nucleo ↔ SN65HVD230 (Waveshare B형 CAN 트랜시버 보드)
- 각 Arduino Nano ↔ MCP2515 (SPI)
- 종단저항 120Ω: 버스 양 끝단 노드에서 활성화
- 공통 GND도 별도 WAGO로 통합

### 카메라

| 카메라 | 연결 | 용도 |
|--------|------|------|
| ArduCam USB 글로벌 셔터 | RPi5 USB3 | 그리퍼 부착, 실시간 볼트/너트 인식 |
| 웹캠 1 | 노트북 USB | 영상 촬영 |
| 웹캠 2 | 노트북 USB | 영상 촬영 |

### 센서 / 액추에이터

| 장치 | 연결 MCU | 인터페이스 |
|------|----------|-----------|
| VL53L0X (ToF) | Arduino Nano #1 | I2C |
| E-stop 버튼 | Arduino Nano #2 | GPIO 인터럽트 |
| MG90 그리퍼 서보 | Nucleo-F103RB | PWM |
| Bus Servo Adapter | Nucleo-F103RB | UART |

### 서보 제어 라인

```
Nucleo-F103RB ──UART── Bus Servo Adapter ── Servo #1 ── #2 ── #3 ── #4 ── #5 ── #6
                              ▲
                              │
                       7.4V (SMPS → 퓨즈)
```

- 서보: Feetech STS3215 × 6축
- Bus Servo Adapter의 서보 포트 #1에 첫 서보 연결 후 데이지체인으로 6축 확장
- Half-duplex는 Bus Servo Adapter 내부에서 자동 처리
- MG90 그리퍼 서보는 Nucleo PWM 핀에서 직접 제어 (Bus Servo Adapter와 별개 라인)

### 전원 분배

```
SMPS Mean Well RSP-200-7.5 (7.4V 출력)
   │
   ├─ 퓨즈 ── Bus Servo Adapter ── STS3215 × 6축
   │
   └─ 퓨즈 ── XL4015 강하형 DC-DC 컨버터 (FND 전압표시, 가변)
                  │ 5V 출력
                  └── WAGO (5V 분배)
                          ├── Nucleo-F103RB
                          ├── Arduino Nano #1 (ToF)
                          └── Arduino Nano #2 (E-stop)
```

| 전원 | 출력 | 공급 대상 |
|------|------|-----------|
| SMPS Mean Well RSP-200-7.5 | 7.4V | Bus Servo Adapter → STS3215 6축 |
| XL4015 강하형 DC-DC 5A 가변 컨버터 (FND 표시) | 5V | Nucleo-F103RB, Arduino Nano × 2 |
| RPi5 27W USB-C 충전기 | 5V/5A | RPi5 + Hailo 8L |

**공통 GND**: WAGO 분배기로 SMPS GND ↔ XL4015 GND ↔ Bus Servo Adapter GND ↔ Nucleo GND ↔ Arduino Nano GND ↔ CAN 트랜시버 GND를 하나의 노드로 통합.

---

## 시스템 전체 신호 흐름

```
                       ┌── CANable Pro ── USB ── RPi5 + Hailo 8L ── USB3 ── ArduCam 글로벌 셔터
                       │
   CAN 버스 (WAGO) ────┤── SN65HVD230 ── Nucleo-F103RB ──UART── Bus Servo Adapter ── STS3215 × 6
                       │                       │
                       │                       └─ PWM ── MG90 그리퍼
                       │
                       ├── MCP2515 ── Arduino Nano #1 ── I2C ── VL53L0X (ToF)
                       │
                       └── MCP2515 ── Arduino Nano #2 ── GPIO ── E-stop 버튼

   전원: SMPS RSP-200-7.5 7.4V ─┬─ 퓨즈 ─ Bus Servo Adapter (서보 전원)
                                 └─ 퓨즈 ─ XL4015 5V 강하 ─ WAGO ─ Nucleo / Nano×2

   [노트북] ── USB ── 웹캠 1, 웹캠 2
```

---

## 역할 분담

| 컴포넌트 | 책임 |
|----------|------|
| RPi5 + Hailo 8L | 메인 제어, 카메라 영상 처리, AI 추론, 경로 계획 |
| CAN 버스 (WAGO) | RPi5 ↔ Nucleo ↔ Nano×2 간 명령/상태 실시간 교환 |
| Nucleo-F103RB | STS3215 6축 + MG90 그리퍼 제어 |
| Arduino Nano #1 | ToF 거리 측정 |
| Arduino Nano #2 | E-stop 인터럽트 처리 |
| Bus Servo Adapter | UART → STS3215 half-duplex 변환, 서보 전원 분배 |
| STS3215 × 6 | 6축 관절 구동 (피드백 포함) |
| MG90 | 그리퍼 개폐 |
| VL53L0X | 그리퍼 ↔ 대상물 거리 측정 |
| 글로벌 셔터 카메라 | 볼트/너트 실시간 판별 |
| SMPS RSP-200-7.5 + XL4015 | 7.4V 서보 전원 + 5V 로직 전원 통합 분배 |
