# Robot Arm Sorting

삼보모터스 실무 프로젝트. 5축 로봇암으로 볼트/너트를 인식해 자동 분류한다.

## 프로젝트 개요

전체 동작은 **마스터 궤적 재생 + Checkpoint AI 보정 하이브리드** 방식이다. 사람이 시연한 큰 동선을 그대로 재생하되, 정밀이 필요한 핵심 구간에서만 카메라·ToF 센서로 실시간 보정한다.

1. **하드웨어 구축** — 5관절축 로봇암 프레임 설계·제작, 부품 통합 및 배선 정리
2. **MPU 웨이포인트 티칭** — 사람 팔에 부착한 MPU로 동작을 녹화해 마스터 궤적(CSV) 생성
3. **볼트/너트 딥러닝 분류** — 그리퍼에 부착된 글로벌 셔터 카메라 + YOLO(Hailo 8L)로 실시간 판별
4. **실시간 위치 보정** — 작업 도중 물체 위치가 바뀌면 Checkpoint(SCAN/PRE_GRASP/GRASP/PLACE)에서 카메라·ToF·ArUco로 경로 보정

```
[티칭]  Nano#2(USB) + TCA9548A + MPU6050×3 → 노트북 → 웨이포인트 저장 → master_trajectory.csv
[자동]  RPi5가 CSV 재생 → checkpoint에서 YOLO/ToF/ArUco 보정 → 반복
```

---

## 하드웨어 구성

### 컴퓨팅 / 제어

| 장치 | 역할 | 비고 |
|------|------|------|
| 라즈베리파이5 + Hailo 8L | AI 추론(분류), 메인 제어, 궤적 재생 | CAN 마스터 |
| 노트북 | 티칭 데이터 수집, 모델 학습, 디버깅 | CAN 비참여 |
| STM32F103RB (Nucleo-F103RB) | 서보 6개 구동, 그리퍼 PWM | CAN 노드 |
| Arduino Nano #1 | VL53L0X(ToF) 읽어 CAN 송신 | CAN 노드 (MCP2515) |
| Arduino Nano #2 | MPU6050×3 읽어 노트북으로 시리얼 전송 | USB 티칭 전용, CAN 비참여 |

> E-stop은 프로젝트 방향 변경으로 제거되었다.

### 구동부

| 장치 | 수량 | 비고 |
|------|------|------|
| Feetech STS3215 | 6 | 5관절 구동 (어깨에 2개 병렬로 토크 보강) |
| MG90 메탈 기어 그리퍼 | 1 | 그리퍼 개폐 (SG90에서 교체) |
| Bus Servo Adapter | 1 | UART → STS3215 half-duplex 변환, 데이지체인 |

### 센서 / 카메라

| 장치 | 인터페이스 | 연결 |
|------|-----------|------|
| ArduCam USB 글로벌 셔터 | USB3 | RPi5, 그리퍼 eye-in-hand (볼트/너트 인식) |
| VL53L0X (ToF) | I2C | Nano #1 경유 → CAN |
| MPU6050 ×3 + TCA9548A | I2C | Nano #2 경유 → USB (티칭 전용) |
| 웹캠 ×2 | USB | 노트북, 영상 촬영·학습 |

### 전원

| 전원 | 출력 | 공급 대상 |
|------|------|-----------|
| Mean Well RSP-200-7.5 | 7.5V 정격 → 7.4V로 트림 | Bus Servo Adapter → STS3215 ×6 |
| XL4015 가변 벅 | 5V | STM32 E5V + Nano ×2 |
| RPi5 27W USB-C PD | 5V | RPi5 + Hailo 8L |

> 공통 GND 필수(서보 어댑터·컨버터·MCU 전부). 서보 전원(7.4V)을 MCU 로직 VCC에 직결 금지.

---

## CAN 통신 규약

> 앞으로 모든 CAN 통신은 아래 규약을 지켜서 구현한다. 노드 추가·펌웨어 수정 시 이 표를 기준으로 한다.

### 버스 설정

| 항목 | 값 |
|------|-----|
| 비트레이트 | **125 kbps** (전 노드 통일) |
| 샘플포인트 | **0.625** (RPi 쪽에서 명시적으로 강제) |
| ID 형식 | 표준 11-bit |
| 토폴로지 | 데이지체인, 양 끝단에만 120Ω 종단 (정상 측정값 60Ω) |

```
RPi5 ──USB── CANable Pro ═══CAN(H/L) 데이지체인═══ STM32 ─── Nano#1
            (종단 120Ω)                                    (종단 120Ω)
```

전체 CAN 노드는 **RPi5 + STM32 + Nano#1**, 총 3개다. (Nano#2는 USB 티칭 전용으로 버스에 참여하지 않는다.)

### 노드별 비트레이트 설정값

| 노드 | 설정 |
|------|------|
| RPi5 (CANable / socketcan) | `bitrate 125000`, `sample-point 0.625` 명시 |
| STM32F103RB | `Prescaler=16`, `BS1=13TQ`, `BS2=2TQ` (PCLK1 32MHz 기준 → 125k) |
| Nano #1 (MCP2515 8MHz) | `CAN_125KBPS`, `MCP_8MHZ` |

> STM32의 CAN 핀은 **PB8(RX) / PB9(TX)** 로 AFIO 리맵해서 사용한다. (PA11/PA12는 보드의 USB 회로와 충돌)

### CAN ID 할당표

| ID | 방향 | 용도 | 데이터 |
|------|------|------|--------|
| `0x100` | RPi5 → STM32 | 모터 명령 | `[cmd, motor_id, pos_lo, pos_hi, 0, 0, torque, 0]` |
| `0x200` | STM32 → RPi5 | 모터 ACK | `[cmd, motor_id, status]` (status: `0x00`=OK, `0xFF`=오류) |
| `0x300` | Nano#1 → RPi5 | ToF 거리 데이터 | `[distance_lo, distance_hi]` |
| `0x37F` | RPi5 → Nano#1 | PING 요청 | (선택) |
| `0x3FF` | Nano#1 → RPi5 | PING ACK | |

**cmd 코드**

| cmd | 의미 |
|-----|------|
| `0x01` | 위치 지령 (0~4095 step) |
| `0x02` | 토크 ON/OFF |
| `0x7F` | ping |

> `0x100` / `0x200` 은 RPi↔STM32 전용이므로 절대 재사용 금지. 신규 CAN 노드를 추가할 경우 `0x400` 대역부터 할당한다.
