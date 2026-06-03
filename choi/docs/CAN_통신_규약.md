# 삼보모터스 로봇암 CAN 통신 규약

확정일: 2026-05-30
검증: RPi5(CANable Pro) ↔ STM32(NUCLEO-F103RB) ↔ Nano(MCP2515) 3노드 동작 확인

> **본 문서는 두 번의 디버깅 세션(어제 RPi↔STM32, 오늘 RPi↔Nano)을 통합한 최종 규약이다.**
> 어제 결정된 500 kbps / 87.5% 셋업은 8MHz MCP2515가 합류하는 순간 호환 불가하므로 폐기되었고, 본 문서의 125 kbps / 62.5% 값이 최종이다.

---

## 1. 토폴로지

```
   [종단 120Ω]                                                [종단 120Ω]
        |                                                           |
   RPi5 (CANable) ─── STM32 NUCLEO-F103RB ─── Arduino Nano
   gs_usb         │   SN65HVD230 (R2=120Ω)  │  MCP2515 (J1 OFF)
                  │                          │
              양 끝 노드 (종단 활성)       중간 노드 (종단 해제)
```

종단저항: 양 끝에 120Ω × 2 = **60Ω** (CANable 내장 120 + SN65HVD230 R2 120)
중간 노드 MCP2515 모듈의 **내장 종단 점퍼 J1은 반드시 OFF** (셋 다 ON이면 40Ω로 떨어져 신호 왜곡)
공통 GND: 전 노드 GND 한 점으로 묶을 것. 별도 전원 사용 시 single-point ground 유지

---

## 2. 물리 계층 (Physical Layer)

| 항목 | 값 | 비고 |
|---|---|---|
| 비트레이트 | **125 kbps** | 8MHz MCP2515의 샘플포인트 한계로 500k 사용 불가 |
| 샘플 포인트 | **62.5%** | MCP2515 8MHz @ 125k 라이브러리 고정값. RPi/STM32 모두 강제 매칭 필수 |
| 프레임 포맷 | Standard 11-bit ID | Extended 사용 안 함 |
| 종단 | 60Ω (120 ‖ 120) | 멀티미터로 CANH-CANL 측정 시 60Ω 떠야 정상 |

---

## 3. CAN ID 할당표

| ID | 방향 | 용도 | DLC | 노드 |
|---|---|---|---|---|
| `0x100` | RPi → STM32 | 모터 명령 | 8 | rpi_motor_control.py |
| `0x200` | STM32 → RPi | 모터 ACK | 3 | NUCLEO main.cpp |
| `0x300` | Nano → RPi | 센서 데이터 (Nano #1) | 8 | nano_can_sensor_sender.ino |
| `0x37F` | RPi → Nano | PING 요청 (진단) | 1 | 선택 |
| `0x3FF` | Nano → RPi | PING ACK (진단) | 3 | 선택 |
| `0x400`~ | 향후 | Nano #2 이상 추가 노드 | - | 0x100/0x200 절대 재사용 금지 |

---

## 4. 프레임 포맷

### 4.1 모터 명령 (`0x100`, RPi → STM32)

| 바이트 | 의미 |
|---|---|
| [0] | cmd: `0x01`=위치, `0x02`=토크, `0x7F`=ping |
| [1] | motor_id (1~5) |
| [2..3] | pos_lo, pos_hi (0~4095 step, little-endian) |
| [4..5] | (예약 0) |
| [6] | torque (0/1) — cmd=0x02일 때만 |
| [7] | (예약 0) |

### 4.2 모터 ACK (`0x200`, STM32 → RPi)

| 바이트 | 의미 |
|---|---|
| [0] | cmd (echo) |
| [1] | motor_id (echo) |
| [2] | status: `0x00`=OK, `0xFF`=error |

### 4.3 센서 데이터 (`0x300`, Nano → RPi, little-endian)

| 바이트 | 의미 | 변환 |
|---|---|---|
| [0..1] | 온도 int16 | ÷100 → °C |
| [2..3] | 습도 uint16 | ÷100 → % |
| [4..5] | 조도 uint16 | → lux |
| [6..7] | 카운터 uint16 | 시퀀스 검증용 |

주기 100ms.

---

## 5. 노드별 셋업

### 5.1 RPi5 (CANable Pro, gs_usb 드라이버)

```
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 sample-point 0.625
sudo ip link set can0 up
ip -details link show can0
```

**⚠️ 두 번째 줄은 반드시 한 줄로 입력할 것.** `type can`부터 `sample-point 0.625`까지가 하나의 명령이다. 줄바꿈하면 `sample-point: command not found` 에러가 난다(오늘 세션에서 실제로 발생).

확인: 출력에 `bitrate 125000 sample-point 0.625` 정확히 찍혀야 함.

부팅시 자동화 — `/etc/network/interfaces.d/can0`에 등록 권장:

```
auto can0
iface can0 inet manual
    pre-up /sbin/ip link set $IFACE type can bitrate 125000 sample-point 0.625
    up    /sbin/ip link set $IFACE up
    down  /sbin/ip link set $IFACE down
```

**주의:** `restart-ms` 옵션은 gs_usb가 지원 안 함 (`Error: Device doesn't support restart from Bus Off`). 빼고 사용. Bus-off 발생시 수동으로 `ip link set can0 down && up` 필요.

(선택) 송신 큐 크기를 늘리고 싶으면: `sudo ip link set can0 txqueuelen 1000`

### 5.2 STM32 (NUCLEO-F103RB, HAL CAN)

> **⚠️ 어제 셋업 대비 변경 — BS1/BS2 재조정**
> 어제까지는 `Prescaler=4, BS1=13TQ, BS2=2TQ` (87.5%)로 RPi와 통신했으나, 이는 RPi↔STM32 2노드 한정으로만 동작한다. Nano(MCP2515 62.5%) 합류 시 STM32도 **62.5%로 통일** 필수.
> PCLK1=32MHz, Prescaler=16, TQ=16 조건에서 62.5%는 `BS1=9TQ, BS2=6TQ`임 ((1+9)/16 = 0.625).

```c
// CAN 핀: PB8(RX), PB9(TX)  — AFIO CAN1_2 리맵
AFIO->MAPR = (AFIO->MAPR & ~(0x3UL << 13)) | (0x2UL << 13);

// 125kbps @ PCLK1=32MHz
// 32MHz / 16 / (1+9+6) = 125kbps, 샘플포인트 62.5%
hcan.Init.Prescaler     = 16;
hcan.Init.TimeSeg1      = CAN_BS1_9TQ;   // ★ 어제 13TQ → 9TQ
hcan.Init.TimeSeg2      = CAN_BS2_6TQ;   // ★ 어제 2TQ  → 6TQ
hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
hcan.Init.AutoRetransmission = ENABLE;   // 실서비스용 (디버깅 단계에선 DISABLE도 가능)
```

**RX 필터 (필수)**

기본값으로는 모든 프레임이 거절된다. 최소 1개 all-pass 필터 활성화:

```c
CAN_FilterTypeDef f = {0};
f.FilterBank = 0;
f.FilterMode = CAN_FILTERMODE_IDMASK;
f.FilterScale = CAN_FILTERSCALE_32BIT;
f.FilterIdHigh = 0; f.FilterIdLow = 0;
f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;   // 마스크 0 = 모두 통과
f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
f.FilterActivation = ENABLE;
HAL_CAN_ConfigFilter(&hcan, &f);
```

**물리적 주의사항:**
- SN65HVD230 모듈의 **CANH/CANL 단락 점퍼 반드시 제거** (없으면 dominant 비트 못 보냄)
- PA11/PA12는 NUCLEO에서 USB 회로(CN5)에 물려있어 CAN 사용 불가 → PB8/PB9 리맵 필수
- 빌드 플래그에 `-DHAL_CAN_MODULE_ENABLED` 필수 (없으면 컴파일 에러)

### 5.3 Arduino Nano (MCP2515 모듈, coryjfowler/mcp_can)

```cpp
// 핀: VCC=5V, GND=GND, CS=D10, SI=D11, SO=D12, SCK=D13, INT=D2
CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ);  // 크리스털 8MHz 확인 필수
CAN0.setMode(MCP_NORMAL);
```

**물리적 주의사항:**
- MCP2515 모듈의 크리스털 마킹 확인 (`8.000` 또는 `16.000`). 코드와 일치 필수
- **MCP2515 모듈 내장 종단 점퍼 J1은 OFF로 둘 것.** 양 끝 종단은 CANable과 SN65HVD230이 담당. 셋 다 활성이면 40Ω로 떨어져 통신 불가
- 전원 모두 끄고 멀티미터로 CANH-CANL 측정 → **60Ω 떠야 정상**
- TJA1050 트랜시버 부분만 고장나는 케이스 있음 (루프백은 되는데 NORMAL에서 ACK 안 옴) → 모듈 교체로 해결 (오늘 세션 최종 원인)

---

## 6. 트러블슈팅 체크리스트

### 6.1 stat=6 / LEC=3 / ACK 없음 — 두 세션에서 확립된 정석 순서

증상별이 아닌 **진단 순서**로 진행. 위에서부터 한 단계씩 통과시키며 좁혀간다.

1. **RPi 자기 송신 검증** — 한 창에 `candump can0`, 다른 창에 `cansend can0 123#DEADBEEF`. 자기 candump에 뜨면 RPi/CANable는 정상
2. **Nano 루프백 모드** — `#define LOOPBACK_TEST 1`로 빌드. `TX OK` 카운터 증가하면 MCP2515 칩+SPI+크리스털 정상
3. **RPi 카운터 해석** — `ip -details -statistics link show can0`의 `RX`, `bus-errors` 동시 관찰 → 6.2 참조
4. **멀티미터 도통/종단** (전원 모두 OFF)
   - CANH↔CANH, CANL↔CANL 도통 (0Ω 근처)
   - CANH-CANL 사이 60Ω (∞면 종단 빠짐, 40Ω이면 종단 셋 이상 → MCP2515 J1 OFF 확인)
5. **CANH/CANL 좌우 스왑** — 보드 라벨 거꾸로 박힌 모듈 존재. 30초 만에 검증
6. **GND 도통 실측** — "공통 GND"라 알고 있어도 멀티미터로 직접 확인. 한 노드라도 GND가 떠 있으면 차동 신호 기준 무너짐
7. **모듈 교체** — 위 모두 통과인데도 안 되면 TJA1050 트랜시버 사망 가능성. 동일 사양 새 모듈로 교체해 분기 (오늘 세션 최종 원인)

### 6.2 RPi 카운터 해석 (`ip -details -statistics link show can0`)

| 조합 | 의미 | 주 의심처 |
|---|---|---|
| `RX=0` ∧ `bus-errors=0` | 비트 자체가 도달 안 함 | 와이어 단선, CANH/CANL 스왑, 트랜시버 사망 |
| `RX=0` ∧ `bus-errors > 0 ↑` | 비트는 도달, 깨짐 | 샘플포인트/비트레이트 불일치, 종단 |
| `state BUS-OFF` | 누적 오류로 노드 차단 | 수동 `down && up` 복구. gs_usb는 자동 복구 없음 |

`bus-errors`가 증가하느냐 0이냐가 **물리 단절 vs 타이밍 오류**를 가르는 결정적 지표.

### 6.3 RX 데이터 깨짐 (LEN=15, ID=0x0 등)

- 비트 타이밍 불일치 (샘플포인트 어긋남). 5.1의 sample-point 명령 재확인
- 크리스털 미스매치 (코드 `MCP_8MHZ` vs 실제 16MHz 등)

---

## 7. 트래픽 마진 (참고)

현재 사용률 (125kbps 기준):

| 시나리오 | 프레임/초 | 사용률 |
|---|---|---|
| 모터 명령 50Hz + ACK | 100 | 9% |
| Nano 센서 10Hz | 10 | 1% |
| **합계 (현재)** | **110** | **10%** |
| 향후 Nano #2 추가 (10Hz) | +10 | +1% |

CAN 권장 사용률 30~40% 대비 충분히 여유. 노드 5~6개까지 확장 가능.

---

## 8. 두 세션에서 얻은 교훈

1. **500kbps는 8MHz MCP2515와 양립 불가** — 라이브러리 샘플포인트 62.5%, gs_usb는 87.5%. 비트폭 2μs에서 25%p 차이 흡수 불가
2. **125k에서도 샘플포인트 명시 매칭 필요** — 비트폭 8μs라 우연히 통과되기도 하지만 환경 변화에 취약. RPi에서 0.625로 강제하는 게 결정론적
3. **루프백 OK ≠ NORMAL OK** — MCP2515 컨트롤러와 TJA1050 트랜시버는 같은 보드여도 독립 회로. 트랜시버만 사망 가능 (오늘 세션 최종 원인)
4. **STM32 SN65HVD230 점퍼는 CANH/CANL 단락용** — 반드시 제거. 안 빼면 dominant 비트 안 나감
5. **PA11/PA12 못 씀** — NUCLEO USB 회로와 충돌. PB8/PB9 AFIO 리맵
6. **3노드 종단은 양 끝 두 개만** — 모든 트랜시버 모듈에 내장 종단이 있으면 무신경하게 합쳐서 40Ω 됨. MCP2515 J1 OFF로 중간 노드 종단 해제
7. **gs_usb는 `restart-ms` 미지원** — Bus-off 자동복구 안 됨. 수동 down/up
8. **`bus-errors=0` ∧ `RX=0` = 물리 단절** — 비트가 도달했는데 깨졌으면 bus-errors가 즉시 증가. 이 두 지표로 단선과 타이밍 오류를 빠르게 분리

---

## 9. 자주 쓰는 명령어

### RPi 측

```
# 기동
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 sample-point 0.625
sudo ip link set can0 up

# 상태 / 통계
ip -details link show can0
ip -details -statistics link show can0

# 수신 / 송신
candump can0
candump -tz can0
cansend can0 123#DEADBEEF
cansend can0 37F#7F                    # Nano ping
cansend can0 100#7F00000000000000      # STM32 ping

# Bus-off 수동 복구
sudo ip link set can0 down && sudo ip link set can0 up
```

### Nano 측 시리얼 로그 해석

```
TX OK cnt=N             정상 송신
TX FAIL stat=6          GETTXBFTIMEOUT — ACK 없음 (가장 흔함)
TX FAIL stat=8          GETMSGBUFTIMEOUT — 송신 버퍼 풀
[DIAG] TEC=N REC=M      송수신 오류 카운터. TEC만 증가 = ACK 못 받음
```
