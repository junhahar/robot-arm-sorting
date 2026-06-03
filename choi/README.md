# choi — 5축(6모터) 로봇팔 STM32 + RPi 제어 코드

삼보모터스 5축 로봇팔(STS3215 ×6)의 **STM32 펌웨어 + 라즈베리파이 제어/역기구학** 코드 모음.

```
RPi5 (CANable) ─ CAN 125kbps/62.5% ─ STM32 (NUCLEO-F103RB) ─ UART ─ STS3215 ×6 (ID 1~6)
                                      + Arduino Nano (가상센서, 0x300)
```

---

## 폴더 구조
```
choi/
├── stm32/        STM32 펌웨어 (PlatformIO)
│   ├── src/main.cpp
│   └── platformio.ini
├── rpi/          라즈베리파이 파이썬 코드
│   ├── rpi_keyboard_control_v2.py   주력: 키보드 수동 제어 (축단위·거울쌍·프리로드·EEPROM 영점)
│   ├── rpi_current_monitor.py       전류 실시간 모니터 (읽기 전용)
│   └── arm_ik.py                    역기구학 (좌표 → 모터각)
└── docs/
    ├── 로봇팔_작업정리.md           전체 작업 정리 (프로토콜·영점·IK·과열 등)
    └── CAN_통신_규약.md             CAN 통신 규약 (브링업·트러블슈팅)
```

---

## CAN 프로토콜 (요약)

| ID | 방향 | 용도 |
|---|---|---|
| `0x100` | RPi→STM | 모터 명령 |
| `0x200` | STM→RPi | ACK |
| `0x201` | STM→RPi | 텔레메트리 (각도·온도·부하) |
| `0x202` | STM→RPi | 전류 |
| `0x300` | Nano→RPi | 가상 센서 |

**0x100 명령:** `0x01`=위치(step), `0x03`=각도(angle×10), `0x04`=영점180°(중위보정), `0x05`=영점 임의각(오프셋 기록), `0x02`=토크, `0x7F`=ping.

물리: 125 kbps / sample-point 62.5% / PB8=RX, PB9=TX (AFIO 리맵). 자세한 건 `docs/CAN_통신_규약.md`.

---

## 빌드 / 실행

### STM32 (PlatformIO, board: nucleo_f103rb)
```bash
pio run                 # 빌드
pio run -t upload       # ST-Link 업로드
```

### RPi (CAN 셋업 후 실행)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 sample-point 0.625
sudo ip link set can0 up

python3 rpi/rpi_keyboard_control_v2.py   # 키보드 제어
python3 rpi/rpi_current_monitor.py       # (다른 터미널) 전류 모니터
python3 rpi/arm_ik.py                     # 역기구학 self-test (FK 라운드트립)
```

---

## 주요 특징

- **키보드 제어 v2**: 모터를 논리 축으로 묶음. **M2·M3 거울쌍**(어깨, 한 관절 두 모터), M4 gain 5배(무거운 관절), 누르고 있으면 속도 램프.
- **영점 보정**: 180°(중위보정) + 임의각(서보 EEPROM 오프셋 직접 기록). 모터 안 움직이고 영구 저장.
- **거울쌍 부하 분담(프리로드)**: M2·M3가 부하를 나눠 들도록 실시간(`[` `]` 키) 튜닝. 전류 모니터로 M2≈M3 확인.
- **역기구학(arm_ik.py)**: 기하학적 2링크 해석해. 위치+접근방향만 제약(5축 적합), 모터별 영점기준 변환. FK 라운드트립 0mm 검증.

자세한 설계·실측 캘리브레이션·트러블슈팅은 `docs/로봇팔_작업정리.md` 참고.
