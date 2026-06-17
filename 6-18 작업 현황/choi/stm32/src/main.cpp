// ============================================================
// NUCLEO-F103RB: Raspberry Pi IK 결과 수신 -> STS3215 모터 제어
// ============================================================
//
// 역할 분담:
//   1. Raspberry Pi가 카메라/AI/역기구학(IK) 알고리즘을 계산합니다.
//   2. Raspberry Pi가 계산된 각도를 CAN 0x100 명령으로 STM32에 보냅니다.
//   3. STM32는 받은 목표 각도를 STS3215 위치값으로 바꿔 모터를 제어합니다.
//   4. STM32는 현재 각도/온도/부하를 0x201로, 전류를 0x202로 RPi에 다시 보냅니다.
//
// CAN 물리 설정 (기존 규약 유지): 125kbps / sample point 62.5% / 11-bit ID
//   PB8=CAN_RX, PB9=CAN_TX, AFIO CAN1_2 remap
//
// CAN ID:
//   0x100: RPi -> STM32 모터 명령
//   0x200: STM32 -> RPi ACK
//   0x201: STM32 -> RPi 모터 상태 텔레메트리 (각도/온도/부하)
//   0x202: STM32 -> RPi 전류 [motor_id, cur_lo, cur_hi, ok]   ← 추가
//
// 0x100 명령 프레임:
//   cmd=0x01: 위치 step 명령   [0x01, id, step_lo, step_hi, move_ms_lo, move_ms_hi, 0, 0]
//   cmd=0x03: 각도 명령        [0x03, id, ax10_lo, ax10_hi, move_ms_lo, move_ms_hi, 0, 0]
//   cmd=0x04: 영점 보정        [0x04, id, ...] 현재 위치를 180°(중위)로 (모터 안 움직임)  ← 추가
//   cmd=0x02: 토크 ON/OFF      [0x02, id, 0,0,0,0, torque, 0]
//   cmd=0x06: MG90 그리퍼 PWM  [0x06, angle(0~180), 0,0,0,0,0,0]   ← 추가 (STM32 직접 PWM)
//   cmd=0x7F: ping
//
// 0x201 텔레메트리 (DLC 7): [id, ax10_lo, ax10_hi, tempC, load_lo, load_hi, flags]
//   flags bit0=각도OK, bit1=온도OK, bit2=부하OK
//
// 0x202 전류 (DLC 4): [id, cur_lo, cur_hi, ok(1=읽기성공)]
//   값 × 약 6.5mA = 전류 (스케일은 RPi에서 적용)
// ============================================================

#include <Arduino.h>
#include "stm32f1xx_hal.h"
#include <Servo.h>   // MG90 그리퍼 PWM (STM32duino Servo)

// STS3215 버스 서보 어댑터 UART입니다.
// HardwareSerial 생성자 순서는 (RX, TX)이므로 PA10=RX, PA9=TX입니다.
HardwareSerial ServoSerial(PA10, PA9);

#define SERVO_COUNT 6

// MG90 그리퍼 (PWM 서보) — STS3215와 별개, STM32가 직접 PWM 구동
#define GRIPPER_PWM_PIN  PA6    // TIM3_CH1 (사용 중인 핀과 충돌 없음)
#define GRIPPER_INIT_DEG 90
// MG90S 펄스폭 범위[us]. STM32duino Servo 기본(544~2400)은 MG90S 끝단과 안 맞아
// 0°/180°에서 위치를 못 찾고 계속 도는 일이 있다 → 실측 일반값 600~2400으로 명시.
// (서보가 끝에서 여전히 떨거나 돌면 이 두 값을 좁혀가며 맞춘다: 예 700~2300)
#define GRIPPER_PULSE_MIN_US 600
#define GRIPPER_PULSE_MAX_US 2400

#define CAN_ID_CMD       0x100
#define CAN_ID_ACK       0x200
#define CAN_ID_TELEMETRY 0x201
#define CAN_ID_CURRENT   0x202   // 추가: 전류 [motor_id, cur_lo, cur_hi, ok]

#define CMD_SET_STEP     0x01
#define CMD_TORQUE       0x02
#define CMD_SET_ANGLE    0x03
#define CMD_SET_MIDDLE   0x04   // 현재 위치를 180°(중위)로 영점 보정 (EEPROM)
#define CMD_SET_REF      0x05   // 현재 위치를 임의 각도(angle×10)로 영점 보정 (EEPROM 오프셋 직접 기록)
#define CMD_PING         0x7F
#define CMD_GRIPPER      0x06   // MG90 그리퍼 PWM 각도 [0x06, angle(0~180)]

#define STATUS_OK        0x00
#define STATUS_ERROR     0xFF

static CAN_HandleTypeDef hcan;

Servo gripper;   // MG90 그리퍼

// STS3215 프로토콜 명령과 레지스터 주소입니다.
static const uint8_t STS_INST_READ = 0x02;
static const uint8_t STS_INST_WRITE = 0x03;
static const uint8_t STS_ADDR_TORQUE_ENABLE = 0x28;
static const uint8_t STS_ADDR_GOAL_POSITION = 0x2A;
static const uint8_t STS_ADDR_PRESENT_POSITION = 0x38;
static const uint8_t STS_ADDR_PRESENT_LOAD = 0x3C;
static const uint8_t STS_ADDR_PRESENT_TEMPERATURE = 0x3F;
static const uint8_t STS_ADDR_PRESENT_CURRENT = 0x45;  // 추가: 2바이트, ≈6.5mA/LSB
static const uint8_t STS_ADDR_OFFSET = 0x1F;           // 위치 보정(오프셋) L (H=0x20), bit11=부호, ±2047
static const uint8_t STS_ADDR_LOCK   = 0x37;           // EEPROM 락 (0=언락, 1=락)

// 텔레메트리는 한 번에 모터 1개만 읽습니다.
// 이유: 모터 전부를 한 loop에서 읽으면 CAN 명령 처리가 늦어집니다.
// ※ 전류를 더 빠르게 보고 싶으면 이 값을 낮추세요(예: 40~50ms). 6모터 순환이라
//   150ms면 모터당 전류 갱신이 ~0.9s로 느립니다. 50ms면 ~0.3s.
// 한 tick에 모터 1개의 '항목 1개'만 UART로 읽는다(블로킹 분산). 한 항목당 한 tick.
// 그래서 텔레메트리가 loop를 잡는 시간이 1/4로 줄어 → 그리퍼/명령 반응 지연 완화.
// 4항목(위치/온도/부하/전류)을 다 읽으면 그때 CAN 프레임(0x201/0x202)을 보낸다.
// 주기도 150→25ms로 줄임: 한 항목만 읽으니 부담 적고, 25ms×4=100ms로 모터당 1.5바퀴/0.6s 갱신.
static const uint32_t TELEMETRY_PERIOD_MS = 25;
static uint32_t lastTelemetryMs = 0;
static uint8_t telemetryMotorId = 1;
static uint8_t telemetryStage = 0;   // 0=위치,1=온도,2=부하,3=전류→전송

// 한 모터의 항목을 모으는 누적 버퍼 (4항목 다 읽으면 한 번에 전송)
static int16_t  telAngleX10 = 0;
static uint8_t  telTempC = 0;
static uint16_t telRawLoad = 0;
static uint16_t telRawCurrent = 0;
static uint8_t  telFlags = 0;      // bit0 위치, bit1 온도, bit2 부하 OK
static uint8_t  telCurOk = 0;

// CAN 송신 실패 카운터입니다.
static uint32_t canTxNoMailboxCount = 0;
static uint32_t canTxAddFailCount = 0;
static uint32_t lastDiagPrintMs = 0;


// ============================================================
// CAN 초기화와 송수신
// ============================================================
void canInit() {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // CAN1을 PB8/PB9로 리맵합니다. 기존 통신 규약의 핀 배치를 유지합니다.
    AFIO->MAPR = (AFIO->MAPR & ~(0x3UL << 13)) | (0x2UL << 13);

    GPIO_InitTypeDef gpio = {};
    gpio.Pin   = GPIO_PIN_9;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin  = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    hcan.Instance                  = CAN1;
    hcan.Init.Mode                 = CAN_MODE_NORMAL;
    hcan.Init.AutoBusOff           = ENABLE;
    hcan.Init.AutoRetransmission   = ENABLE;
    hcan.Init.AutoWakeUp           = DISABLE;
    hcan.Init.ReceiveFifoLocked    = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    // 125kbps @ PCLK1=32MHz, sample point 62.5%.
    hcan.Init.Prescaler            = 16;
    hcan.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1             = CAN_BS1_9TQ;
    hcan.Init.TimeSeg2             = CAN_BS2_6TQ;

    HAL_CAN_Init(&hcan);

    // 모든 Standard ID를 받는 all-pass 필터입니다.
    CAN_FilterTypeDef filter = {};
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0;
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = 0;
    filter.FilterMaskIdLow      = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &filter);

    HAL_CAN_Start(&hcan);
}

bool canSend(uint16_t id, uint8_t* data, uint8_t len) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) {
        canTxNoMailboxCount++;
        return false;
    }

    CAN_TxHeaderTypeDef txh = {};
    txh.StdId = id;
    txh.RTR   = CAN_RTR_DATA;
    txh.IDE   = CAN_ID_STD;
    txh.DLC   = len;

    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage(&hcan, &txh, data, &mailbox) != HAL_OK) {
        canTxAddFailCount++;
        return false;
    }

    return true;
}

bool canRecv(uint16_t* id, uint8_t* data, uint8_t* len) {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0) return false;

    CAN_RxHeaderTypeDef rxh = {};
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxh, data) != HAL_OK) return false;

    *id = (uint16_t)rxh.StdId;
    *len = (uint8_t)rxh.DLC;
    return true;
}


// ============================================================
// STS3215 저수준 통신
// ============================================================
void stsWriteReg(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len) {
    uint8_t pktLen = len + 3;
    uint8_t checksum = id + pktLen + STS_INST_WRITE + addr;

    for (uint8_t i = 0; i < len; i++) checksum += data[i];
    checksum = ~checksum;

    ServoSerial.write(0xFF);
    ServoSerial.write(0xFF);
    ServoSerial.write(id);
    ServoSerial.write(pktLen);
    ServoSerial.write(STS_INST_WRITE);
    ServoSerial.write(addr);
    for (uint8_t i = 0; i < len; i++) ServoSerial.write(data[i]);
    ServoSerial.write(checksum);
    ServoSerial.flush();
}

bool stsReadReg(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len, uint16_t timeoutMs = 12) {
    while (ServoSerial.available() > 0) ServoSerial.read();

    uint8_t pktLen = 4;
    uint8_t checksum = ~(id + pktLen + STS_INST_READ + addr + len);

    ServoSerial.write(0xFF);
    ServoSerial.write(0xFF);
    ServoSerial.write(id);
    ServoSerial.write(pktLen);
    ServoSerial.write(STS_INST_READ);
    ServoSerial.write(addr);
    ServoSerial.write(len);
    ServoSerial.write(checksum);
    ServoSerial.flush();

    uint32_t start = millis();
    uint8_t state = 0;
    uint8_t respId = 0;
    uint8_t respLen = 0;
    uint8_t err = 0;
    uint8_t params[8] = {};

    while ((uint32_t)(millis() - start) < timeoutMs) {
        if (ServoSerial.available() <= 0) continue;
        uint8_t b = (uint8_t)ServoSerial.read();

        if (state == 0) {
            if (b == 0xFF) state = 1;
        } else if (state == 1) {
            state = (b == 0xFF) ? 2 : 0;
        } else if (state == 2) {
            respId = b;
            state = 3;
        } else if (state == 3) {
            respLen = b;
            if (respLen < 2 || respLen > sizeof(params) + 2) return false;
            state = 4;
        } else if (state == 4) {
            err = b;
            state = 5;
        } else {
            uint8_t paramIndex = state - 5;
            if (paramIndex < respLen - 2) {
                params[paramIndex] = b;
                state++;
            } else {
                uint8_t sum = respId + respLen + err;
                for (uint8_t i = 0; i < respLen - 2; i++) sum += params[i];

                if ((uint8_t)(~sum) != b) return false;
                if (respId != id || err != 0 || respLen != len + 2) return false;

                for (uint8_t i = 0; i < len; i++) data[i] = params[i];
                return true;
            }
        }
    }

    return false;
}


// ============================================================
// 각도 변환과 서보 제어
// ============================================================
uint16_t clampStep(uint16_t step) {
    if (step > 4095) return 4095;
    return step;
}

uint16_t angleX10ToStep(int16_t angleX10) {
    if (angleX10 < 0) angleX10 = 0;
    if (angleX10 > 3600) angleX10 = 3600;

    uint32_t step = ((uint32_t)angleX10 * 4095UL + 1800UL) / 3600UL;
    if (step > 4095UL) step = 4095UL;
    return (uint16_t)step;
}

int16_t stepToAngleX10(uint16_t step) {
    if (step > 4095) step = 4095;
    uint32_t angleX10 = ((uint32_t)step * 3600UL + 2047UL) / 4095UL;
    if (angleX10 > 3600UL) angleX10 = 3600UL;
    return (int16_t)angleX10;
}

void setTorque(uint8_t id, bool on) {
    uint8_t value = on ? 1 : 0;
    stsWriteReg(id, STS_ADDR_TORQUE_ENABLE, &value, 1);
}

void setPositionStepTimed(uint8_t id, uint16_t step, uint16_t moveTimeMs) {
    step = clampStep(step);

    uint8_t data[4] = {
        (uint8_t)(step & 0xFF),
        (uint8_t)(step >> 8),
        (uint8_t)(moveTimeMs & 0xFF),
        (uint8_t)(moveTimeMs >> 8)
    };

    stsWriteReg(id, STS_ADDR_GOAL_POSITION, data, 4);
}

void setPositionAngleTimed(uint8_t id, int16_t angleX10, uint16_t moveTimeMs) {
    uint16_t step = angleX10ToStep(angleX10);
    setPositionStepTimed(id, step, moveTimeMs);
}

// ─── 영점 보정 (추가) ────────────────────────────────────────
// 현재 물리 위치를 180°(중위 2048)로 영점 보정합니다.
// Feetech STS: Torque_Enable(0x28)에 128을 쓰면 "중위 보정" → 현재 위치가 2048로
// 읽히도록 오프셋이 EEPROM에 영구 저장됩니다. 모터는 움직이지 않습니다.
void setMiddleCalibration(uint8_t id) {
    uint8_t v = 128;
    stsWriteReg(id, STS_ADDR_TORQUE_ENABLE, &v, 1);  // 중위 보정 명령
    delay(15);                                        // EEPROM 기록 대기
    setPositionStepTimed(id, 2048, 0);                // 새 중심(2048)을 목표로 → 안 움직임
    setTorque(id, true);                              // 토크 재확인
}

bool readServoPositionStep(uint8_t id, uint16_t* step) {
    uint8_t data[2] = {};
    if (!stsReadReg(id, STS_ADDR_PRESENT_POSITION, data, 2)) return false;
    *step = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

bool readServoTemperature(uint8_t id, uint8_t* tempC) {
    return stsReadReg(id, STS_ADDR_PRESENT_TEMPERATURE, tempC, 1);
}

bool readServoLoadRaw(uint8_t id, uint16_t* rawLoad) {
    uint8_t data[2] = {};
    if (!stsReadReg(id, STS_ADDR_PRESENT_LOAD, data, 2)) return false;
    *rawLoad = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

// ─── 전류 읽기 (추가) ────────────────────────────────────────
bool readServoCurrentRaw(uint8_t id, uint16_t* rawCurrent) {
    // Present Current(0x45) 2바이트. 값 × 약 6.5mA = 전류 (스케일은 RPi에서 적용).
    uint8_t data[2] = {};
    if (!stsReadReg(id, STS_ADDR_PRESENT_CURRENT, data, 2)) return false;
    *rawCurrent = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

// ─── 임의각 영점 보정 (추가, EEPROM 오프셋 직접 기록) ─────────
// 현재 offset 레지스터(0x1F) 읽기. bit11=부호, 하위 11비트=크기(±2047).
int16_t readServoOffsetSteps(uint8_t id) {
    uint8_t d[2] = {};
    if (!stsReadReg(id, STS_ADDR_OFFSET, d, 2)) return 0;
    uint16_t raw = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
    int16_t mag = raw & 0x07FF;
    return (raw & 0x0800) ? (int16_t)(-mag) : mag;
}

// 현재 물리 위치가 angleX10(예: 3600=360°)로 읽히도록 오프셋을 EEPROM에 영구 기록.
// 모터는 안 움직임(기록 후 목표를 그 step으로 다시 써서 좌표이동 점프 방지).
// 원리: present = raw + ofs.  새 ofs = targetStep - raw = targetStep - present + ofs_cur.
void setReferenceAngle(uint8_t id, int16_t angleX10) {
    uint16_t targetStep = angleX10ToStep(angleX10);
    uint16_t present = 0;
    if (!readServoPositionStep(id, &present)) return;   // present = raw + ofs_cur
    int16_t ofsCur = readServoOffsetSteps(id);
    int32_t newOfs = (int32_t)targetStep - (int32_t)present + (int32_t)ofsCur;
    if (newOfs > 2047) newOfs = 2047;                   // 오프셋 한계 ±2047 (≈±180°)
    if (newOfs < -2047) newOfs = -2047;

    uint16_t enc = (newOfs < 0) ? (((uint16_t)(-newOfs)) | 0x0800) : (uint16_t)newOfs;
    uint8_t ofsData[2] = { (uint8_t)(enc & 0xFF), (uint8_t)((enc >> 8) & 0xFF) };

    uint8_t unlock = 0, lock = 1;
    stsWriteReg(id, STS_ADDR_LOCK, &unlock, 1);   delay(5);   // EEPROM 언락
    stsWriteReg(id, STS_ADDR_OFFSET, ofsData, 2); delay(10);  // 오프셋 기록
    stsWriteReg(id, STS_ADDR_LOCK, &lock, 1);     delay(5);   // 락

    // ★안전★ 목표를 '지금 실제로 읽히는 현재값'으로 설정 → 오프셋이 clamp돼도 절대 안 튐.
    // (이전엔 targetStep으로 최대속도 명령 → 오프셋 못 맞추면 그 차이만큼 확 돌아 위험했음)
    uint16_t presentNew = present;
    readServoPositionStep(id, &presentNew);
    setPositionStepTimed(id, presentNew, 0);
}


// ============================================================
// Raspberry Pi로 상태 회신 (블로킹 분산: 한 tick에 한 항목만 UART 읽기)
// ============================================================
// 한 모터에 대해 stage 0→1→2→3 순으로 한 항목씩 읽어 누적하고,
// stage 3(전류)에서 두 프레임(0x201 텔레메트리, 0x202 전류)을 한 번에 보낸 뒤
// 다음 모터로 넘어간다. 이렇게 하면 loop가 한 번에 UART에 잡히는 시간이
// 4읽기 → 1읽기로 줄어 그리퍼/명령 반응 지연이 크게 완화된다.
void sendServoTelemetryTick() {
    uint32_t now = millis();
    if (lastTelemetryMs != 0 && (uint32_t)(now - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
    lastTelemetryMs = now;

    uint8_t id = telemetryMotorId;

    switch (telemetryStage) {
        case 0: {  // 위치(각도)
            uint16_t step = 0;
            if (readServoPositionStep(id, &step)) {
                telAngleX10 = stepToAngleX10(step);
                telFlags |= 0x01;
            }
            break;
        }
        case 1: {  // 온도
            uint8_t tempC = 0;
            if (readServoTemperature(id, &tempC)) {
                telTempC = tempC;
                telFlags |= 0x02;
            }
            break;
        }
        case 2: {  // 부하
            uint16_t rawLoad = 0;
            if (readServoLoadRaw(id, &rawLoad)) {
                telRawLoad = rawLoad;
                telFlags |= 0x04;
            }
            break;
        }
        case 3: {  // 전류 → 이번 모터 항목 다 모았으니 두 프레임 전송
            uint16_t rawCurrent = 0;
            telCurOk = readServoCurrentRaw(id, &rawCurrent) ? 1 : 0;
            telRawCurrent = telCurOk ? rawCurrent : 0;

            uint8_t frame[7] = {
                id,
                (uint8_t)(telAngleX10 & 0xFF),
                (uint8_t)((uint16_t)telAngleX10 >> 8),
                telTempC,
                (uint8_t)(telRawLoad & 0xFF),
                (uint8_t)(telRawLoad >> 8),
                telFlags
            };
            canSend(CAN_ID_TELEMETRY, frame, 7);

            uint8_t curFrame[4] = {
                id,
                (uint8_t)(telRawCurrent & 0xFF),
                (uint8_t)(telRawCurrent >> 8),
                telCurOk
            };
            canSend(CAN_ID_CURRENT, curFrame, 4);
            break;
        }
    }

    // 다음 단계로. stage가 한 바퀴(0~3) 돌면 다음 모터로 + 누적버퍼 리셋.
    telemetryStage++;
    if (telemetryStage > 3) {
        telemetryStage = 0;
        telFlags = 0;
        telAngleX10 = 0;
        telTempC = 0;
        telRawLoad = 0;
        telRawCurrent = 0;
        telCurOk = 0;
        telemetryMotorId++;
        if (telemetryMotorId > SERVO_COUNT) telemetryMotorId = 1;
    }
}


// ============================================================
// Raspberry Pi 명령 처리
// ============================================================
void sendAck(uint8_t cmd, uint8_t motorId, uint8_t status) {
    uint8_t ack[3] = {cmd, motorId, status};
    canSend(CAN_ID_ACK, ack, 3);
}

void handleCanFrame(uint16_t canId, uint8_t* buf, uint8_t len) {
    if (canId != CAN_ID_CMD || len < 2) return;

    uint8_t cmd = buf[0];
    uint8_t motorId = buf[1];

    if (cmd == CMD_PING) {
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    if (cmd == CMD_GRIPPER) {
        // MG90 그리퍼 PWM. buf[1]=각도(0~180). 서보 1~6이 아니므로 motorId 검증 전에 처리.
        uint8_t angle = buf[1];
        if (angle > 180) angle = 180;
        gripper.write(angle);
        sendAck(cmd, 0, STATUS_OK);
        return;
    }

    if (motorId < 1 || motorId > SERVO_COUNT) {
        sendAck(cmd, motorId, STATUS_ERROR);
        return;
    }

    if (cmd == CMD_SET_STEP && len >= 4) {
        uint16_t step = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint16_t moveTimeMs = 0;
        if (len >= 6) moveTimeMs = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

        setPositionStepTimed(motorId, step, moveTimeMs);
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    if (cmd == CMD_SET_ANGLE && len >= 4) {
        int16_t angleX10 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        uint16_t moveTimeMs = 0;
        if (len >= 6) moveTimeMs = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

        setPositionAngleTimed(motorId, angleX10, moveTimeMs);
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    if (cmd == CMD_SET_MIDDLE) {
        // 현재 위치를 180°(중위)로 영점 보정. 모터는 안 움직이고 서보 EEPROM만 바뀐다.
        setMiddleCalibration(motorId);
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    if (cmd == CMD_SET_REF && len >= 4) {
        // 현재 위치를 임의 각도(angle×10)로 영점 보정 (EEPROM 영구). 모터 안 움직임.
        int16_t angleX10 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
        setReferenceAngle(motorId, angleX10);
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    if (cmd == CMD_TORQUE && len >= 7) {
        setTorque(motorId, buf[6] != 0);
        sendAck(cmd, motorId, STATUS_OK);
        return;
    }

    sendAck(cmd, motorId, STATUS_ERROR);
}

void processCanRx() {
    // 한 loop에서 RX FIFO를 '비어질 때까지' 모두 비운다.
    // (이전엔 최대 3개만 읽어, 빠르게 쌓이면 FIFO(깊이 3)가 넘쳐 프레임을 놓쳤다.)
    // canRecv()는 FIFO가 비면 false를 반환하므로 자연히 멈춘다.
    // RX보다 처리속도가 빠르므로 정상이면 곧 빠져나오지만, 만일을 대비해
    // 한 loop당 처리 상한(16)을 둬서 무한루프/루프 기아를 막는다.
    uint16_t canId = 0;
    uint8_t data[8] = {};
    uint8_t len = 0;

    for (uint8_t i = 0; i < 16; i++) {
        if (!canRecv(&canId, data, &len)) return;
        handleCanFrame(canId, data, len);
    }
}

void printDiagTick() {
    uint32_t now = millis();
    if (lastDiagPrintMs != 0 && (uint32_t)(now - lastDiagPrintMs) < 1200) return;
    lastDiagPrintMs = now;

    Serial.print("[CAN] tx_no_mailbox=");
    Serial.print(canTxNoMailboxCount);
    Serial.print(" tx_add_fail=");
    Serial.println(canTxAddFailCount);
}


// ============================================================
// Arduino 진입점
// ============================================================
void setup() {
    Serial.begin(115200);
    ServoSerial.begin(1000000);
    delay(500);

    Serial.println("STM32 IK angle receiver + STS3215 controller");
    Serial.println("CAN: 125kbps / sample-point 62.5%");
    Serial.println("RX 0x100: step/angle/torque/middle/gripper, TX 0x200 ACK, 0x201 telemetry, 0x202 current");

    // 펄스폭 범위를 명시해 attach (기본범위는 MG90S 끝단에서 계속 도는 원인이 됨)
    gripper.attach(GRIPPER_PWM_PIN, GRIPPER_PULSE_MIN_US, GRIPPER_PULSE_MAX_US);
    gripper.write(GRIPPER_INIT_DEG);

    for (uint8_t id = 1; id <= SERVO_COUNT; id++) {
        setTorque(id, true);
        delay(20);
    }

    canInit();
}

void loop() {
    processCanRx();
    sendServoTelemetryTick();
    printDiagTick();
}
