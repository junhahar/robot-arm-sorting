-- ============================================================
-- 삼보모터스 로봇팔 운전 일지 (SQLite 스키마)
-- ------------------------------------------------------------
-- 테이블 5개. 모든 기록에 꼬리표 4종(ts / session / cycle_no / step)을
-- 붙여 "N번째 운전 · N번째 사이클 · 어느 단계"로 추적 가능하게 함.
-- 단계명: HOME/SCAN/DETECT/PRE_GRASP/GRASP/LIFT/CARRY/PLACE
-- ts 는 epoch 초(REAL, time.time()). 사람용 변환: datetime(ts,'unixepoch','localtime')
--
-- ※ 이 파일은 robot_logger.py 의 내장 스키마와 동일하게 유지(드리프트 주의).
--   robot_logger.py 는 이 파일이 옆에 있으면 우선 사용, 없으면 내장본 사용.
-- ============================================================

-- 1) 운전 세션: 켜고 끈 한 번의 운전
CREATE TABLE IF NOT EXISTS run_session (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at   REAL NOT NULL,          -- 시작 시각(epoch)
    ended_at     REAL,                   -- 종료 시각(epoch), 진행중이면 NULL
    mode         TEXT,                   -- auto / manual / test
    total_sorted INTEGER DEFAULT 0,      -- 이 세션에서 분류한 개수
    note         TEXT
);

-- 2) 모터 샘플(40Hz 원본): 가장 빽빽한 표. 모터 1개당 1행.
--    공간 절약 위해 별도 id 컬럼 없이 암묵 rowid 사용.
CREATE TABLE IF NOT EXISTS motor_sample (
    ts        REAL NOT NULL,
    session   INTEGER,                   -- run_session.id
    cycle_no  INTEGER,
    step      TEXT,
    motor_id  INTEGER NOT NULL,          -- 1..6
    cur_angle REAL,                      -- 현재각(deg)
    tgt_angle REAL,                      -- 목표각(deg)
    temp      REAL,                      -- 온도
    load      REAL,                      -- 부하
    current   REAL                       -- 전류
);

-- 3) 단계 전환: 집기→들기→옮기기→놓기 등으로 넘어갈 때
CREATE TABLE IF NOT EXISTS step_event (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts        REAL NOT NULL,
    session   INTEGER,
    cycle_no  INTEGER,
    step      TEXT,                       -- 새 단계
    prev_step TEXT                        -- 직전 단계(없으면 NULL)
);

-- 4) 분류 완료: 물체 하나 처리가 끝났을 때
CREATE TABLE IF NOT EXISTS sort_event (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    ts         REAL NOT NULL,
    session    INTEGER,
    cycle_no   INTEGER,
    object_type TEXT,                     -- bolt / nut ...
    bin        TEXT,                       -- 어느 분류통
    success    INTEGER,                    -- 1 성공 / 0 실패
    yolo_conf  REAL,                       -- YOLO 신뢰도
    tof        REAL,                        -- ToF 거리
    corr_dx    REAL,                        -- 보정량 dx
    corr_dy    REAL,                        -- 보정량 dy
    cycle_sec  REAL                         -- 이 사이클 소요(초)
);

-- 5) 시스템 사건: 모드변경 / 과열 / 경고 등 특이사항
CREATE TABLE IF NOT EXISTS system_event (
    id       INTEGER PRIMARY KEY AUTOINCREMENT,
    ts       REAL NOT NULL,
    session  INTEGER,
    cycle_no INTEGER,
    step     TEXT,
    level    TEXT,                          -- INFO / WARN / ERROR
    message  TEXT
);

-- ------------------------------------------------------------
-- 인덱스
-- 40Hz 표(motor_sample)는 쓰기 부담을 줄이려 인덱스 1개만:
--   "세션+사이클+단계 필터 후 시간순" 조회를 한 방에 커버.
-- ------------------------------------------------------------
CREATE INDEX IF NOT EXISTS idx_motor_main ON motor_sample(session, cycle_no, step, ts);

-- 가벼운 표들은 사이클 필터 + 시간순 조회용 인덱스(쓰기 부담 적음)
CREATE INDEX IF NOT EXISTS idx_step_session ON step_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sort_session ON sort_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sys_session  ON system_event(session, cycle_no, ts);
