-- ============================================================
-- 삼보모터스 로봇팔 운전 일지 (SQLite 스키마)
-- ------------------------------------------------------------
-- 표 6개. 모든 기록에 꼬리표(ts / session / cycle_no / step)를 붙여
-- "N번째 운전 · N번째 사이클 · 어느 단계"로 추적.
-- ★파이프 일생(인피드 집기→CNC 투입→회수 집기→적재)은 cycle_no로는 못 묶음
--   → part_id 로 묶는다(grip_event·sort_event 공통 키). 파이프 1개 = 같은 part_id.
-- 백엔드 단계(step, TEXT 자유값):
--   HOME/SCAN/DETECT/PIPE_TO_CNC/SORT/RETRIEVE/
--   PRE_GRASP/GRASP/LIFT/CNC_MOVE/CNC_INSERT/CNC_GRAB_MOVE/CNC_GRAB/
--   RACK_MOVE/RACK_PLACE/RETURN/GRIP_FAIL/IDLE
--   (대시보드는 STAGE_ALIAS 로 8노드에 매핑해 표시 — GRIP_CHECK 은 대시보드 파생 노드)
-- ts 는 epoch 초(REAL, time.time()). 사람용 변환: datetime(ts,'unixepoch','localtime')
--
-- ※ 이 파일은 robot_logger.py 의 EMBEDDED_SCHEMA 와 동일하게 유지(드리프트 주의).
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

-- 4) 처리 완료(분류/적재): 물체 하나 처리가 끝났을 때.
--    볼트/너트 = 분류통(dest=bolt/nut) · 파이프 = 적재함(dest=rack, part_id 로 일생 연결).
CREATE TABLE IF NOT EXISTS sort_event (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    session     INTEGER,
    cycle_no    INTEGER,
    part_id     INTEGER,                  -- ★파이프 식별(볼트/너트는 NULL)
    object_type TEXT,                     -- bolt / nut / pipe
    dest        TEXT,                     -- ★놓은 곳: bolt / nut / rack (/ bin)
    bin         TEXT,                     -- (레거시/상세) 통·적재 V홈 이름
    success     INTEGER,                  -- 1 성공 / 0 실패
    yolo_conf   REAL,                     -- YOLO 신뢰도
    tof         REAL,                     -- ToF 거리(요약값; 상세는 grip_event)
    corr_dx     REAL,                     -- 보정량 dx
    corr_dy     REAL,                     -- 보정량 dy
    cycle_sec   REAL                      -- 이 사이클 소요(초)
);

-- 5) 파지 판정(ToF) 이벤트 — ★파이프만. 인피드 집기 / CNC 회수 집기 시점에 1행.
--    연속 시계열 아님(이벤트). part_id 로 sort_event 와 조인 → "같은 파이프의 투입·회수·적재" 추적.
CREATE TABLE IF NOT EXISTS grip_event (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    ts           REAL NOT NULL,
    session      INTEGER,
    cycle_no     INTEGER,
    part_id      INTEGER,                 -- ★파이프 식별(투입↔회수↔적재 연결)
    phase        TEXT,                    -- infeed_grip(인피드 집기) / retrieve_grip(CNC 회수)
    tof_mm       REAL,                    -- 측정 거리(mm)
    threshold_mm REAL,                    -- 판정 임계(GRIP_OK_MAX, 보통 130)
    result       INTEGER                  -- 1 잡힘 / 0 놓침
);

-- 6) 시스템 사건: 모드변경 / 과열 / 경고 등 특이사항
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
-- 가벼운 표들은 사이클 필터 + 시간순. part_id 표는 일생 조인용 인덱스 추가.
-- ------------------------------------------------------------
CREATE INDEX IF NOT EXISTS idx_motor_main   ON motor_sample(session, cycle_no, step, ts);
CREATE INDEX IF NOT EXISTS idx_step_session ON step_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sort_session ON sort_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sort_part    ON sort_event(part_id);
CREATE INDEX IF NOT EXISTS idx_grip_session ON grip_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_grip_part    ON grip_event(part_id);
CREATE INDEX IF NOT EXISTS idx_sys_session  ON system_event(session, cycle_no, ts);
