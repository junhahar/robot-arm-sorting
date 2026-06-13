# 로봇팔 운전 일지 DB (SQLite)

로봇이 일하는 동안의 모터 움직임·단계 전환·분류 결과·시스템 사건을
한 파일에 기록해, **"몇 번째 운전 · 몇 번째 사이클 · 어느 단계에서 무슨 일이 있었나"**
를 시각순으로 다시 꺼내볼 수 있게 하는 블랙박스다.

## 파일 구성
| 파일 | 설명 |
|------|------|
| `../robot_logger.py` | 기록 도우미 모듈 + 단독 실행 CLI (제어 스크립트가 `import` 해서 사용) |
| `schema.sql` | 표 5개 설계도 (사람용 참조본 / `robot_logger.py` 내장본과 동일 유지) |

## DB 위치 · 저장 정책
- **파일 1개**: 기본 `~/sambo_robot/data/robot_log.db` (환경변수 `ROBOT_LOG_DB` 로 변경 가능)
- **현재는 SD카드**에 저장. 나중에 USB SSD 꽂으면 이 파일 하나만 옮기면 됨.
- **WAL 모드 + 배치 커밋**: 40Hz 모터 샘플을 메모리에 모았다가 `batch_size`(기본 200행)
  또는 `flush_interval`(기본 0.5초)마다 한 번에 커밋 → SD 마모/성능 부담 완화.
- **보존정책**: `motor_sample`(40Hz 원본)은 `purge_old(days=60)` 으로 60일 후 삭제 권장.
  세션/단계/분류/사건 기록은 가벼우므로 장기 보관.

## 표 5개 · 꼬리표 4종
모든 기록에 **`ts` / `session` / `cycle_no` / `step`** 꼬리표가 붙어,
"5번째 사이클 + GRASP 단계"처럼 한 번에 필터된다.

| 표 | 언제 기록 | 주요 컬럼 |
|----|-----------|-----------|
| `run_session` | 운전 켜고 끌 때 | started_at, ended_at, mode, total_sorted |
| `motor_sample` | 40Hz 연속(모터 1개당 1행) | ts, session, cycle_no, step, motor_id(1~6), cur_angle, tgt_angle, temp, load, current |
| `step_event` | 단계 전환 시 | ts, session, cycle_no, step, prev_step |
| `sort_event` | 물체 1개 완료 시 | object_type, bin, success, yolo_conf, tof, corr_dx/dy, cycle_sec |
| `system_event` | 사건 발생 시 | level(INFO/WARN/ERROR), message |

- `ts` 는 epoch 초(`time.time()`). 사람용 변환: `datetime(ts,'unixepoch','localtime')`
- 단계명: `HOME / SCAN / DETECT / PRE_GRASP / GRASP / LIFT / CARRY / PLACE`

## 사용법 — CLI
```
python3 robot_logger.py --init      # DB 파일 + 표 5개 생성(있으면 그대로 둠)
python3 robot_logger.py --info      # 표 목록 / 행수 / 파일크기 / WAL 상태
python3 robot_logger.py --selftest  # 임시 DB로 동작 검증(실 DB는 안 건드림)
python3 robot_logger.py --purge --days 60
# 경로 바꾸기: --db /path/to/robot_log.db
```
손으로 SQL 조회하려면(선택): `sudo apt install sqlite3` 후
`sqlite3 ~/sambo_robot/data/robot_log.db`.

## 사용법 — 제어 스크립트에서
```python
from robot_logger import RobotLogger

log = RobotLogger(mode_default="auto")     # DB 자동 생성/연결, WAL 설정
log.purge_old(60)                          # 부팅 시 한 번: 오래된 원본 정리
sid = log.start_session("auto")            # 운전 시작

log.set_step("SCAN", cycle_no=1)           # 단계 바뀔 때마다 한 줄
for ...:                                   # 40Hz 루프
    log.log_motor(1, cur_angle=12.3, tgt_angle=12.0,
                  temp=41, load=0.2, current=0.6)   # 모터별 호출
log.log_sort("bolt", bin="A", success=True,
             yolo_conf=0.93, tof=120, cycle_sec=8.2)  # 분류 완료
log.log_event("WARN", "M2 과열 감지")      # 특이사항

log.end_session(total_sorted=7)            # 운전 종료(버퍼 flush 포함)
log.close()
```
세션/사이클/단계 컨텍스트는 모듈이 들고 있으므로, 호출부는 `set_step`/`log_motor` 등만
부르면 꼬리표가 자동으로 붙는다. 예외 처리 위해 `with RobotLogger() as log:` 도 가능.

## 조회 예시 (sqlite3)
```sql
-- 특정 세션의 분류 실적
SELECT cycle_no, object_type, bin, success, cycle_sec FROM sort_event WHERE session=1;

-- 5번째 사이클 GRASP 단계의 모터 추이
SELECT ts, motor_id, cur_angle, temp, current
FROM motor_sample WHERE session=1 AND cycle_no=5 AND step='GRASP' ORDER BY ts;

-- 경고/에러만
SELECT datetime(ts,'unixepoch','localtime'), level, message
FROM system_event WHERE level IN ('WARN','ERROR') ORDER BY ts;
```

## 현황 (2026-06-14)
- 라파에 배포·생성 완료: `control/robot_logger.py`, `control/db/schema.sql`,
  빈 DB `data/robot_log.db` (표 5개, WAL). self-test PASS.
- **TODO**: 자동운전(`auto_sorting.py`) 루프에 `start_session`/`set_step`/`log_motor`/
  `log_sort` 호출 배선 (자동운전 구현 시 함께).
