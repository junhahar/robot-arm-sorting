#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""robot_logger.py — 삼보모터스 로봇팔 운전 일지 로거 (SQLite)

용도
  (1) DB 환경 구축/검증용 단독 실행 CLI
  (2) 제어 스크립트가 import 해서 쓰는 로깅 모듈

설계 요점
  - SQLite 파일 1개. WAL 모드 + 배치 커밋
    → 40Hz 모터 샘플을 메모리에 모았다가 주기적으로 한 번에 커밋(SD 마모 완화).
  - 모든 기록에 꼬리표 4종: ts / session / cycle_no / step.
  - motor_sample 60일 자동 삭제(purge_old).

CLI
  python3 robot_logger.py --init      # DB 파일 + 표 5개 생성(있으면 그대로 둠)
  python3 robot_logger.py --info      # 표 목록 / 행수 / 파일크기 / WAL 상태
  python3 robot_logger.py --selftest  # 임시 DB에 더미 기록 후 읽어서 검증(실DB 안 건드림)
  python3 robot_logger.py --purge --days 60
옵션:
  --db PATH   기본값: 환경변수 ROBOT_LOG_DB → 없으면 ~/sambo_robot/data/robot_log.db

사용 예(제어 스크립트에서):
  from robot_logger import RobotLogger
  log = RobotLogger(mode_default="auto")
  sid = log.start_session("auto")
  log.set_step("SCAN", cycle_no=1)
  log.log_motor(1, cur_angle=12.3, tgt_angle=12.0, temp=41, load=0.2, current=0.6)  # 40Hz
  log.log_sort("bolt", bin="A", success=True, yolo_conf=0.93, tof=120, cycle_sec=8.2)
  log.end_session(total_sorted=7)
  log.close()
"""

import os
import sys
import time
import argparse
import sqlite3
import threading
import tempfile
from pathlib import Path

# ----------------------------------------------------------------------
# 기본 DB 경로: 환경변수 우선, 없으면 ~/sambo_robot/data/robot_log.db
# (라파에서 home=/home/pi 이므로 /home/pi/sambo_robot/data/robot_log.db)
# ----------------------------------------------------------------------
DEFAULT_DB = os.environ.get("ROBOT_LOG_DB") or str(
    Path.home() / "sambo_robot" / "data" / "robot_log.db"
)

# robot_logger.py 옆 db/schema.sql 을 우선 사용. 없으면 아래 내장본 사용.
_SCHEMA_FILE = Path(__file__).resolve().parent / "db" / "schema.sql"

# 내장 스키마(폴백). schema.sql 과 동일하게 유지할 것.
EMBEDDED_SCHEMA = """
CREATE TABLE IF NOT EXISTS run_session (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at   REAL NOT NULL,
    ended_at     REAL,
    mode         TEXT,
    total_sorted INTEGER DEFAULT 0,
    note         TEXT
);
CREATE TABLE IF NOT EXISTS motor_sample (
    ts        REAL NOT NULL,
    session   INTEGER,
    cycle_no  INTEGER,
    step      TEXT,
    motor_id  INTEGER NOT NULL,
    cur_angle REAL,
    tgt_angle REAL,
    temp      REAL,
    load      REAL,
    current   REAL
);
CREATE TABLE IF NOT EXISTS step_event (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    ts        REAL NOT NULL,
    session   INTEGER,
    cycle_no  INTEGER,
    step      TEXT,
    prev_step TEXT
);
CREATE TABLE IF NOT EXISTS sort_event (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    session     INTEGER,
    cycle_no    INTEGER,
    object_type TEXT,
    bin         TEXT,
    success     INTEGER,
    yolo_conf   REAL,
    tof         REAL,
    corr_dx     REAL,
    corr_dy     REAL,
    cycle_sec   REAL
);
CREATE TABLE IF NOT EXISTS system_event (
    id       INTEGER PRIMARY KEY AUTOINCREMENT,
    ts       REAL NOT NULL,
    session  INTEGER,
    cycle_no INTEGER,
    step     TEXT,
    level    TEXT,
    message  TEXT
);
CREATE INDEX IF NOT EXISTS idx_motor_main   ON motor_sample(session, cycle_no, step, ts);
CREATE INDEX IF NOT EXISTS idx_step_session ON step_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sort_session ON sort_event(session, cycle_no, ts);
CREATE INDEX IF NOT EXISTS idx_sys_session  ON system_event(session, cycle_no, ts);
"""

TABLES = ("run_session", "motor_sample", "step_event", "sort_event", "system_event")


def _load_schema():
    if _SCHEMA_FILE.exists():
        return _SCHEMA_FILE.read_text(encoding="utf-8")
    return EMBEDDED_SCHEMA


class RobotLogger:
    """제어 스크립트가 들고 쓰는 로깅 도우미.

    세션/사이클/단계 컨텍스트를 내부에 보관하므로, 호출부는
    set_step()/log_motor() 등만 부르면 꼬리표가 자동으로 붙는다.
    """

    def __init__(self, db_path=DEFAULT_DB, batch_size=200, flush_interval=0.5,
                 mode_default="manual"):
        self.db_path = str(db_path)
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        # 제어 루프와 다른 스레드(플러시 등)에서 같이 써도 되게 check_same_thread=False
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False, timeout=5.0)
        self._configure()
        self._ensure_schema()

        self.batch_size = int(batch_size)
        self.flush_interval = float(flush_interval)
        self.mode_default = mode_default

        self._buf = []
        self._last_flush = time.time()
        self._lock = threading.Lock()

        # 컨텍스트(꼬리표)
        self.session_id = None
        self.cycle_no = None
        self.step = None

    # -------------------- 내부 --------------------
    def _configure(self):
        c = self.conn
        c.execute("PRAGMA journal_mode=WAL")      # 파일 헤더에 영구 기록됨
        c.execute("PRAGMA synchronous=NORMAL")    # WAL 에서 안전+빠름의 균형
        c.execute("PRAGMA busy_timeout=5000")
        c.execute("PRAGMA foreign_keys=ON")

    def _ensure_schema(self):
        self.conn.executescript(_load_schema())
        self.conn.commit()

    def _flush_locked(self):
        if not self._buf:
            return
        self.conn.executemany(
            "INSERT INTO motor_sample"
            "(ts, session, cycle_no, step, motor_id, cur_angle, tgt_angle, temp, load, current) "
            "VALUES(?,?,?,?,?,?,?,?,?,?)",
            self._buf,
        )
        self.conn.commit()
        self._buf.clear()
        self._last_flush = time.time()

    # -------------------- 세션 --------------------
    def start_session(self, mode=None, note=None):
        mode = mode or self.mode_default
        cur = self.conn.execute(
            "INSERT INTO run_session(started_at, mode, note) VALUES(?,?,?)",
            (time.time(), mode, note),
        )
        self.conn.commit()
        self.session_id = cur.lastrowid
        self.cycle_no = None
        self.step = None
        self.log_event("INFO", "session start (mode=%s)" % mode)
        return self.session_id

    def end_session(self, total_sorted=None):
        self.flush()
        if self.session_id is None:
            return None
        if total_sorted is None:
            self.conn.execute(
                "UPDATE run_session SET ended_at=? WHERE id=?",
                (time.time(), self.session_id),
            )
        else:
            self.conn.execute(
                "UPDATE run_session SET ended_at=?, total_sorted=? WHERE id=?",
                (time.time(), int(total_sorted), self.session_id),
            )
        self.log_event("INFO", "session end")
        self.conn.commit()
        sid, self.session_id = self.session_id, None
        return sid

    # -------------------- 컨텍스트 --------------------
    def set_cycle(self, cycle_no):
        self.cycle_no = cycle_no

    def set_step(self, step, cycle_no=None):
        if cycle_no is not None:
            self.cycle_no = cycle_no
        prev, self.step = self.step, step
        self.conn.execute(
            "INSERT INTO step_event(ts, session, cycle_no, step, prev_step) VALUES(?,?,?,?,?)",
            (time.time(), self.session_id, self.cycle_no, step, prev),
        )
        self.conn.commit()

    # -------------------- 기록 --------------------
    def log_motor(self, motor_id, cur_angle=None, tgt_angle=None,
                  temp=None, load=None, current=None, ts=None):
        """40Hz 호출 지점. 버퍼에 쌓다가 batch_size 또는 flush_interval 마다 커밋."""
        row = (
            time.time() if ts is None else ts,
            self.session_id, self.cycle_no, self.step,
            motor_id, cur_angle, tgt_angle, temp, load, current,
        )
        with self._lock:
            self._buf.append(row)
            if (len(self._buf) >= self.batch_size
                    or (time.time() - self._last_flush) >= self.flush_interval):
                self._flush_locked()

    def flush(self):
        with self._lock:
            self._flush_locked()

    def log_sort(self, object_type, bin=None, success=None, yolo_conf=None,
                 tof=None, corr_dx=None, corr_dy=None, cycle_sec=None):
        self.conn.execute(
            "INSERT INTO sort_event"
            "(ts, session, cycle_no, object_type, bin, success, yolo_conf, tof, corr_dx, corr_dy, cycle_sec) "
            "VALUES(?,?,?,?,?,?,?,?,?,?,?)",
            (time.time(), self.session_id, self.cycle_no, object_type, bin,
             None if success is None else int(bool(success)),
             yolo_conf, tof, corr_dx, corr_dy, cycle_sec),
        )
        self.conn.commit()

    def log_event(self, level, message):
        self.conn.execute(
            "INSERT INTO system_event(ts, session, cycle_no, step, level, message) VALUES(?,?,?,?,?,?)",
            (time.time(), self.session_id, self.cycle_no, self.step, level, message),
        )
        self.conn.commit()

    # -------------------- 유지보수 --------------------
    def purge_old(self, days=60):
        """motor_sample 중 days일 지난 것 삭제. 삭제 행수 반환."""
        cutoff = time.time() - days * 86400
        cur = self.conn.execute("DELETE FROM motor_sample WHERE ts < ?", (cutoff,))
        self.conn.commit()
        return cur.rowcount

    def counts(self):
        return {t: self.conn.execute("SELECT COUNT(*) FROM %s" % t).fetchone()[0]
                for t in TABLES}

    def close(self):
        try:
            self.flush()
        finally:
            self.conn.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ======================================================================
# CLI
# ======================================================================
def _human_size(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.1f %s" % (n, unit)
        n /= 1024.0


def cmd_info(db_path):
    p = Path(db_path)
    if not p.exists():
        print("[info] DB 파일 없음: %s  (먼저 --init)" % db_path)
        return 1
    conn = sqlite3.connect(db_path)
    journal = conn.execute("PRAGMA journal_mode").fetchone()[0]
    names = [r[0] for r in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table' "
        "AND name NOT LIKE 'sqlite_%' ORDER BY name")]
    print("DB 파일   : %s" % db_path)
    print("파일 크기 : %s" % _human_size(p.stat().st_size))
    print("journal   : %s" % journal)
    print("표(%d개)  :" % len(names))
    for t in names:
        try:
            cnt = conn.execute("SELECT COUNT(*) FROM %s" % t).fetchone()[0]
        except sqlite3.Error:
            cnt = "?"
        print("   - %-14s %s행" % (t, cnt))
    conn.close()
    missing = [t for t in TABLES if t not in names]
    if missing:
        print("[경고] 누락 표: %s" % ", ".join(missing))
        return 1
    print("OK: 표 5개 모두 존재.")
    return 0


def cmd_init(db_path):
    RobotLogger(db_path).close()
    print("[init] 생성/확인 완료 → %s" % db_path)
    return cmd_info(db_path)


def cmd_purge(db_path, days):
    if not Path(db_path).exists():
        print("[purge] DB 없음: %s" % db_path)
        return 1
    log = RobotLogger(db_path)
    n = log.purge_old(days)
    log.close()
    print("[purge] %d일 지난 motor_sample %d행 삭제" % (days, n))
    return 0


def cmd_selftest():
    """임시 DB에 더미 기록을 쓰고 읽어서 정상 동작을 확인(실 DB는 건드리지 않음)."""
    tmp = Path(tempfile.gettempdir()) / ("robot_log_selftest_%d.db" % os.getpid())
    try:
        log = RobotLogger(str(tmp), batch_size=50, flush_interval=0.2)
        sid = log.start_session("test", note="selftest")
        for cyc in range(1, 4):
            for step in ("SCAN", "GRASP", "PLACE"):
                log.set_step(step, cycle_no=cyc)
                for _ in range(20):           # 모터 6개 × 20프레임
                    for m in range(1, 7):
                        log.log_motor(m, cur_angle=float(m), tgt_angle=float(m),
                                      temp=40.0, load=0.1, current=0.5)
            log.log_sort("bolt" if cyc % 2 else "nut", bin="A", success=True,
                         yolo_conf=0.9, tof=120.0, corr_dx=0.5, corr_dy=-0.3,
                         cycle_sec=7.5)
        log.log_event("WARN", "selftest sample warning")
        log.end_session(total_sorted=3)
        c = log.counts()
        log.close()

        expected_motor = 3 * 3 * 20 * 6  # cycle * step * frame * motor = 1080
        print("self-test 결과:")
        for t in TABLES:
            print("   - %-14s %d행" % (t, c[t]))
        ok = (c["run_session"] == 1 and c["motor_sample"] == expected_motor
              and c["step_event"] == 9 and c["sort_event"] == 3
              and c["system_event"] >= 2)
        print("판정: %s (motor_sample 기대 %d)" %
              ("PASS" if ok else "FAIL", expected_motor))
        return 0 if ok else 1
    finally:
        for suffix in ("", "-wal", "-shm"):
            f = Path(str(tmp) + suffix)
            if f.exists():
                f.unlink()


def main(argv=None):
    ap = argparse.ArgumentParser(description="로봇팔 운전 일지 DB 도구")
    ap.add_argument("--db", default=DEFAULT_DB, help="DB 경로 (기본 %s)" % DEFAULT_DB)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--init", action="store_true", help="DB+표 생성")
    g.add_argument("--info", action="store_true", help="현황 출력")
    g.add_argument("--selftest", action="store_true", help="임시 DB로 동작 검증")
    g.add_argument("--purge", action="store_true", help="오래된 motor_sample 삭제")
    ap.add_argument("--days", type=int, default=60, help="--purge 보존일수(기본 60)")
    args = ap.parse_args(argv)

    if args.selftest:
        return cmd_selftest()
    if args.init:
        return cmd_init(args.db)
    if args.purge:
        return cmd_purge(args.db, args.days)
    # 기본 동작은 --info
    return cmd_info(args.db)


if __name__ == "__main__":
    sys.exit(main())
