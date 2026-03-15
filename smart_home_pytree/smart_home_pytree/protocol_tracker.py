"""
Persistent protocol tracker backed by SQLite.

Two tables:
  * ``protocol_log``   - immutable history of every protocol run
  * ``protocol_state``  - live per-protocol state (one row per protocol)

The database file lives at ``$SHR_USER_DIR/database/protocol_tracker.db`` by
default. This can still be overridden via the *db_path* constructor arg or the
``PROTOCOL_TRACKER_DB`` environment variable.
"""

from __future__ import annotations

import os
import sqlite3
from datetime import datetime, timedelta
from pathlib import Path
from threading import RLock
from typing import Any

from smart_home_pytree.utils import get_user_database_dir

# Sentinel object to distinguish "not provided" from None
_SENTINEL = object()


class ProtocolTracker:
    """Thread-safe SQLite persistence layer for protocol tracking."""

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __init__(self, db_path: str | None = None):
        user_db_dir = get_user_database_dir(required=True)
        default_db_path = str((user_db_dir / "protocol_tracker.db").resolve())
        resolved = db_path or os.getenv("PROTOCOL_TRACKER_DB", default_db_path)
        self._db_path = str(Path(resolved).expanduser().resolve())
        os.makedirs(os.path.dirname(self._db_path), exist_ok=True)
        self._lock = RLock()
        self._conn: sqlite3.Connection | None = None
        self._connect()
        self._migrate()

    def _connect(self):
        self._conn = sqlite3.connect(
            self._db_path,
            check_same_thread=False,
            timeout=10,
        )
        self._conn.row_factory = sqlite3.Row
        # WAL mode for concurrent reads/writes and crash safety
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.execute("PRAGMA foreign_keys=ON")

    def _migrate(self):
        """Create tables if they don't exist."""
        with self._lock:
            self._conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS protocol_log (
                    id             INTEGER PRIMARY KEY AUTOINCREMENT,
                    date           TEXT    NOT NULL,
                    protocol       TEXT    NOT NULL,
                    run_session_id TEXT    DEFAULT NULL,
                    start_time     TEXT    NOT NULL,
                    end_time       TEXT    NOT NULL,
                    status         TEXT    NOT NULL,
                    failure_reason TEXT    DEFAULT NULL,
                    detail         TEXT    DEFAULT ''
                );

                CREATE INDEX IF NOT EXISTS idx_log_date
                    ON protocol_log (date);
                CREATE INDEX IF NOT EXISTS idx_log_protocol
                    ON protocol_log (protocol);

                CREATE TABLE IF NOT EXISTS protocol_state (
                    protocol       TEXT PRIMARY KEY,
                    state          TEXT NOT NULL DEFAULT 'idle',
                    reset_type     TEXT NOT NULL DEFAULT 'eod',
                    started_at     TEXT DEFAULT NULL,
                    eligible_at    TEXT DEFAULT NULL,
                    resume_at      TEXT DEFAULT NULL,
                    last_completed TEXT DEFAULT NULL,
                    last_status    TEXT DEFAULT NULL,
                    total_steps    INTEGER DEFAULT NULL,
                    completed_step INTEGER DEFAULT 0,
                    run_session_id TEXT DEFAULT NULL,
                    updated_at     TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS _tracker_meta (
                    key   TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS protocol_completion_event (
                    id                    INTEGER PRIMARY KEY AUTOINCREMENT,
                    source_protocol       TEXT NOT NULL,
                    source_run_session_id TEXT NOT NULL,
                    status                TEXT NOT NULL,
                    completed_at          TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS protocol_completion_consumption (
                    event_id         INTEGER NOT NULL,
                    target_protocol  TEXT NOT NULL,
                    consumed_at      TEXT NOT NULL,
                    PRIMARY KEY (event_id, target_protocol),
                    FOREIGN KEY (event_id) REFERENCES protocol_completion_event(id) ON DELETE CASCADE
                );
                """
            )
            self._conn.commit()
            # --- incremental migrations for existing databases ---
            self._add_column_if_missing("protocol_log", "run_session_id", "TEXT DEFAULT NULL")
            self._add_column_if_missing("protocol_state", "total_steps", "INTEGER DEFAULT NULL")
            self._add_column_if_missing("protocol_state", "completed_step", "INTEGER DEFAULT 0")
            self._add_column_if_missing("protocol_state", "run_session_id", "TEXT DEFAULT NULL")
            self._conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_log_session ON protocol_log (run_session_id)"
            )
            self._conn.execute(
                """
                CREATE UNIQUE INDEX IF NOT EXISTS idx_completion_event_unique
                ON protocol_completion_event (source_protocol, source_run_session_id, status)
                """
            )
            self._conn.execute(
                """
                CREATE INDEX IF NOT EXISTS idx_completion_event_lookup
                ON protocol_completion_event (source_protocol, status, completed_at)
                """
            )
            self._conn.commit()

    def _add_column_if_missing(self, table: str, column: str, col_def: str):
        """Add a column to *table* if it doesn't already exist (idempotent)."""
        cur = self._conn.execute(f"PRAGMA table_info({table})")
        existing = {row[1] for row in cur.fetchall()}
        if column not in existing:
            self._conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {col_def}")
            self._conn.commit()

    def close(self):
        with self._lock:
            if self._conn:
                self._conn.close()
                self._conn = None

    # ------------------------------------------------------------------
    # _tracker_meta  – lightweight cross-process coordination
    # ------------------------------------------------------------------

    def set_meta(self, key: str, value: str | None):
        """Insert or update a tracker meta key."""
        with self._lock:
            if value is None:
                self._conn.execute("DELETE FROM _tracker_meta WHERE key = ?", (key,))
            else:
                self._conn.execute(
                    "INSERT OR REPLACE INTO _tracker_meta (key, value) VALUES (?, ?)",
                    (key, value),
                )
            self._conn.commit()

    def get_meta(self, key: str, default: str | None = None) -> str | None:
        """Return a tracker meta value or *default* when absent."""
        with self._lock:
            row = self._conn.execute(
                "SELECT value FROM _tracker_meta WHERE key = ?",
                (key,),
            ).fetchone()
            return row["value"] if row else default

    def request_config_reload(self, yaml_path: str):
        """Mark a new dashboard YAML save as pending runtime reload."""
        now_iso = datetime.now().isoformat()
        self.set_meta("config_reload_state", "pending")
        self.set_meta("config_reload_yaml_path", yaml_path)
        self.set_meta("config_reload_requested_at", now_iso)
        self.set_meta("config_reload_applied_at", None)
        self.set_meta("config_reload_error", None)

    def mark_config_reload_applied(self, yaml_path: str | None = None):
        """Mark the latest config reload request as applied."""
        self.set_meta("config_reload_state", "applied")
        if yaml_path is not None:
            self.set_meta("config_reload_yaml_path", yaml_path)
        self.set_meta("config_reload_applied_at", datetime.now().isoformat())
        self.set_meta("config_reload_error", None)

    def mark_config_reload_error(self, error: str, yaml_path: str | None = None):
        """Mark the latest config reload request as failed."""
        self.set_meta("config_reload_state", "error")
        if yaml_path is not None:
            self.set_meta("config_reload_yaml_path", yaml_path)
        self.set_meta("config_reload_error", error)

    def get_config_reload_status(self) -> dict[str, str | None]:
        """Return the current dashboard/runtime config reload handshake state."""
        return {
            "state": self.get_meta("config_reload_state"),
            "yaml_path": self.get_meta("config_reload_yaml_path"),
            "requested_at": self.get_meta("config_reload_requested_at"),
            "applied_at": self.get_meta("config_reload_applied_at"),
            "error": self.get_meta("config_reload_error"),
        }

    # ------------------------------------------------------------------
    # protocol_log  – append-only history
    # ------------------------------------------------------------------

    def log_protocol_run(
        self,
        protocol: str,
        start_time: datetime,
        end_time: datetime,
        status: str,
        run_session_id: str | None = None,
        failure_reason: str | None = None,
        detail: str = "",
    ) -> int:
        """Insert a row into ``protocol_log`` and return the new row id."""
        date_str = start_time.strftime("%Y-%m-%d")
        with self._lock:
            cur = self._conn.execute(
                """
                INSERT INTO protocol_log
                    (date, protocol, run_session_id, start_time, end_time, status, failure_reason, detail)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    date_str,
                    protocol,
                    run_session_id,
                    start_time.isoformat(),
                    end_time.isoformat(),
                    status,
                    failure_reason,
                    detail,
                ),
            )
            self._conn.commit()
            return cur.lastrowid

    def get_log_by_date(self, date_str: str | None = None) -> list[dict]:
        """Return all log rows for a given date (default: today)."""
        if date_str is None:
            date_str = datetime.now().strftime("%Y-%m-%d")
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM protocol_log WHERE date = ? ORDER BY start_time",
                (date_str,),
            ).fetchall()
            return [dict(r) for r in rows]

    def get_log_by_date_range(self, date_from: str, date_to: str) -> list[dict]:
        """Return all log rows where date is in [date_from, date_to]."""
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT * FROM protocol_log
                WHERE date BETWEEN ? AND ?
                ORDER BY start_time
                """,
                (date_from, date_to),
            ).fetchall()
            return [dict(r) for r in rows]

    def get_all_logs(self) -> list[dict]:
        """Return all log rows ordered by start time."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM protocol_log ORDER BY start_time"
            ).fetchall()
            return [dict(r) for r in rows]

    def get_log_by_protocol(self, protocol: str, limit: int = 50) -> list[dict]:
        """Return recent log rows for a specific protocol."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM protocol_log WHERE protocol = ? ORDER BY start_time DESC LIMIT ?",
                (protocol, limit),
            ).fetchall()
            return [dict(r) for r in rows]

    def _compact_entries(
        self,
        entries: list[dict],
        *,
        include_yielded: bool = False,
    ) -> list[dict]:
        """Compact raw log rows to one canonical row per run session."""
        if not entries:
            return []

        by_session: dict[str, list[dict]] = {}
        for e in entries:
            session_key = e.get("run_session_id") or f"legacy:{e['id']}"
            by_session.setdefault(session_key, []).append(e)

        compact: list[dict] = []
        terminal_statuses = {"completed", "failed", "preempted"}

        for rows in by_session.values():
            terminal = [r for r in rows if r.get("status") in terminal_statuses]
            if terminal:
                compact.append(terminal[-1])
                continue

            if include_yielded:
                yielded = [r for r in rows if r.get("status") == "yielded"]
                if yielded:
                    compact.append(yielded[-1])

        compact.sort(key=lambda r: r.get("start_time") or "")
        return compact

    def get_compact_log_by_date(
        self,
        date_str: str | None = None,
        *,
        include_yielded: bool = False,
    ) -> list[dict]:
        """Return one canonical row per run session for a date."""
        return self._compact_entries(
            self.get_log_by_date(date_str),
            include_yielded=include_yielded,
        )

    def get_compact_log_by_date_range(
        self,
        date_from: str,
        date_to: str,
        *,
        include_yielded: bool = False,
    ) -> list[dict]:
        """Return one canonical row per run session for a date range."""
        return self._compact_entries(
            self.get_log_by_date_range(date_from, date_to),
            include_yielded=include_yielded,
        )

    def get_compact_log_all(
        self,
        *,
        include_yielded: bool = False,
    ) -> list[dict]:
        """Return one canonical row per run session across all dates."""
        return self._compact_entries(
            self.get_all_logs(),
            include_yielded=include_yielded,
        )

    def get_daily_summary(self, date_str: str | None = None) -> str:
        """Return a human-readable summary of today's completions for Discord."""
        entries = self.get_compact_log_by_date(date_str)
        if not entries:
            return "No protocols ran today."

        lines = []
        for e in entries:
            icon = "✅" if e["status"] == "completed" else "❌"
            name = e["protocol"].split(".", 1)[-1] if "." in e["protocol"] else e["protocol"]
            time_str = e["end_time"].split("T")[-1][:5] if "T" in e["end_time"] else e["end_time"]
            reason = f" ({e['failure_reason']})" if e.get("failure_reason") else ""
            lines.append(f"  {icon} {name} [{e['status']}{reason}] at {time_str}")

        header = f"Protocol summary for {date_str or 'today'} ({len(entries)} runs):"
        return header + "\n" + "\n".join(lines)

    # ------------------------------------------------------------------
    # protocol_completion_event  – time-bounded follow-up trigger source
    # ------------------------------------------------------------------

    def record_protocol_completion_event(
        self,
        *,
        source_protocol: str,
        source_run_session_id: str,
        status: str,
        completed_at: datetime | str,
    ) -> int:
        """Create or reuse a protocol completion event row for a source run."""
        completed_at_iso = (
            completed_at.isoformat() if isinstance(completed_at, datetime) else str(completed_at)
        )
        with self._lock:
            self._conn.execute(
                """
                INSERT OR IGNORE INTO protocol_completion_event
                    (source_protocol, source_run_session_id, status, completed_at)
                VALUES (?, ?, ?, ?)
                """,
                (source_protocol, source_run_session_id, status, completed_at_iso),
            )
            row = self._conn.execute(
                """
                SELECT id FROM protocol_completion_event
                WHERE source_protocol = ? AND source_run_session_id = ? AND status = ?
                """,
                (source_protocol, source_run_session_id, status),
            ).fetchone()
            self._conn.commit()
            return int(row["id"])

    def get_unconsumed_protocol_completion_events(
        self,
        *,
        target_protocol: str,
        since: datetime | str | None = None,
    ) -> list[dict]:
        """Return completion events not yet consumed by *target_protocol*."""
        since_iso = (
            since.isoformat()
            if isinstance(since, datetime)
            else str(since)
            if since is not None
            else None
        )
        sql = """
            SELECT e.*
            FROM protocol_completion_event e
            LEFT JOIN protocol_completion_consumption c
              ON c.event_id = e.id AND c.target_protocol = ?
            WHERE c.event_id IS NULL
        """
        params: list[Any] = [target_protocol]
        if since_iso is not None:
            sql += " AND e.completed_at >= ?"
            params.append(since_iso)
        sql += " ORDER BY e.completed_at ASC, e.id ASC"

        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
            return [dict(r) for r in rows]

    def consume_protocol_completion_events(
        self,
        *,
        target_protocol: str,
        event_ids: list[int],
    ) -> None:
        """Mark completion events as consumed for one target protocol."""
        ids = [int(event_id) for event_id in event_ids if event_id is not None]
        if not ids:
            return

        now_iso = datetime.now().isoformat()
        with self._lock:
            self._conn.executemany(
                """
                INSERT OR IGNORE INTO protocol_completion_consumption
                    (event_id, target_protocol, consumed_at)
                VALUES (?, ?, ?)
                """,
                [(event_id, target_protocol, now_iso) for event_id in ids],
            )
            self._conn.commit()

    # ------------------------------------------------------------------
    # protocol_state  – live per-protocol state
    # ------------------------------------------------------------------

    def upsert_state(
        self,
        protocol: str,
        *,
        state: str | None = None,
        reset_type: str | None = None,
        started_at: datetime | str | None = _SENTINEL,
        eligible_at: datetime | str | None = _SENTINEL,
        resume_at: datetime | str | None = _SENTINEL,
        last_completed: datetime | str | None = _SENTINEL,
        last_status: str | None = _SENTINEL,
        total_steps: int | None = _SENTINEL,
        completed_step: int | None = _SENTINEL,
        run_session_id: str | None = _SENTINEL,
    ):
        """Create or update a row in ``protocol_state``.

        Only the fields explicitly passed are updated; others keep their
        current value.  Pass ``None`` to clear a nullable column.
        """
        now_iso = datetime.now().isoformat()

        def _dt(val):
            if val is _SENTINEL:
                return _SENTINEL  # will be skipped
            if val is None:
                return None
            if isinstance(val, datetime):
                return val.isoformat()
            return val  # already a string

        with self._lock:
            existing = self._conn.execute(
                "SELECT * FROM protocol_state WHERE protocol = ?", (protocol,)
            ).fetchone()

            if existing is None:
                # INSERT with provided values or sensible defaults
                self._conn.execute(
                    """
                    INSERT INTO protocol_state
                        (protocol, state, reset_type, started_at, eligible_at,
                         resume_at, last_completed, last_status,
                         total_steps, completed_step, run_session_id, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        protocol,
                        state or "idle",
                        reset_type or "eod",
                        _dt(started_at) if started_at is not _SENTINEL else None,
                        _dt(eligible_at) if eligible_at is not _SENTINEL else None,
                        _dt(resume_at) if resume_at is not _SENTINEL else None,
                        _dt(last_completed) if last_completed is not _SENTINEL else None,
                        last_status if last_status is not _SENTINEL else None,
                        total_steps if total_steps is not _SENTINEL else None,
                        completed_step if completed_step is not _SENTINEL else 0,
                        run_session_id if run_session_id is not _SENTINEL else None,
                        now_iso,
                    ),
                )
            else:
                # Build dynamic UPDATE for only the provided fields
                updates: list[tuple[str, Any]] = []
                if state is not None:
                    updates.append(("state", state))
                if reset_type is not None:
                    updates.append(("reset_type", reset_type))
                if started_at is not _SENTINEL:
                    updates.append(("started_at", _dt(started_at)))
                if eligible_at is not _SENTINEL:
                    updates.append(("eligible_at", _dt(eligible_at)))
                if resume_at is not _SENTINEL:
                    updates.append(("resume_at", _dt(resume_at)))
                if last_completed is not _SENTINEL:
                    updates.append(("last_completed", _dt(last_completed)))
                if last_status is not _SENTINEL:
                    updates.append(("last_status", last_status))
                if total_steps is not _SENTINEL:
                    updates.append(("total_steps", total_steps))
                if completed_step is not _SENTINEL:
                    updates.append(("completed_step", completed_step))
                if run_session_id is not _SENTINEL:
                    updates.append(("run_session_id", run_session_id))

                if not updates:
                    return  # nothing to change

                updates.append(("updated_at", now_iso))
                set_clause = ", ".join(f"{col} = ?" for col, _ in updates)
                vals = [v for _, v in updates] + [protocol]
                self._conn.execute(
                    f"UPDATE protocol_state SET {set_clause} WHERE protocol = ?",
                    vals,
                )
            self._conn.commit()

    def get_state(self, protocol: str) -> dict | None:
        """Return the state row for a single protocol, or None."""
        with self._lock:
            row = self._conn.execute(
                "SELECT * FROM protocol_state WHERE protocol = ?", (protocol,)
            ).fetchone()
            return dict(row) if row else None

    def get_all_states(self) -> list[dict]:
        """Return all protocol_state rows."""
        with self._lock:
            rows = self._conn.execute(
                "SELECT * FROM protocol_state ORDER BY protocol"
            ).fetchall()
            return [dict(r) for r in rows]

    # -- Bulk queries used by TriggerMonitor on startup --

    def get_protocols_in_cooldown(self) -> list[dict]:
        """Protocols whose cooldown has not yet expired."""
        now_iso = datetime.now().isoformat()
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT protocol, eligible_at FROM protocol_state
                WHERE state = 'cooldown' AND eligible_at > ?
                """,
                (now_iso,),
            ).fetchall()
            return [dict(r) for r in rows]

    def get_protocols_waiting(self) -> list[dict]:
        """Protocols in yield/wait whose resume_at has not elapsed."""
        now_iso = datetime.now().isoformat()
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT protocol, resume_at FROM protocol_state
                WHERE state = 'waiting' AND resume_at > ?
                """,
                (now_iso,),
            ).fetchall()
            return [dict(r) for r in rows]

    def expire_stale_states(self):
        """Transition cooldown/waiting/running rows whose timers have elapsed back to idle.

        Also transitions any 'running' protocols back to idle (they were
        interrupted by a reboot).
        """
        now_iso = datetime.now().isoformat()
        updated_at = datetime.now().isoformat()
        with self._lock:
            # Running protocols that survived a reboot → idle
            self._conn.execute(
                """
                UPDATE protocol_state
                SET state = 'idle', started_at = NULL, run_session_id = NULL, updated_at = ?
                WHERE state = 'running'
                """,
                (updated_at,),
            )
            # Expired cooldowns → idle
            self._conn.execute(
                """
                UPDATE protocol_state
                SET state = 'idle', eligible_at = NULL, run_session_id = NULL, updated_at = ?
                WHERE state = 'cooldown' AND eligible_at <= ?
                """,
                (updated_at, now_iso),
            )
            # Expired waits → idle (resume_at passed while robot was down)
            self._conn.execute(
                """
                UPDATE protocol_state
                SET state = 'idle', resume_at = NULL, run_session_id = NULL, updated_at = ?
                WHERE state = 'waiting' AND resume_at <= ?
                """,
                (updated_at, now_iso),
            )
            self._conn.commit()

    def update_step_progress(self, protocol: str, completed_step: int):
        """Set ``completed_step`` for a protocol (called by the BT behavior)."""
        now_iso = datetime.now().isoformat()
        with self._lock:
            self._conn.execute(
                """
                UPDATE protocol_state
                SET completed_step = ?, updated_at = ?
                WHERE protocol = ?
                """,
                (completed_step, now_iso, protocol),
            )
            self._conn.commit()

    def reset_all_to_idle(self):
        """Daily reset: set every protocol to idle, clear cooldown/wait fields."""
        updated_at = datetime.now().isoformat()
        with self._lock:
            self._conn.execute(
                """
                UPDATE protocol_state
                SET state = 'idle',
                    started_at = NULL,
                    eligible_at = NULL,
                    resume_at = NULL,
                    total_steps = NULL,
                    completed_step = 0,
                    run_session_id = NULL,
                    updated_at = ?
                """,
                (updated_at,),
            )
            self._conn.commit()

    def reset_specific_to_idle(self, protocol: str):
        """Reset a single protocol back to idle (e.g. after periodic cooldown expires)."""
        self.upsert_state(
            protocol,
            state="idle",
            started_at=None,
            eligible_at=None,
            resume_at=None,
            total_steps=None,
            completed_step=0,
            run_session_id=None,
        )

    def get_latest_update_time(self) -> str | None:
        """Return the most recent updated_at from protocol_state, or None."""
        with self._lock:
            row = self._conn.execute(
                "SELECT MAX(updated_at) AS latest FROM protocol_state"
            ).fetchone()
            return row["latest"] if row else None

    def get_latest_log_id(self, date_str: str | None = None) -> int:
        """Return the highest log id for a given date (default: today), or 0."""
        if date_str is None:
            date_str = datetime.now().strftime("%Y-%m-%d")
        with self._lock:
            row = self._conn.execute(
                "SELECT MAX(id) AS max_id FROM protocol_log WHERE date = ?",
                (date_str,),
            ).fetchone()
            return row["max_id"] or 0 if row else 0
