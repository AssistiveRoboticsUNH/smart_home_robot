"""Persistence helpers for trigger runtime state."""

from __future__ import annotations

from datetime import datetime


class TriggerMonitorPersistence:
    def __init__(self, tracker, bb_logger):
        self.tracker = tracker
        self.bb_logger = bb_logger

    def sync_external_state_changes(
        self,
        *,
        completed_protocols,
        protocols_to_reset,
        pending_waits,
        monitor_state_success,
        lock,
        reset_specific_protocol_dones,
    ):
        all_states = self.tracker.get_all_states()
        with lock:
            for row in all_states:
                full_name = row["protocol"]
                db_state = row["state"]
                if db_state == "idle" and full_name in completed_protocols:
                    self.bb_logger.info(
                        f"[TriggerMonitor] External reset detected for {full_name} -> removing from completed"
                    )
                    completed_protocols.discard(full_name)
                    protocols_to_reset_copy = {
                        entry for entry in protocols_to_reset if entry[0] != full_name
                    }
                    protocols_to_reset.clear()
                    protocols_to_reset.update(protocols_to_reset_copy)
                    pending_waits.pop(full_name, None)
                    monitor_state_success.pop(full_name, None)
                    sub_name = full_name.split(".", 1)[-1]
                    reset_specific_protocol_dones(sub_name)

    def restore_monitor_state(self, *, completed_protocols, protocols_to_reset, pending_waits, lock):
        self.tracker.expire_stale_states()

        stored_day = self.get_stored_last_day()
        today = datetime.now().strftime("%Y-%m-%d")
        if stored_day and stored_day != today:
            self.bb_logger.debug(
                f"[TriggerMonitor] Day changed while offline ({stored_day} -> {today}). Resetting."
            )
            self.tracker.reset_all_to_idle()
            self.set_stored_last_day(today)
            return

        with lock:
            for row in self.tracker.get_protocols_in_cooldown():
                full_name = row["protocol"]
                completed_protocols.add(full_name)
                state = self.tracker.get_state(full_name)
                if state and state["reset_type"] == "periodic" and state["eligible_at"]:
                    eligible = datetime.fromisoformat(state["eligible_at"])
                    remaining = (eligible - datetime.now()).total_seconds()
                    if remaining > 0:
                        protocols_to_reset.add((full_name, datetime.now(), remaining))
                self.bb_logger.debug(f"[TriggerMonitor] Restored cooldown: {full_name}")

            for row in self.tracker.get_protocols_waiting():
                full_name = row["protocol"]
                resume_at = datetime.fromisoformat(row["resume_at"])
                pending_waits[full_name] = resume_at
                completed_protocols.add(full_name)
                self.bb_logger.debug(f"[TriggerMonitor] Restored wait: {full_name} -> {resume_at}")

        self.set_stored_last_day(today)
        self.bb_logger.info(
            f"[TriggerMonitor] Restored state: {len(completed_protocols)} completed, {len(pending_waits)} waiting"
        )

    def get_stored_last_day(self) -> str | None:
        with self.tracker._lock:
            row = self.tracker._conn.execute(
                "SELECT value FROM _tracker_meta WHERE key = 'last_day'"
            ).fetchone()
            return row["value"] if row else None

    def set_stored_last_day(self, day: str):
        with self.tracker._lock:
            self.tracker._conn.execute(
                "INSERT OR REPLACE INTO _tracker_meta (key, value) VALUES ('last_day', ?)",
                (day,),
            )
            self.tracker._conn.commit()
