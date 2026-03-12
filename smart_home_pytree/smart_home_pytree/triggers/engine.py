from __future__ import annotations

import threading
import time
from datetime import datetime, timedelta

import py_trees

from smart_home_pytree.protocol_tracker import ProtocolTracker
from smart_home_pytree.protocols.loader import load_house_config_yaml
from smart_home_pytree.triggers.evaluator import (
    check_day_requirement,
    check_event_requirement,
    check_location_requirement,
    check_time_requirement,
    collect_current_events,
    event_condition_match,
    event_xy_within_point_match,
    extract_event_keys,
    normalize_success_on,
)
from smart_home_pytree.triggers.persistence import TriggerMonitorPersistence
from smart_home_pytree.triggers.scheduler import (
    get_current_time_string,
    parse_reset_pattern,
    resume_time_from_request,
)
from smart_home_pytree.utils import get_house_yaml_path


class TriggerMonitor:
    # pylint: disable=too-many-instance-attributes
    """Monitor high-level trigger conditions and expose satisfied protocols."""

    def __init__(
        self,
        robot_interface,
        wake_event: threading.Event | None = None,
        yaml_path: str | None = None,
        test_time: str = "",
    ):
        self.robot_interface = robot_interface
        self.yaml_path = yaml_path or get_house_yaml_path()
        self.protocols_yaml = load_house_config_yaml(self.yaml_path)

        self.current_satisfied_protocols = []
        self.lock = threading.RLock()
        self.stop_flag = False
        self.completed_protocols = set()
        self.protocols_to_reset = set()

        self.blackboard = py_trees.blackboard.Blackboard()
        if not self.blackboard.exists("logger"):
            raise ValueError("logger is not set up yet. Make sure it runs before trigger monitor")

        self.bb_logger = self.blackboard.get("logger")
        self.bb_logger.notify_discord("[TriggerMonitor] TriggerMonitor initialized")

        self.event_keys = self._extract_event_keys()
        self.bb_logger.info(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")

        self.pending_waits = {}
        self.monitor_state_success = {}
        self.satisfied_changed_event = wake_event or threading.Event()

        self.tracker = ProtocolTracker()
        self.persistence = TriggerMonitorPersistence(self.tracker, self.bb_logger)
        self._restore_state_from_db()

        self.time_offset = None
        self.bb_logger.debug(f"[TriggerMonitor] test_time: {test_time}")
        if test_time:
            now = datetime.now()
            t_struct = datetime.strptime(test_time, "%H:%M").time()
            target_time = now.replace(hour=t_struct.hour, minute=t_struct.minute, second=0)
            self.time_offset = target_time - now
            self.bb_logger.debug(
                f"[TriggerMonitor] Time Simulation Active. Starts at {test_time} (Offset: {self.time_offset})"
            )

    def reload_config(self, yaml_path: str | None = None):
        """Reload trigger YAML and derived trigger metadata."""
        self.yaml_path = yaml_path or get_house_yaml_path()
        self.protocols_yaml = load_house_config_yaml(self.yaml_path)
        with self.lock:
            self.current_satisfied_protocols = []
            self.completed_protocols.clear()
            self.protocols_to_reset.clear()
            self.pending_waits.clear()
            self.monitor_state_success.clear()
        self._restore_state_from_db()
        self.event_keys = self._extract_event_keys()
        self.bb_logger.info(
            f"[TriggerMonitor] Reloaded house yaml. Event keys: {self.event_keys}"
        )
        self.recompute_satisfied()

    def _sync_external_state_changes(self):
        self.persistence.sync_external_state_changes(
            completed_protocols=self.completed_protocols,
            protocols_to_reset=self.protocols_to_reset,
            pending_waits=self.pending_waits,
            monitor_state_success=self.monitor_state_success,
            lock=self.lock,
            reset_specific_protocol_dones=self.reset_specific_protocol_dones,
        )

    def _restore_state_from_db(self):
        self.persistence.restore_monitor_state(
            completed_protocols=self.completed_protocols,
            protocols_to_reset=self.protocols_to_reset,
            pending_waits=self.pending_waits,
            lock=self.lock,
        )

    def _get_stored_last_day(self) -> str | None:
        return self.persistence.get_stored_last_day()

    def _set_stored_last_day(self, day: str):
        self.persistence.set_stored_last_day(day)

    def get_current_time_string(self):
        return get_current_time_string(self.time_offset)

    def check_success_on_conditions(self):
        to_remove = []
        for full_name, rule in self.monitor_state_success.items():
            mode = rule["mode"]
            conditions = rule["conditions"]

            results = []
            for cond in conditions:
                current = self.robot_interface.state.get(cond["state"])
                results.append(current == cond["value"])

            success = all(results) if mode == "all" else any(results)
            if success:
                self.bb_logger.notify_discord(f"[success_on] {full_name} satisfied ({mode})")
                self.pending_waits.pop(full_name, None)
                to_remove.append(full_name)
                self.tracker.upsert_state(full_name, state="idle", resume_at=None)

        for name in to_remove:
            self.monitor_state_success.pop(name, None)

    def normalize_success_on(self, success_on):
        return normalize_success_on(success_on)

    def collect_wait_requests(self):
        if not self.blackboard.exists("wait_requests"):
            self.blackboard.set("wait_requests", {})

        wait_requests = self.blackboard.get("wait_requests")
        if wait_requests:
            self.bb_logger.notify_discord(
                f"[Yield wait] from collect_wait_requests got wait_requests: {wait_requests} "
            )

        for full_name, req in list(wait_requests.items()):
            seconds = req["seconds"]
            timestamp = req["timestamp"]
            resume_time = resume_time_from_request(timestamp, seconds)

            self.pending_waits[full_name] = resume_time
            self.completed_protocols.add(full_name)
            self.tracker.upsert_state(full_name, state="waiting", resume_at=resume_time)

            try:
                _runner, sub = full_name.split(".", 1)
                high_level = self.protocols_yaml["protocols"][sub]["high_level"]
                success_on = high_level.get("success_on", None)
                if success_on:
                    self.monitor_state_success[full_name] = self.normalize_success_on(success_on)
                    self.bb_logger.debug(f"[success_on] Monitoring {full_name}: {success_on}")
            except Exception as exc:
                self.bb_logger.debug(f"[success_on] Failed to register {full_name}: {exc}")

            self.recompute_satisfied()
            del wait_requests[full_name]

    def wait_resumed(self, full_name):
        self.bb_logger.debug(f"[Yield wait] {full_name}: {self.pending_waits}")
        if full_name not in self.pending_waits:
            return False

        if datetime.now() >= self.pending_waits[full_name]:
            del self.pending_waits[full_name]
            self.completed_protocols.discard(full_name)
            self.tracker.upsert_state(full_name, state="idle", resume_at=None)
            return True
        return False

    def reset_specific_protocol_dones(self, protocol_name: str):
        key = f"{protocol_name}_done"
        if not self.blackboard.exists(key):
            self.bb_logger.warn(f"[reset] Skipped {key} (not found on blackboard).")
            return

        value = self.blackboard.get(key)
        if not isinstance(value, dict):
            self.bb_logger.warn(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        reset_dict = {sub_key: False for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)
        self.bb_logger.debug(f"[reset] Reset {key} -> {reset_dict}")

    def reset_all_protocol_dones(self):
        for key, value in list(self.blackboard.storage.items()):
            if key.endswith("_done") and isinstance(value, dict):
                reset_dict = {sub_key: False for sub_key in value.keys()}
                self.blackboard.set(key, reset_dict)
                self.bb_logger.notify_discord(f"Reset {key} -> {reset_dict}")

    def set_complete_specific_protocol(self, protocol_name: str):
        key = f"{protocol_name}_done"
        value = self.blackboard.get(key)
        if value is None:
            self.bb_logger.warn(f"[reset] No such protocol done key: {key}")
            return
        if not isinstance(value, dict):
            self.bb_logger.warn(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        reset_dict = {sub_key: True for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)
        self.bb_logger.debug(f"[reset] Reset {key} -> {reset_dict}")

    def _extract_event_keys(self):
        return extract_event_keys(self.protocols_yaml)

    def _collect_current_events(self):
        return collect_current_events(self.robot_interface, self.event_keys, self.bb_logger)

    def check_location_requirement(self, permissible_locations):
        return check_location_requirement(self.robot_interface, permissible_locations, self.bb_logger)

    def check_day_requirement(self, day_req, current_day=None):
        return check_day_requirement(day_req, current_day)

    def check_time_requirement(self, time_req):
        return check_time_requirement(time_req, self.get_current_time_string())

    def check_event_requirement(self, event_reqs, current_events):
        return check_event_requirement(event_reqs, current_events, self.bb_logger)

    def _event_xy_within_point_match(self, current_value, event_rule: dict) -> bool:
        return event_xy_within_point_match(current_value, event_rule)

    def _event_condition_match(self, current_value, expected_value, op: str) -> bool:
        return event_condition_match(current_value, expected_value, op)

    def recompute_satisfied(self):
        current_events = self._collect_current_events()
        new_satisfied = self.satisfied_protocols(current_events)

        with self.lock:
            changed = new_satisfied != self.current_satisfied_protocols
            self.current_satisfied_protocols = new_satisfied

        if changed or new_satisfied:
            if changed:
                self.bb_logger.debug(f"Satisfied change event : {new_satisfied}")
            self.satisfied_changed_event.set()
        return changed

    def satisfied_protocols(self, current_events, current_day=None):
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml["protocols"].items():
            high_level_subdata = protocol_data["high_level"]
            runner = protocol_data.get("runner", "GenericProtocol")
            full_name = f"{runner}.{protocol_name}"

            resumed = self.wait_resumed(full_name)
            if full_name in self.pending_waits:
                self.bb_logger.debug(f"[TriggerMonitor] Skipping {full_name} (PENDING WAIT)")
                continue
            if full_name in self.completed_protocols:
                continue

            if not resumed:
                triggers = high_level_subdata.get("triggers", {})
                event_reqs = triggers.get("event", [])
                time_req = triggers.get("time", {})
                day_req = time_req.get("day", []) if isinstance(time_req, dict) else []
                loc_req = triggers.get("permissible_locations", [])

                if (
                    self.check_day_requirement(day_req, current_day)
                    and self.check_event_requirement(event_reqs, current_events)
                    and self.check_time_requirement(time_req)
                    and self.check_location_requirement(loc_req)
                ):
                    priority = high_level_subdata.get("priority", 1)
                    satisfied.append((full_name, priority))
            else:
                priority = high_level_subdata.get("priority", 1)
                satisfied.append((full_name, priority))

        satisfied.sort(key=lambda x: x[1])
        return satisfied

    def start_monitor(self):
        last_day = datetime.now().strftime("%Y-%m-%d")
        while not self.stop_flag:
            self.collect_wait_requests()
            self.check_success_on_conditions()

            today = datetime.now().strftime("%Y-%m-%d")
            if today != last_day:
                try:
                    summary = self.tracker.get_daily_summary(last_day)
                    self.bb_logger.notify_discord(f"[DailySummary]\n{summary}")
                except Exception as exc:
                    self.bb_logger.warn(f"[TriggerMonitor] Failed to send daily summary: {exc}")

                self.bb_logger.debug("[TriggerMonitor] New day -> resetting protocol done flags.")
                self.reset_all_protocol_dones()
                with self.lock:
                    self.completed_protocols.clear()
                self.tracker.reset_all_to_idle()
                self._set_stored_last_day(today)
                last_day = today

            self.check_and_reset_protocols()
            self._sync_external_state_changes()
            self.recompute_satisfied()
            time.sleep(0.5)

    def parse_reset_pattern(self, reset_pattern):
        return parse_reset_pattern(reset_pattern, self.bb_logger)

    def mark_completed(self, full_name):
        with self.lock:
            self.bb_logger.debug(f"[TriggerMonitor] Marking {full_name} as completed.")
            try:
                main, sub = full_name.split(".", 1)
            except ValueError:
                self.bb_logger.error(f"[ERROR] Invalid protocol name format: {full_name}")
                return

            try:
                sub_data = self.protocols_yaml["protocols"][sub]
                configured_runner = sub_data.get("runner", "GenericProtocol")
                if configured_runner != main:
                    self.bb_logger.warn(
                        f"[TriggerMonitor] Runner mismatch for {full_name}: yaml has {configured_runner}"
                    )
                high_level = sub_data["high_level"]
                reset_pattern = high_level.get("reset_pattern", None)
            except KeyError:
                self.bb_logger.error(f"[ERROR] Protocol not found in YAML: {full_name}")
                return

            pattern_type = reset_pattern.get("type", "eod") if reset_pattern else "eod"
            if pattern_type == "default":
                pattern_type = "eod"

            if pattern_type == "instant":
                self.bb_logger.debug(
                    f"[TriggerMonitor] Type INSTANT: Resetting flags for {sub} immediately."
                )
                self.reset_specific_protocol_dones(sub)
                self.tracker.upsert_state(
                    full_name,
                    state="idle",
                    reset_type="instant",
                    last_completed=datetime.now(),
                    last_status="completed",
                    eligible_at=None,
                )
                return

            self.completed_protocols.add(full_name)
            now = datetime.now()

            if pattern_type == "periodic":
                reset_seconds = self.parse_reset_pattern(reset_pattern)
                if reset_seconds:
                    timestamp = datetime.now()
                    self.protocols_to_reset.add((full_name, timestamp, reset_seconds))
                    eligible_at = now + timedelta(seconds=reset_seconds)
                    self.tracker.upsert_state(
                        full_name,
                        state="cooldown",
                        reset_type="periodic",
                        eligible_at=eligible_at,
                        last_completed=now,
                        last_status="completed",
                    )
                    self.bb_logger.debug(
                        f"[TriggerMonitor] Type PERIODIC: Scheduled reset for {full_name} in {reset_seconds}s"
                    )
                else:
                    self.bb_logger.debug(
                        f"[ERROR] Periodic protocol {full_name} has invalid time settings."
                    )
            elif pattern_type == "eod":
                tomorrow = (now + timedelta(days=1)).replace(
                    hour=0, minute=0, second=0, microsecond=0
                )
                self.tracker.upsert_state(
                    full_name,
                    state="cooldown",
                    reset_type="eod",
                    eligible_at=tomorrow,
                    last_completed=now,
                    last_status="completed",
                )
                self.bb_logger.debug(
                    f"[TriggerMonitor] Type EOD: {full_name} marked complete (until daily reset)."
                )

            self.recompute_satisfied()

    def check_and_reset_protocols(self):
        now = datetime.now()
        to_remove = []
        for name, finished_time, reset_seconds in list(self.protocols_to_reset):
            if (now - finished_time).total_seconds() >= reset_seconds:
                self.bb_logger.info(f"[TriggerMonitor] Auto-resetting protocol: {name}")
                sub_name = name.split(".", 1)[-1]
                self.reset_specific_protocol_dones(sub_name)
                if name in self.completed_protocols:
                    self.bb_logger.debug(f"removing {name} from self.completed_protocols")
                    self.completed_protocols.remove(name)
                else:
                    self.bb_logger.warn(
                        f"[TriggerMonitor] Warning protocol {name} not in completed_protocols"
                    )
                to_remove.append((name, finished_time, reset_seconds))

        for entry in to_remove:
            self.bb_logger.info(f"[TriggerMonitor] [reset] protocol to reset : {entry}")
            self.protocols_to_reset.remove(entry)
            self.tracker.reset_specific_to_idle(entry[0])

    def get_satisfied(self):
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        self.stop_flag = True
