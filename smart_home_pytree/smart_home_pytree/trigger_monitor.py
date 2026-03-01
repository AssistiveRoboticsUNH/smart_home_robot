import os
import threading
import time
from datetime import datetime, timedelta

import py_trees

from smart_home_pytree.protocol_schema import load_house_config_yaml
from smart_home_pytree.protocol_tracker import ProtocolTracker


class TriggerMonitor:
    # pylint: disable=too-many-instance-attributes
    """
    Monitors robot state and reports which protocols have their triggers satisfied.
    Triggers can be time, event, and/or location based and are defined in YAML.
    It supports periodic resets, monitoring waiting of protocols, and checking success conditions for waiting protocols.
    """

    def __init__(
        self,
        robot_interface,
        wake_event: threading.Event,
        yaml_path_key=None,
        test_time: str = "",
    ):
        """
        Initialize the TriggerMonitor.
        Args:
            robot_interface: Interface to access robot state.
            wake_event (threading.Event): Event to signal when satisfied protocols change.
            yaml_path (str, optional): Path to the protocols YAML file. Defaults to environment variable house_yaml_path.
            debug (bool, optional): Enable debug logging. Defaults to False.
        """

        self.robot_interface = robot_interface
        print(f"[TriggerMonitor]: yaml_path key {yaml_path_key}")
        if yaml_path_key is None:
            yaml_path = os.getenv("house_yaml_path")
        else:
            print(f"[TriggerMonitor]: Using yaml_path_key: {yaml_path_key}")
            yaml_path = os.getenv(yaml_path_key)

        self.protocols_yaml = load_house_config_yaml(yaml_path)

        print(f"[TriggerMonitor]: yaml_path {yaml_path}")
        self.current_satisfied_protocols = []  # [(full_name, priority)]
        # Use RLock (Re-entrant Lock) to prevent deadlocks
        self.lock = threading.RLock()
        self.stop_flag = False
        self.completed_protocols = set()  # track completed ones

        self.protocols_to_reset = set()  # track ones that need resetting
        # self.protocols_to_reset should include (protocl_name, time it fines,
        # after how much time to reset)

        self.blackboard = py_trees.blackboard.Blackboard()

        if not self.blackboard.exists("logger"):
            raise ValueError(
                f"logger is not set up yet. Make sure it runs before trigger monitor"
            )

        self.bb_logger = self.blackboard.get("logger")
        self.bb_logger.notify_discord("[TriggerMonitor] TriggerMonitor initilized")

        # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        self.bb_logger.info(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")

        # support yield waiting
        self.pending_waits = {}
        # success_on monitoring:  support finishing a protocol if a req is met
        # full_name -> {"state": str, "value": Any}
        self.monitor_state_success = {}

        # change orchestrator to become reactive
        self.satisfied_changed_event = wake_event

        # --- PERSISTENT TRACKER ---
        self.tracker = ProtocolTracker()
        self._restore_state_from_db()

        # --- TIME SIMULATION LOGIC ---
        self.time_offset = None
        self.bb_logger.debug(f"[TriggerMonitor] test_time: {test_time}")

        if test_time:
            # 1. Get real 'now'
            now = datetime.now()

            # 2. Create a datetime object for the target test_time (e.g. Today at 10:30)
            t_struct = datetime.strptime(test_time, "%H:%M").time()
            target_time = now.replace(
                hour=t_struct.hour, minute=t_struct.minute, second=0
            )

            # 3. Calculate the difference (Offset = Target - Real)
            self.time_offset = target_time - now

            self.bb_logger.debug(
                f"[TriggerMonitor] Time Simulation Active. Starts at {test_time} (Offset: {self.time_offset})"
            )

    def _sync_external_state_changes(self):
        """Detect protocols that were externally set to idle (e.g. from the webapp)
        and update the in-memory state so TriggerMonitor re-evaluates them.

        Called every iteration of the monitor loop.
        """
        all_states = self.tracker.get_all_states()
        with self.lock:
            for row in all_states:
                full_name = row["protocol"]
                db_state = row["state"]

                # If DB says idle but we still have it blocked in memory → external edit
                if db_state == "idle" and full_name in self.completed_protocols:
                    self.bb_logger.info(
                        f"[TriggerMonitor] External reset detected for {full_name} → removing from completed"
                    )
                    self.completed_protocols.discard(full_name)

                    # Remove from periodic reset schedule if present
                    self.protocols_to_reset = {
                        entry for entry in self.protocols_to_reset if entry[0] != full_name
                    }

                    # Remove from pending waits if present
                    self.pending_waits.pop(full_name, None)
                    self.monitor_state_success.pop(full_name, None)

                    # Reset blackboard _done flags so the behaviour tree can re-trigger
                    sub_name = full_name.split(".", 1)[-1]
                    self.reset_specific_protocol_dones(sub_name)

    def _restore_state_from_db(self):
        """Restore volatile tracking state from the persistent database on startup."""
        # 1. Expire anything that became stale while robot was off
        self.tracker.expire_stale_states()

        # 2. Check if day changed while robot was down
        stored_day = self._get_stored_last_day()
        today = datetime.now().strftime("%Y-%m-%d")
        if stored_day and stored_day != today:
            self.bb_logger.debug(
                f"[TriggerMonitor] Day changed while offline ({stored_day} → {today}). Resetting."
            )
            self.tracker.reset_all_to_idle()
            self._set_stored_last_day(today)
            # completed_protocols and protocols_to_reset stay empty
            return

        # 3. Restore cooldowns
        for row in self.tracker.get_protocols_in_cooldown():
            full_name = row["protocol"]
            self.completed_protocols.add(full_name)

            # Reconstruct the protocols_to_reset entry for periodic types
            st = self.tracker.get_state(full_name)
            if st and st["reset_type"] == "periodic" and st["eligible_at"]:
                eligible = datetime.fromisoformat(st["eligible_at"])
                remaining = (eligible - datetime.now()).total_seconds()
                if remaining > 0:
                    self.protocols_to_reset.add(
                        (full_name, datetime.now(), remaining)
                    )
            self.bb_logger.debug(f"[TriggerMonitor] Restored cooldown: {full_name}")

        # 4. Restore pending waits
        for row in self.tracker.get_protocols_waiting():
            full_name = row["protocol"]
            resume_at = datetime.fromisoformat(row["resume_at"])
            self.pending_waits[full_name] = resume_at
            self.completed_protocols.add(full_name)
            self.bb_logger.debug(f"[TriggerMonitor] Restored wait: {full_name} → {resume_at}")

        self._set_stored_last_day(today)
        self.bb_logger.info(
            f"[TriggerMonitor] Restored state: {len(self.completed_protocols)} completed, "
            f"{len(self.pending_waits)} waiting"
        )

    def _get_stored_last_day(self) -> str | None:
        """Read last_day from a lightweight tracker_state helper row."""
        with self.tracker._lock:
            row = self.tracker._conn.execute(
                "SELECT value FROM _tracker_meta WHERE key = 'last_day'"
            ).fetchone()
            return row["value"] if row else None

    def _set_stored_last_day(self, day: str):
        """Persist last_day so daily reset survives reboots."""
        with self.tracker._lock:
            self.tracker._conn.execute(
                "INSERT OR REPLACE INTO _tracker_meta (key, value) VALUES ('last_day', ?)",
                (day,),
            )
            self.tracker._conn.commit()

    # --- TIME HELPER ---
    def get_current_time_string(self):
        """Returns the current simulated time (HH:MM)."""
        now = datetime.now()

        if self.time_offset:
            # Add the fixed offset to the ticking real clock
            simulated_now = now + self.time_offset
            return simulated_now.strftime("%H:%M")

        return now.strftime("%H:%M")

    # --- SUCCESS ON LOGIC ---
    def check_success_on_conditions(self):
        """
        If a protocol has success_on conditions and is in the pending_waits dictionary. Check if the conditions are satisfied in the robot state.
        If satisfied, remove from pending_waits.
        """
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
                # self.completed_protocols.discard(full_name)
                to_remove.append(full_name)

                # Persist: success_on met → idle
                self.tracker.upsert_state(
                    full_name, state="idle", resume_at=None
                )

        for name in to_remove:
            self.monitor_state_success.pop(name, None)

    def normalize_success_on(self, success_on):
        """
        Normalize success_on into:
        {
            "mode": "all" | "any",
            "conditions": [{"state": str, "value": Any}, ...]
        }

        support formats:
        1. success_on:
            all:
                - {state: coffee, value: true}
                - {state: coffee_pot, value: true}
        2. success_on:
            any:
                - {state: coffee, value: true}
                - {state: tea, value: true}
        3. success_on:
            state: coffee
            value: true
        """
        # single condition (legacy)
        if "state" in success_on:
            return {"mode": "all", "conditions": [success_on]}

        if "all" in success_on:
            return {"mode": "all", "conditions": success_on["all"]}

        if "any" in success_on:
            return {"mode": "any", "conditions": success_on["any"]}

        raise ValueError(f"Invalid success_on format: {success_on}")

    # --- YIELD/WAIT LOGIC ---
    def collect_wait_requests(self):
        """
        Check the blackboard for any wait_requests and register them in pending_waits dictionary.
        The wait_requests is a dictionary on the blackboard with entries:
        full_protocol_name -> {"seconds": int, "timestamp": float}
        """
        # Ensure the key exists
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

            resume_time = datetime.fromtimestamp(timestamp) + timedelta(seconds=seconds)

            # Register resume
            self.pending_waits[full_name] = resume_time
            self.completed_protocols.add(full_name)

            # Persist wait state
            self.tracker.upsert_state(
                full_name, state="waiting", resume_at=resume_time
            )

            # register success_on if present
            # only done once
            try:
                _runner, sub = full_name.split(".", 1)
                high_level = self.protocols_yaml["protocols"][sub]["high_level"]
                success_on = high_level.get("success_on", None)

                if success_on:
                    self.monitor_state_success[full_name] = self.normalize_success_on(
                        success_on
                    )
                    self.bb_logger.debug(f"[success_on] Monitoring {full_name}: {success_on}")
            except Exception as e:
                self.bb_logger.debug(f"[success_on] Failed to register {full_name}: {e}")

            self.recompute_satisfied()
            # Remove request so it is processed once
            del wait_requests[full_name]

    def wait_resumed(self, full_name):
        """
        Check if a waiting time for the given protocol in pending_waits dictionary has been satisfied.
        Args:
            full_name (_type_): the protocol_class.protocol_name

        Returns:
            bool: if protocol should be resumed
        """

        self.bb_logger.debug(f"[Yield wait] {full_name}: {self.pending_waits}")

        if full_name not in self.pending_waits.keys():
            return False

        if datetime.now() >= self.pending_waits[full_name]:
            del self.pending_waits[full_name]
            # can use remove but discard doest give errors
            self.completed_protocols.discard(full_name)

            # Persist: wait done → back to idle
            self.tracker.upsert_state(
                full_name, state="idle", resume_at=None
            )
            return True

        # check if protocol has key to check that it is done
        return False

    # --- RESET HELPERS ---
    def reset_specific_protocol_dones(self, protocol_name: str):
        """
        Set the <protocol_name>_done dictionary in the py_trees blackboard to False.
        """
        key = f"{protocol_name}_done"

        # Check if the key exists using the blackboard API
        if not self.blackboard.exists(key):
            # It is valid for a protocol not to have a _done key like charging.
            self.bb_logger.warn(f"[reset] Skipped {key} (not found on blackboard).")
            return

        value = self.blackboard.get(key)

        if not isinstance(value, dict):
            self.bb_logger.warn(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Reset each entry to False
        reset_dict = {sub_key: False for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        self.bb_logger.debug(f"[reset] Reset {key} → {reset_dict}")

    def reset_all_protocol_dones(self):
        """
        Reset all *_done dictionaries in the py_trees blackboard.
        """
        for key, value in list(self.blackboard.storage.items()):
            # Look only for keys ending with "_done"
            if key.endswith("_done") and isinstance(value, dict):
                reset_dict = {sub_key: False for sub_key in value.keys()}
                self.blackboard.set(key, reset_dict)
                self.bb_logger.notify_discord(f"Reset {key} → {reset_dict}")

    def set_complete_specific_protocol(self, protocol_name: str):
        """
        Set the <protocol_name>_done dictionary in the py_trees blackboard to true.
        """
        key = f"{protocol_name}_done"

        # Ensure it exists
        value = self.blackboard.get(key)
        if value is None:
            self.bb_logger.warn(f"[reset] No such protocol done key: {key}")
            return

        if not isinstance(value, dict):
            self.bb_logger.warn(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Set each entry to True
        reset_dict = {sub_key: True for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        self.bb_logger.debug(f"[reset] Reset {key} → {reset_dict}")

    # --- TRIGGER CHECKS ---
    def _extract_event_keys(self):
        """Collect all event topic names used in the YAML protocols."""
        event_keys = set()
        for _, protocol_data in self.protocols_yaml.get("protocols", {}).items():
            high_level_subdata = protocol_data.get("high_level", {})
            triggers = high_level_subdata.get("triggers", {})
            event_reqs = triggers.get("event", [])
            for ev in event_reqs:
                if "state" in ev:
                    event_keys.add(ev["state"])
        return sorted(event_keys)

    def _collect_current_events(self):
        """Build a dict of current event states from RobotState."""
        current_events = {}
        for key in self.event_keys:
            val = self.robot_interface.state.get(key)

            if val is None:
                self.bb_logger.debug(
                    f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False"
                )
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

    def check_location_requirement(self, permissible_locations):
        """Check if the person is in one of the allowed locations."""
        if not permissible_locations:
            return True

        # Get the current person location from the robot state
        current_loc = self.robot_interface.state.get("person_location")

        if current_loc is None:
            self.bb_logger.debug("[TriggerMonitor] 'person_location' not found in state.")
            return False

        return current_loc in permissible_locations

    def check_day_requirement(self, day_req, current_day=None):
        """Check if the current day satisfies the day trigger (from triggers.time.day)."""
        if not day_req or len(day_req) == 0:
            return True

        if current_day is None:
            current_day = datetime.now().strftime("%A")

        if current_day.lower() not in [d.lower() for d in day_req]:
            return False

        return True

    def check_time_requirement(self, time_req):
        """Check if the current time satisfies the time requirement."""
        if not time_req:
            return True

        current_time_str = self.get_current_time_string()
        fmt = "%H:%M"
        t_from = datetime.strptime(time_req["from"], fmt)
        t_to = datetime.strptime(time_req["to"], fmt)
        t_now = datetime.strptime(current_time_str, fmt)
        return t_from <= t_now <= t_to

    def check_event_requirement(self, event_reqs, current_events):
        """Check if all event trigger conditions listed in YAML are satisfied.
        Args:
            event_reqs (list): list of event trigger dicts with 'state', 'value', and optional 'op'.
            current_events (dict): current event states.
        Returns:
            bool: True if all event trigger conditions are satisfied, False otherwise.
        """
        if not event_reqs:
            return True
        for ev in event_reqs:
            topic = ev["state"]
            if topic not in current_events:
                return False
            if topic == "robot_location_xy":
                if not self._event_xy_within_point_match(current_events[topic], ev):
                    return False
                continue

            expected = ev["value"]
            op = ev.get("op", "=")
            if not self._event_condition_match(current_events[topic], expected, op):
                return False
        return True

    def _event_xy_within_point_match(self, current_value, event_rule: dict) -> bool:
        """Evaluate proximity trigger for robot_location_xy against a target point."""
        if (
            not isinstance(current_value, (tuple, list))
            or len(current_value) < 2
            or not isinstance(current_value[0], (int, float))
            or not isinstance(current_value[1], (int, float))
        ):
            return False

        within_m = event_rule.get("within_m")
        point_xy = event_rule.get("point_xy")
        if (
            isinstance(within_m, bool)
            or not isinstance(within_m, (int, float))
            or within_m <= 0
        ):
            return False
        if (
            not isinstance(point_xy, (tuple, list))
            or len(point_xy) != 2
            or isinstance(point_xy[0], bool)
            or isinstance(point_xy[1], bool)
            or not isinstance(point_xy[0], (int, float))
            or not isinstance(point_xy[1], (int, float))
        ):
            return False
        cx, cy = float(current_value[0]), float(current_value[1])
        threshold_sq = float(within_m) ** 2
        tx, ty = float(point_xy[0]), float(point_xy[1])
        dx = cx - tx
        dy = cy - ty
        return (dx * dx + dy * dy) <= threshold_sq

    def _event_condition_match(self, current_value, expected_value, op: str) -> bool:
        """Evaluate an event trigger comparison operator."""
        if op == "=":
            return current_value == expected_value
        if op == "!=":
            return current_value != expected_value

        # Numeric comparisons only.
        if (
            isinstance(current_value, bool)
            or isinstance(expected_value, bool)
            or not isinstance(current_value, (int, float))
            or not isinstance(expected_value, (int, float))
        ):
            return False

        if op == ">":
            return current_value > expected_value
        if op == ">=":
            return current_value >= expected_value
        if op == "<":
            return current_value < expected_value
        if op == "<=":
            return current_value <= expected_value

        self.bb_logger.debug(f"[TriggerMonitor] Unsupported event operator '{op}'")
        return False

    # --- SATISFIED LOGIC ---
    def recompute_satisfied(self):
        """Recompute the list of satisfied protocols based on current events, day, and time.
        Returns:
            bool: True if the set of satisfied protocols changed, False otherwise.
        """
        current_events = self._collect_current_events()
        new_satisfied = self.satisfied_protocols(current_events)

        with self.lock:
            changed = new_satisfied != self.current_satisfied_protocols
            self.current_satisfied_protocols = new_satisfied

        if (
            changed or new_satisfied
        ):  # trigger if not empty and if change happened might be redundant
            if changed:
                self.bb_logger.debug(f"Satisfied change event : {new_satisfied}")
            self.satisfied_changed_event.set()

        return changed

    def satisfied_protocols(self, current_events, current_day=None):
        """Determine which protocols have their triggers satisfied.
        Args:
            current_events (dict): current event states.
            current_day (str, optional): current day name. Defaults to None which uses the actually time.

            Returns:
                list of (protocol_name, priority) tuples that are satisfied.
        """
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml["protocols"].items():
            high_level_subdata = protocol_data["high_level"]
            runner = protocol_data.get("runner", "GenericProtocol")
            full_name = f"{runner}.{protocol_name}"

            # Yield Wait Check
            resumed = self.wait_resumed(full_name)

            # If currently waiting (and not just resumed), skip it
            if full_name in self.pending_waits:
                self.bb_logger.debug(f"[TriggerMonitor] Skipping {full_name} (PENDING WAIT)")
                continue

            # Completed Check
            if (
                full_name in self.completed_protocols
            ):  # skip completed ones ## when resumed it deletes the protocol from completed
                continue

            if not resumed:
                triggers = high_level_subdata.get("triggers", {})
                event_reqs = triggers.get("event", [])
                time_req = triggers.get("time", {})
                # Day is nested under triggers.time.day
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
                # resume  dont check for req thye were already satisfied
                priority = high_level_subdata.get("priority", 1)
                satisfied.append((full_name, priority))

        satisfied.sort(key=lambda x: x[1])
        return satisfied

    # --- LOOP ---
    def start_monitor(self):
        """
        Method to be invoked in a separate thread to monitor triggers.
        It periodically checks for trigger conditions and updates the satisfied protocols. Also handles daily resets, and finishing waiting protocols with success_on conditions met.
        """
        last_day = datetime.now().strftime("%Y-%m-%d")

        while not self.stop_flag:
            self.collect_wait_requests()
            # check if pending protocols got success_on achieved
            self.check_success_on_conditions()

            # Check for daily reset
            today = datetime.now().strftime("%Y-%m-%d")
            if today != last_day:
                # Send yesterday's summary to Discord before clearing
                try:
                    summary = self.tracker.get_daily_summary(last_day)
                    self.bb_logger.notify_discord(f"[DailySummary]\n{summary}")
                except Exception as e:
                    self.bb_logger.warn(f"[TriggerMonitor] Failed to send daily summary: {e}")

                self.bb_logger.debug("[TriggerMonitor] New day → resetting protocol done flags.")
                self.reset_all_protocol_dones()
                with self.lock:
                    self.completed_protocols.clear()
                self.tracker.reset_all_to_idle()
                self._set_stored_last_day(today)
                last_day = today

            # resets periodic protocols in reset array
            self.check_and_reset_protocols()
            # Detect external state edits (e.g. webapp set-idle)
            self._sync_external_state_changes()
            self.recompute_satisfied()
            time.sleep(0.5)  # small delay to avoid busy loop

    # --- MARK COMPLETE & RESET ---
    def parse_reset_pattern(self, reset_pattern):
        """
        Convert a reset_pattern dict into total seconds.
        Safe for missing fields.
        Examples:
            {"hours": 1, "minutes": 30} → 5400
            {"minutes": 30}             → 1800
            {"hours": 1}                → 3600
            None or {}                  → None
        """
        if not reset_pattern:
            return None

        hours = reset_pattern.get("hours", 0)
        minutes = reset_pattern.get("minutes", 0)

        # Validate types
        if not isinstance(hours, (int, float)) or not isinstance(minutes, (int, float)):
            self.bb_logger.debug(f"[reset_pattern] Invalid format: {reset_pattern}")
            return None

        total_seconds = int(hours * 3600 + minutes * 60)

        return total_seconds if total_seconds > 0 else None

    def mark_completed(self, full_name):
        """
        Mark a protocol as successfully completed based on its reset_pattern.

        Rules:
        1. INSTANT: Do NOT mark complete. Reset flags immediately so it can run again.
        2. PERIODIC: Mark complete. Schedule a reset based on time.
        3. EOD / NO PATTERN: Mark complete (runs once until end-of-day/daily reset).
        """
        with self.lock:
            self.bb_logger.debug(f"[TriggerMonitor] Marking {full_name} as completed.")

            # Parse Name & Validate
            try:
                main, sub = full_name.split(".", 1)
            except ValueError:
                self.bb_logger.error(f"[ERROR] Invalid protocol name format: {full_name}")
                return

            # Retrieve YAML Config
            try:
                sub_data = self.protocols_yaml["protocols"][sub]
                configured_runner = sub_data.get("runner", "GenericProtocol")
                if configured_runner != main:
                    self.bb_logger.warn(
                        f"[TriggerMonitor] Runner mismatch for {full_name}: yaml has {configured_runner}"
                    )
                high_level = sub_data["high_level"]
                reset_pattern = high_level.get("reset_pattern", None)  # None if missing
            except KeyError:
                self.bb_logger.error(f"[ERROR] Protocol not found in YAML: {full_name}")
                return

            # Determine Type
            # If no pattern exists, treat it as end-of-day (daily reset).
            pattern_type = reset_pattern.get("type", "eod") if reset_pattern else "eod"
            if pattern_type == "default":
                # Backward-compatible alias.
                pattern_type = "eod"

            # --- CASE 1: INSTANT ---
            if pattern_type == "instant":
                # Logic: Don't block triggers (completed_protocols).
                # Reset Blackboard flags so the behavior tree can tick again immediately.
                self.bb_logger.debug(
                    f"[TriggerMonitor] Type INSTANT: Resetting flags for {sub} immediately."
                )
                self.reset_specific_protocol_dones(sub)
                # Persist: instant goes straight back to idle
                self.tracker.upsert_state(
                    full_name,
                    state="idle",
                    reset_type="instant",
                    last_completed=datetime.now(),
                    last_status="completed",
                    eligible_at=None,
                )
                return

            # --- CASE 2 & 3: Mark as Complete (Blocks re-triggering) ---
            self.completed_protocols.add(full_name)
            now = datetime.now()

            # --- CASE 2: PERIODIC ---
            if pattern_type == "periodic":
                reset_seconds = self.parse_reset_pattern(reset_pattern)
                if reset_seconds:
                    timestamp = datetime.now()
                    self.protocols_to_reset.add(
                        (full_name, timestamp, reset_seconds)
                    )
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

            # --- CASE 3: EOD (No Pattern / daily reset) ---
            elif pattern_type == "eod":
                # eligible_at = midnight tonight
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

            # Update satisfied list because one protocol just became "Complete" (blocked)
            self.recompute_satisfied()

    def check_and_reset_protocols(self):
        """Check all protocols scheduled for auto-reset and reset those whose timer expired."""
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

        # Remove after iteration (safe)
        for entry in to_remove:
            self.bb_logger.info(f"[TriggerMonitor] [reset] protocol to reset : {entry}")
            self.protocols_to_reset.remove(entry)
            # Persist: cooldown expired → idle
            self.tracker.reset_specific_to_idle(entry[0])

    def get_satisfied(self):
        """Get the current list of satisfied protocols in a thread-safe manner."""
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        """Stop the trigger monitoring loop."""
        self.stop_flag = True
