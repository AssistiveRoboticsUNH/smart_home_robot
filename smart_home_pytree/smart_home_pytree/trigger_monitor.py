import os
import threading
import time
from datetime import datetime, timedelta

import py_trees
import yaml

# TODO: unify naming class name protocol name
#  TRIGGER MONITOR


class TriggerMonitor:
    # pylint: disable=too-many-instance-attributes
    """
    Monitors robot state and reports which protocol has its requirements satisfied, the requirements can be time or event based and they are defined in a YAML file.
    It supports periodic resets, monitoring waiting of protocols, and checking success conditions for waiting protocols.
    """

    def __init__(
        self,
        robot_interface,
        wake_event: threading.Event,
        yaml_path=None,
        debug: bool = False,
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

        if yaml_path is None:
            yaml_path = os.getenv("house_yaml_path")

        with open(yaml_path, "r") as f:
            self.protocols_yaml = yaml.safe_load(f)

        self.current_satisfied_protocols = []  # [(protocol_name, priority)]
        # Use RLock (Re-entrant Lock) to prevent deadlocks
        self.lock = threading.RLock()
        self.stop_flag = False
        self.completed_protocols = set()  # track completed ones

        self.protocols_to_reset = set()  # track ones that need resetting
        # self.protocols_to_reset should include (protocl_name, time it fines,
        # after how much time to reset)

        # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        print(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")

        self.blackboard = py_trees.blackboard.Blackboard()

        # support yield waiting
        self.pending_waits = {}
        # success_on monitoring:  support finishing a protocol if a req is met
        # full_name -> {"state": str, "value": Any}
        self.monitor_state_success = {}

        # change orchestrator to become reactive
        self.satisfied_changed_event = wake_event
        self.debug = debug

        # --- TIME SIMULATION LOGIC ---
        self.time_offset = None
        print("$$$$$$$ test time", test_time)
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

            if self.debug:
                print(
                    f"[TriggerMonitor] Time Simulation Active. Starts at {test_time} (Offset: {self.time_offset})"
                )

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
                print(f"[success_on] {full_name} satisfied ({mode})")

                self.pending_waits.pop(full_name, None)
                # self.completed_protocols.discard(full_name)
                to_remove.append(full_name)

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
        bb = py_trees.blackboard.Blackboard()
        # Ensure the key exists
        if not bb.exists("wait_requests"):
            bb.set("wait_requests", {})

        wait_requests = bb.get("wait_requests")
        if self.debug and wait_requests:
            print(
                f"[Yield wait] from collect_wait_requests got wait_requests: {wait_requests} "
            )

        for full_name, req in list(wait_requests.items()):
            seconds = req["seconds"]
            timestamp = req["timestamp"]
            print(f"[Yield wait] from full_name {full_name} got wait_requests: {req} ")
            resume_time = datetime.fromtimestamp(timestamp) + timedelta(seconds=seconds)

            # Register resume
            self.pending_waits[full_name] = resume_time
            self.completed_protocols.add(full_name)

            # register success_on if present
            # only done once
            try:
                protocol, sub = full_name.split(".")
                high_level = self.protocols_yaml["protocols"][protocol][sub][
                    "high_level"
                ]
                success_on = high_level.get("success_on", None)

                if success_on:
                    self.monitor_state_success[full_name] = self.normalize_success_on(
                        success_on
                    )
                    print(f"[success_on] Monitoring {full_name}: {success_on}")
            except Exception as e:
                print(f"[success_on] Failed to register {full_name}: {e}")

            self.recompute_satisfied()
            # Remove request so it is processed once
            del wait_requests[full_name]

    def wait_resumed(self, full_name):
        """
        Check if a waiting time for the given protocol in pending_waits dictionary has been satisfied.
        Args:
            full_name (_type_): _description_

        Returns:
            _type_: _description_
        """
        print(f"[Yield wait] {full_name}: {self.pending_waits}")
        if full_name not in self.pending_waits.keys():
            return False

        if datetime.now() >= self.pending_waits[full_name]:
            # print(f"[Yield wait] {full_name} reached time removing from pending_waits")
            del self.pending_waits[full_name]
            # can use remove but discard doest give errors
            # print(f"self.completed_protocols {full_name}")
            self.completed_protocols.discard(full_name)
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
            # We just return silently (or debug print) instead of printing an error.
            if self.debug:
                print(f"[reset] Skipped {key} (not found on blackboard).")
            return

        value = self.blackboard.get(key)

        if not isinstance(value, dict):
            print(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Reset each entry to False
        reset_dict = {sub_key: False for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        print(f"[reset] Reset {key} → {reset_dict}")

    def reset_all_protocol_dones(self):
        """
        Reset all *_done dictionaries in the py_trees blackboard.
        """
        for key, value in list(self.blackboard.storage.items()):
            # Look only for keys ending with "_done"
            if key.endswith("_done") and isinstance(value, dict):
                reset_dict = {sub_key: False for sub_key in value.keys()}
                self.blackboard.set(key, reset_dict)
                print(f"Reset {key} → {reset_dict}")

    def set_complete_specific_protocol(self, protocol_name: str):
        """
        Set the <protocol_name>_done dictionary in the py_trees blackboard to true.
        """
        key = f"{protocol_name}_done"

        # Ensure it exists
        value = self.blackboard.get(key)
        if value is None:
            print(f"[reset] No such protocol done key: {key}")
            return

        if not isinstance(value, dict):
            print(f"[reset] Key '{key}' exists but is not a dictionary.")
            return

        # Set each entry to True
        reset_dict = {sub_key: True for sub_key in value.keys()}
        self.blackboard.set(key, reset_dict)

        print(f"[reset] Reset {key} → {reset_dict}")

    # --- REQUIREMENTS CHECKS ---
    def _extract_event_keys(self):
        """Collect all event topic names used in the YAML protocols."""
        event_keys = set()
        for _, protocol_data in self.protocols_yaml.get("protocols", {}).items():
            for _, sub_data in protocol_data.items():
                high_level_subdata = sub_data["high_level"]
                reqs = high_level_subdata.get("requirements", {})
                event_reqs = reqs.get("event", [])
                for ev in event_reqs:
                    if "state" in ev:
                        event_keys.add(ev["state"])
        return sorted(event_keys)

    def _collect_current_events(self):
        """Build a dict of current event states from RobotState."""
        current_events = {}
        for key in self.event_keys:
            val = self.robot_interface.state.get(key)
            # print("val")
            # print("key: ", key, "value: ", val)
            if val is None:
                if self.debug:
                    print(
                        f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False"
                    )
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

    def check_day_requirement(self, day_req, current_day=None):
        """Check if the current day satisfies the day requirement."""
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
        """Check if all event requirements listed in the yaml are satisfied.
        Args:
            event_reqs (list): list of event requirement dicts with 'state' and 'value' keys.
            current_events (dict): current event states.
        Returns:
            bool: True if all event requirements are satisfied, False otherwise.
        """
        if not event_reqs:
            return True
        for ev in event_reqs:
            topic, val = ev["state"], ev["value"]
            if topic not in current_events or current_events[topic] != val:
                return False
        return True

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
        print("##############################################################")
        print("$$$$$$$$$$$$$$$$$$$$$ Satisfied new_satisfied event : ", new_satisfied)
        print("##############################################################")
        if (
            changed or new_satisfied
        ):  # trigger if not empty and if change happened might be redundant
            # if self.debug and changed:
            print("Satisfied change event : ", new_satisfied)
            self.satisfied_changed_event.set()

        return changed

    def satisfied_protocols(self, current_events, current_day=None):
        """Determine which protocols have their requirements satisfied.
        Args:
            current_events (dict): current event states.
            current_day (str, optional): current day name. Defaults to None which uses the actually time.

            Returns:
                list of (protocol_name, priority) tuples that are satisfied.
        """
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml["protocols"].items():
            # print("protocol_name: ", protocol_name)
            for sub_name, sub_data in protocol_data.items():
                # print("sub_name: ", sub_name)
                high_level_subdata = sub_data["high_level"]
                full_name = f"{protocol_name}.{sub_name}"

                # Yield Wait Check
                resumed = self.wait_resumed(full_name)
                if self.debug:
                    print(f"[Yield wait] resumed: {resumed}")

                # If currently waiting (and not just resumed), skip it
                print(f"[DEBUG waiting] name {full_name} pending {self.pending_waits} ")
                if full_name in self.pending_waits:
                    continue

                # Completed Check
                if (
                    full_name in self.completed_protocols
                ):  # skip completed ones ## when resumed it deletes the protocol from completed
                    continue

                if not resumed:
                    reqs = high_level_subdata.get("requirements", {})
                    event_reqs = reqs.get("event", [])
                    time_req = reqs.get("time", {})
                    day_req = reqs.get("day", [])
                    # print("$$$$$ time_req: ", time_req)

                    if (
                        self.check_day_requirement(day_req, current_day)
                        and self.check_event_requirement(event_reqs, current_events)
                        and self.check_time_requirement(time_req)
                    ):
                        priority = high_level_subdata.get("priority", 1)
                        satisfied.append((full_name, priority))
                else:
                    # resume  dont check for req thye were already satisfied
                    # modify to track keys for req resuming.
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
                print("[TriggerMonitor] New day → resetting protocol done flags.")
                self.reset_all_protocol_dones()
                last_day = today

            # resets periodic protocols in reset array
            self.check_and_reset_protocols()
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
            print(f"[reset_pattern] Invalid format: {reset_pattern}")
            return None

        total_seconds = int(hours * 3600 + minutes * 60)

        return total_seconds if total_seconds > 0 else None

    def mark_completed(self, protocol_name):
        """
        Mark a protocol as successfully completed based on its reset_pattern.

        Rules:
        1. INSTANT: Do NOT mark complete. Reset flags immediately so it can run again.
        2. PERIODIC: Mark complete. Schedule a reset based on time.
        3. NO PATTERN / OTHER: Mark complete (runs once until daily reset).
        """
        with self.lock:
            print(f"[TriggerMonitor] Marking {protocol_name} as completed.")

            # Parse Name & Validate
            try:
                main, sub = protocol_name.split(".")
            except ValueError:
                print(f"[ERROR] Invalid protocol name format: {protocol_name}")
                return

            # Retrieve YAML Config
            try:
                sub_data = self.protocols_yaml["protocols"][main][sub]
                high_level = sub_data["high_level"]
                reset_pattern = high_level.get("reset_pattern", None)  # None if missing
            except KeyError:
                print(f"[ERROR] Protocol not found in YAML: {protocol_name}")
                return

            # Determine Type
            # If no pattern exists, we treat it as "default" (run once)
            pattern_type = (
                reset_pattern.get("type", "default") if reset_pattern else "default"
            )

            # --- CASE 1: INSTANT ---
            if pattern_type == "instant":
                # Logic: Don't block triggers (completed_protocols).
                # Reset Blackboard flags so the behavior tree can tick again immediately.
                if self.debug:
                    print(
                        f"[TriggerMonitor] Type INSTANT: Resetting flags for {sub} immediately."
                    )
                self.reset_specific_protocol_dones(sub)
                return

            # --- CASE 2 & 3: Mark as Complete (Blocks re-triggering) ---
            self.completed_protocols.add(protocol_name)

            # --- CASE 2: PERIODIC ---
            if pattern_type == "periodic":
                reset_seconds = self.parse_reset_pattern(reset_pattern)
                if reset_seconds:
                    timestamp = datetime.now()
                    self.protocols_to_reset.add(
                        (protocol_name, timestamp, reset_seconds)
                    )
                    if self.debug:
                        print(
                            f"[TriggerMonitor] Type PERIODIC: Scheduled reset for {protocol_name} in {reset_seconds}s"
                        )
                else:
                    if self.debug:
                        print(
                            f"[ERROR] Periodic protocol {protocol_name} has invalid time settings."
                        )

            # --- CASE 3: DEFAULT (No Pattern) ---
            elif pattern_type == "default":
                if self.debug:
                    print(
                        f"[TriggerMonitor] Type DEFAULT: {protocol_name} marked complete (no auto-reset)."
                    )

            # Update satisfied list because one protocol just became "Complete" (blocked)
            self.recompute_satisfied()

    def check_and_reset_protocols(self):
        """Check all protocols scheduled for auto-reset and reset those whose timer expired."""
        now = datetime.now()
        to_remove = []

        for name, finished_time, reset_seconds in list(self.protocols_to_reset):
            if (now - finished_time).total_seconds() >= reset_seconds:
                print(f"[TriggerMonitor] Auto-resetting protocol: {name}")

                sub_name = name.split(".")[-1]
                self.reset_specific_protocol_dones(sub_name)

                if name in self.completed_protocols:
                    print(f"self.completed_protocols {name}")
                    self.completed_protocols.remove(name)
                else:
                    print(
                        f"[TriggerMonitor] Warning protocol {name} not in completed_protocols"
                    )

                to_remove.append((name, finished_time, reset_seconds))

        # Remove after iteration (safe)
        for entry in to_remove:
            print(f"[reset] ************ protocol to reset : {entry}")
            self.protocols_to_reset.remove(entry)

    def get_satisfied(self):
        """Get the current list of satisfied protocols in a thread-safe manner."""
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        """Stop the trigger monitoring loop."""
        self.stop_flag = True
