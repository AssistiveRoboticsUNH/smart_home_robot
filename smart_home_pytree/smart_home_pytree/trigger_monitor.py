import os
import yaml
import time
import threading
import py_trees        
from datetime import datetime, timedelta

## TODO: unify naming class name protocol name
#  TRIGGER MONITOR 
class TriggerMonitor:
    def __init__(self, robot_interface, wake_event: threading.Event, yaml_path=None, debug: bool = False):
        
        self.robot_interface = robot_interface
        
        if yaml_path is None:
            yaml_path = os.getenv("house_yaml_path")
        
        with open(yaml_path, "r") as f:
            self.protocols_yaml = yaml.safe_load(f)

        self.current_satisfied_protocols = []  # [(protocol_name, priority)]
        self.lock = threading.Lock()
        self.stop_flag = False
        self.completed_protocols = set()  # track completed ones
        
        self.protocols_to_reset = set()  # track ones that need resetting
        # self.protocols_to_reset should include (protocl_name, time it fines, after how much time to reset)
        
         # Dynamically collect event keys from YAML
        self.event_keys = self._extract_event_keys()
        print(f"[TriggerMonitor] Loaded event keys from YAML: {self.event_keys}")
        
        self.blackboard = py_trees.blackboard.Blackboard()
        
        ## support yield waiting
        self.pending_waits = {}
        # success_on monitoring:  support finishing a protocol if a req is met
        # full_name -> {"state": str, "value": Any}
        self.monitor_state_success = {}
        
        ## change orchestrator to become reactive 
        self.satisfied_changed_event = wake_event
        self.debug = debug
    def check_success_on_conditions(self):
        """
        whether any waiting protocol
        has reached its success_on condition.
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
                self.completed_protocols.discard(full_name)
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
            return {
                "mode": "all",
                "conditions": [success_on]
            }

        if "all" in success_on:
            return {
                "mode": "all",
                "conditions": success_on["all"]
            }

        if "any" in success_on:
            return {
                "mode": "any",
                "conditions": success_on["any"]
            }

        raise ValueError(f"Invalid success_on format: {success_on}")

    ## support yield waiting
    ### test no clash when run protocol and satisfied call it at the same time
    def collect_wait_requests(self):
        bb = py_trees.blackboard.Blackboard()
        # Ensure the key exists
        if not bb.exists("wait_requests"):
            bb.set("wait_requests", {})
            
        wait_requests = bb.get("wait_requests")
        
        print(f"[Yield wait] from collect_wait_requests got wait_requests: {wait_requests} ")
        
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
                high_level = self.protocols_yaml["protocols"][protocol][sub]["high_level"]
                success_on = high_level.get("success_on", None)

                if success_on:
                    self.monitor_state_success[full_name] = self.normalize_success_on(success_on)
                    print(f"[success_on] Monitoring {full_name}: {success_on}")
            except Exception as e:
                print(f"[success_on] Failed to register {full_name}: {e}")
            
            # Remove request so it is processed once
            del wait_requests[full_name]
    
    def wait_resumed(self, full_name):
        # print(f"[Yield wait] {full_name}: {self.pending_waits}")
        if full_name not in self.pending_waits.keys():
            return False

        if datetime.now() >= self.pending_waits[full_name]:
            # print(f"[Yield wait] {full_name} reached time removing from pending_waits")
            del self.pending_waits[full_name]
            ### can use remove but discard doest give errors
            # print(f"self.completed_protocols {full_name}") 
            self.completed_protocols.discard(full_name)
            return True

        ## check if protocol has key to check that it is done
        return False
 
    # -------- Helper functions to reset protocols --------
    def reset_specific_protocol_dones(self, protocol_name: str):
        """
        Reset the <protocol_name>_done dictionary in the py_trees blackboard.
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

  
    # -------- Helper functions to get protocols to activate --------
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
                if self.debug: print(f"[TriggerMonitor] Topic '{key}' not publishing or None → treating as False")
                current_events[key] = False
            else:
                current_events[key] = val
        return current_events

    def check_day_requirement(self, day_req, current_day=None):
        if not day_req or len(day_req) == 0:
            return True
        
        if current_day is None:
            current_day = datetime.now().strftime("%A")
        
        if current_day.lower() not in [d.lower() for d in day_req]:
            return False
       
        return True
    
    def check_time_requirement(self, time_req, current_time=None):
        if not time_req:
            return True
        if current_time is None:
            current_time = datetime.now().strftime("%H:%M")
            
        fmt = "%H:%M"
        t_from = datetime.strptime(time_req['from'], fmt)
        t_to = datetime.strptime(time_req['to'], fmt)
        t_now = datetime.strptime(current_time, fmt)
        
        return t_from <= t_now <= t_to

    def check_event_requirement(self, event_reqs, current_events):
        if not event_reqs:
            return True
        for ev in event_reqs:
            topic, val = ev['state'], ev['value']
            if topic not in current_events or current_events[topic] != val:
                return False
        return True
 
   
    def recompute_satisfied(self, current_time=None):
        current_events = self._collect_current_events()
        new_satisfied = self.satisfied_protocols(
            current_events,
            current_time=current_time
        )

        with self.lock:
            changed = new_satisfied != self.current_satisfied_protocols
            self.current_satisfied_protocols = new_satisfied

        if changed or new_satisfied: ## trigger if not empty and if change happened might be redundant
            print("Satisfied change event : ", new_satisfied)
            self.satisfied_changed_event.set()
        
        return changed
    
    # A protocol is satisfied if (normal requirements are met) OR (it has a pending wait whose resume time has passed).
    ## only called in recompute_satisfied
    def satisfied_protocols(self, current_events, current_day=None, current_time=None):
        satisfied = []
        for protocol_name, protocol_data in self.protocols_yaml['protocols'].items():
            # print("protocol_name: ", protocol_name)
            for sub_name, sub_data in protocol_data.items():
                # print("sub_name: ", sub_name)
                high_level_subdata = sub_data["high_level"]
                full_name = f"{protocol_name}.{sub_name}"
                
                # yield wait 
                resumed = self.wait_resumed(full_name)
                # print(f"[Yield wait] resumed: {self.completed_protocols}")
                if full_name in self.completed_protocols:  # skip completed ones ## when resumed it deletes the protocol from completed
                    continue
                
                if not resumed:
                    reqs = high_level_subdata.get('requirements', {})
                    event_reqs = reqs.get('event', [])
                    time_req = reqs.get('time', {})
                    day_req = reqs.get('day', [])
                    # print("$$$$$ time_req: ", time_req)
                    
                    if self.check_day_requirement(day_req, current_day) and \
                        self.check_event_requirement(event_reqs, current_events) and \
                        self.check_time_requirement(time_req, current_time):
                        
                        priority = high_level_subdata.get('priority', 1)
                        satisfied.append((full_name, priority))
                else:
                    ## resume  dont check for req thye were already satisfied
                    ## modify to track keys for req resuming.
                    priority = high_level_subdata.get('priority', 1)
                    satisfied.append((full_name, priority))
                        
        satisfied.sort(key=lambda x: x[1])
        return satisfied

    ## TODO: trigger updates when events change
    def start_monitor(self, current_time: str = None):
        last_day = datetime.now().strftime("%Y-%m-%d")
        
        while not self.stop_flag:
            self.collect_wait_requests()
            ## check if pending protocols got success_on achieved
            self.check_success_on_conditions()

            # Check for daily reset
            today = datetime.now().strftime("%Y-%m-%d")
            if today != last_day:
                print("[TriggerMonitor] New day → resetting protocol done flags.")
                self.reset_all_protocol_dones()
                last_day = today
            
            # resets periodic protocols in reset array
            self.check_and_reset_protocols()
        
            self.recompute_satisfied(current_time=current_time)

            time.sleep(1)  # small delay to avoid busy loop

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

    ## TODO: clean
    def mark_completed(self, protocol_name):
        """Mark a protocol as successfully completed to avoid retriggering."""
        with self.lock:
            print(f"[TriggerMonitor] Marking {protocol_name} as completed.")
            
            
            # ---- Split the name correctly ----
            try:
                main, sub = protocol_name.split(".")
            except ValueError:
                print(f"[ERROR] Invalid protocol name format: {protocol_name}")
                return
            
            # ---- Look up YAML entry safely ----
            try:
                sub_data = self.protocols_yaml['protocols'][main][sub]
            except KeyError:
                print(f"[ERROR] Protocol not found in YAML: {protocol_name}")
                return
        
            high_level = sub_data["high_level"]
            reset_pattern = high_level.get('reset_pattern', None)
            
            if reset_pattern is None:
                print(f"[TriggerMonitor] No reset pattern for {protocol_name}")
                self.completed_protocols.add(protocol_name)
                return

            pattern_type = reset_pattern.get("type", "periodic")  # default to periodic if not given
            # INSTANT -> DONT ADD IT TO COMPLETED
            print("pattern_type: ", pattern_type)
            if pattern_type == "instant":
                ## to enable the protocol to run after
                self.reset_specific_protocol_dones(sub)
                return
            
            self.completed_protocols.add(protocol_name)
            
            # PERIODIC 
            if pattern_type != "periodic":
                return
            
            timestamp = datetime.now()
            
            reset_seconds = self.parse_reset_pattern(reset_pattern)
            print("protocol_name: ", protocol_name, " timestamp: ", timestamp, " reset_pattern: ", reset_seconds)
            self.protocols_to_reset.add(
                (protocol_name, timestamp, reset_seconds)
            )
            print(f"[TriggerMonitor] Scheduled reset for {protocol_name} in {reset_pattern}")
            self.recompute_satisfied()

    def check_and_reset_protocols(self):
        """Check all protocols scheduled for auto-reset and reset those whose timer expired."""
        now = datetime.now()
        to_remove = []

        for (name, finished_time, reset_seconds) in list(self.protocols_to_reset):
            if (now - finished_time).total_seconds() >= reset_seconds:
                print(f"[TriggerMonitor] Auto-resetting protocol: {name}")

                sub_name = name.split(".")[-1]
                self.reset_specific_protocol_dones(sub_name)
                
                if name in self.completed_protocols: 
                    print(f"self.completed_protocols {name}") 
                    self.completed_protocols.remove(name)
                else:
                    print(f"[TriggerMonitor] Warning protocol {name} not in completed_protocols")

                to_remove.append((name, finished_time, reset_seconds))

        # Remove after iteration (safe)
        for entry in to_remove:
            print(f"[reset] ************ protocol to reset : {entry}")
            self.protocols_to_reset.remove(entry)
        
    def get_satisfied(self):
        with self.lock:
            return list(self.current_satisfied_protocols)

    def stop_monitor(self):
        self.stop_flag = True