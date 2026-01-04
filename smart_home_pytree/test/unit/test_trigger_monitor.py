import time
from datetime import datetime, timedelta
from unittest.mock import MagicMock, mock_open, patch

import py_trees
import pytest
import yaml

# Assuming the class is stored in trigger_monitor.py
from smart_home_pytree.trigger_monitor import TriggerMonitor

# --- Mock Data & Fixtures ---

# UPDATED YAML: All protocols now have at least one requirement.
TEST_YAML_CONTENT = """
protocols:
  HighPrioProtocol:
    sub1:
      high_level:
        priority: 1
        requirements:
          event:
            - {state: 'sensor_high', value: True}
  
  LowPrioProtocol:
    sub1:
      high_level:
        priority: 10
        requirements:
          event:
            - {state: 'sensor_low', value: True}

  TimeOnlyProtocol:
    sub1:
      high_level:
        priority: 5
        requirements:
          time: {from: '08:00', to: '12:00'}
          day: ['Monday', 'Friday']

  ComplexSuccessProtocol:
    sub1:
      high_level:
        priority: 2
        # success_on is used AFTER the protocol starts and yields (waits)
        success_on:
          any:
            - {state: 'button_a', value: True}
            - {state: 'button_b', value: True}
        # Requirement to START the protocol initially
        requirements:
            event: 
              - {state: 'start_trigger', value: True}
"""


@pytest.fixture
def mock_robot():
    """Mocks the RobotInterface and its state dictionary."""
    interface = MagicMock()
    interface.state = MagicMock()
    # Default behavior: state.get returns None
    interface.state.get.return_value = None
    return interface


@pytest.fixture
def monitor(mock_robot):
    """Initializes TriggerMonitor with mocked YAML and dependencies."""
    # Patch open to read our test YAML string
    with patch("builtins.open", mock_open(read_data=TEST_YAML_CONTENT)):
        with patch("yaml.safe_load", return_value=yaml.safe_load(TEST_YAML_CONTENT)):
            with patch("os.getenv", return_value="dummy/path"):
                # Clear blackboard before each test to ensure isolation
                py_trees.blackboard.Blackboard().storage.clear()

                mon = TriggerMonitor(mock_robot, yaml_path="dummy.yaml")
                return mon


# --- Tests ---


class TestTriggerMonitorCapabilities:
    # 1. TEST FORMATS AVAILABLE & PARSING
    def test_extract_event_keys(self, monitor):
        """Test if event keys are correctly extracted from YAML."""
        # Defined in YAML requirements: sensor_high, sensor_low, start_trigger
        assert "sensor_high" in monitor.event_keys
        assert "sensor_low" in monitor.event_keys
        assert "start_trigger" in monitor.event_keys

    def test_normalize_success_on_formats(self, monitor):
        """Test valid formats: Single dict, 'all' list, 'any' list."""

        # Case 1: Legacy Single Dict
        single = {"state": "foo", "value": True}
        norm_single = monitor.normalize_success_on(single)
        assert norm_single["mode"] == "all"
        assert norm_single["conditions"] == [single]

        # Case 2: 'All' List
        all_cond = {"all": [{"state": "A", "value": 1}, {"state": "B", "value": 2}]}
        norm_all = monitor.normalize_success_on(all_cond)
        assert norm_all["mode"] == "all"
        assert len(norm_all["conditions"]) == 2

        # Case 3: 'Any' List
        any_cond = {"any": [{"state": "X", "value": 1}, {"state": "Y", "value": 2}]}
        norm_any = monitor.normalize_success_on(any_cond)
        assert norm_any["mode"] == "any"

    def test_parse_reset_pattern_formats(self, monitor):
        """Test time parsing for reset patterns."""
        assert monitor.parse_reset_pattern({"hours": 1}) == 3600
        assert monitor.parse_reset_pattern({"minutes": 30}) == 1800
        assert monitor.parse_reset_pattern({"hours": 1, "minutes": 1}) == 3660
        assert monitor.parse_reset_pattern({}) is None
        assert monitor.parse_reset_pattern(None) is None

    # 2. REQUIREMENTS TRIGGERED WITH TIME, EVENT, OR BOTH
    def test_check_time_requirement(self, monitor):
        req = {"from": "09:00", "to": "17:00"}

        # Inside range
        assert monitor.check_time_requirement(req, current_time="12:00") is True
        # Boundary start
        assert monitor.check_time_requirement(req, current_time="09:00") is True
        # Boundary end
        assert monitor.check_time_requirement(req, current_time="17:00") is True
        # Outside range
        assert monitor.check_time_requirement(req, current_time="18:00") is False

    def test_check_day_requirement(self, monitor):
        req = ["Monday", "Wednesday"]

        assert monitor.check_day_requirement(req, current_day="Monday") is True
        assert (
            monitor.check_day_requirement(req, current_day="monday") is True
        )  # Case insensitive check
        assert monitor.check_day_requirement(req, current_day="Tuesday") is False

    def test_check_event_requirement(self, monitor):
        reqs = [{"state": "door", "value": "open"}]

        # Match
        current_events = {"door": "open", "light": "off"}
        assert monitor.check_event_requirement(reqs, current_events) is True

        # Mismatch value
        current_events_bad = {"door": "closed"}
        assert monitor.check_event_requirement(reqs, current_events_bad) is False

        # Missing key
        current_events_missing = {"light": "off"}
        assert monitor.check_event_requirement(reqs, current_events_missing) is False

    # 3. WHAT PROTOCOLS ARE SATISFIED AND CORRECT PRIORITY
    def test_satisfied_protocols_priority(self, monitor):
        """
        Verify that if multiple protocols are satisfied, they are returned
        sorted by priority (ascending integer: 1 is higher than 10).
        """
        # Simulate conditions where both High (prio 1) and Low (prio 10) are met
        events = {"sensor_high": True, "sensor_low": True, "start_trigger": False}

        satisfied = monitor.satisfied_protocols(events)

        # Expecting [(HighPrioProtocol.sub1, 1), (LowPrioProtocol.sub1, 10)]
        assert len(satisfied) >= 2
        names = [x[0] for x in satisfied]
        priorities = [x[1] for x in satisfied]

        # Ensure 'HighPrioProtocol.sub1' comes before 'LowPrioProtocol.sub1'
        assert "HighPrioProtocol.sub1" in names
        assert "LowPrioProtocol.sub1" in names
        assert names.index("HighPrioProtocol.sub1") < names.index(
            "LowPrioProtocol.sub1"
        )
        assert priorities == sorted(priorities)

    def test_satisfied_protocols_enforces_requirements(self, monitor):
        """
        Ensure ComplexSuccessProtocol does NOT start if 'start_trigger' is False,
        even if it has a success_on block defined.
        """
        # Case 1: Trigger is False -> Should not start
        events_false = {"start_trigger": False}
        satisfied_false = monitor.satisfied_protocols(events_false)
        satisfied_names_false = [x[0] for x in satisfied_false]
        assert "ComplexSuccessProtocol.sub1" not in satisfied_names_false

        # Case 2: Trigger is True -> Should start
        events_true = {"start_trigger": True}
        satisfied_true = monitor.satisfied_protocols(events_true)
        satisfied_names_true = [x[0] for x in satisfied_true]
        assert "ComplexSuccessProtocol.sub1" in satisfied_names_true

    # 4. CORRECTLY ADD PROTOCOLS TO WAIT PENDING
    def test_collect_wait_requests(self, monitor):
        """Test picking up wait requests from Blackboard and moving to pending_waits."""
        bb = py_trees.blackboard.Blackboard()

        # Create a wait request
        now_ts = datetime.now().timestamp()
        req_data = {"ComplexSuccessProtocol.sub1": {"seconds": 10, "timestamp": now_ts}}
        bb.set("wait_requests", req_data)

        # Run collection
        monitor.collect_wait_requests()

        # Check if removed from blackboard
        assert "ComplexSuccessProtocol.sub1" not in bb.get("wait_requests")

        # Check if added to pending_waits
        assert "ComplexSuccessProtocol.sub1" in monitor.pending_waits

        # Check if added to completed_protocols (as per logic to prevent re-trigger during wait)
        assert "ComplexSuccessProtocol.sub1" in monitor.completed_protocols

        # Check if success_on monitoring started
        assert "ComplexSuccessProtocol.sub1" in monitor.monitor_state_success

    # 5. CORRECTLY REMOVE PROTOCOLS FROM WAIT PENDING (TIME EXPIRY)
    def test_wait_resumed_by_time(self, monitor):
        name = "TestProtocol.test_sub"

        # Add a wait that expired 5 seconds ago
        past_time = datetime.now() - timedelta(seconds=5)
        monitor.pending_waits[name] = past_time
        monitor.completed_protocols.add(name)

        # Check resume
        is_resumed = monitor.wait_resumed(name)

        assert is_resumed is True
        assert name not in monitor.pending_waits
        assert (
            name not in monitor.completed_protocols
        )  # Should be removed so it can run again

    # 6. CORRECTLY REMOVE PROTOCOL W SUCCESS ON (SUCCESS CONDITION MET)
    def test_check_success_on_conditions_met(self, monitor):
        name = "ComplexSuccessProtocol.sub1"

        # Setup: Protocol is waiting.
        # NOTE: We don't need 'start_trigger' to be True here because the protocol
        # is already 'inside' the system (in pending_waits)
        monitor.pending_waits[name] = datetime.now() + timedelta(
            minutes=10
        )  # Future time
        monitor.completed_protocols.add(name)

        # Manually register the success criteria
        monitor.monitor_state_success[name] = {
            "mode": "any",
            "conditions": [
                {"state": "button_a", "value": True},
                {"state": "button_b", "value": True},
            ],
        }

        # Mock Robot State: button_b is True
        def side_effect(key):
            if key == "button_b":
                return True
            return False

        monitor.robot_interface.state.get.side_effect = side_effect

        # Action
        monitor.check_success_on_conditions()

        # Assertions
        assert name not in monitor.pending_waits
        assert name not in monitor.completed_protocols  # Should be cleared to run
        assert name not in monitor.monitor_state_success  # Should stop monitoring

    # 7. CORRECTLY SUPPORTS FORMATS FOR DIFFERNT SUCCESS ON (AND/OR Logic)
    def test_success_on_logic_all_vs_any(self, monitor):
        # --- TEST ANY ---
        name_any = "any_test"
        monitor.monitor_state_success[name_any] = {
            "mode": "any",
            "conditions": [
                {"state": "A", "value": True},
                {"state": "B", "value": True},
            ],
        }
        monitor.pending_waits[name_any] = datetime.now() + timedelta(seconds=100)

        # Robot State: A=False, B=True -> Should Succeed
        monitor.robot_interface.state.get.side_effect = (
            lambda k: True if k == "B" else False
        )
        monitor.check_success_on_conditions()
        assert name_any not in monitor.pending_waits

        # --- TEST ALL ---
        name_all = "all_test"
        monitor.monitor_state_success[name_all] = {
            "mode": "all",
            "conditions": [
                {"state": "A", "value": True},
                {"state": "B", "value": True},
            ],
        }
        monitor.pending_waits[name_all] = datetime.now() + timedelta(seconds=100)

        # Robot State: A=True, B=False -> Should FAIL (stay in wait)
        monitor.robot_interface.state.get.side_effect = (
            lambda k: True if k == "A" else False
        )
        monitor.check_success_on_conditions()
        assert name_all in monitor.pending_waits

        # Robot State: A=True, B=True -> Should SUCCEED
        monitor.robot_interface.state.get.side_effect = lambda k: True
        monitor.check_success_on_conditions()
        assert name_all not in monitor.pending_waits

    # 8. CORRECTLY RESET PROTOCOLS (PERIODIC)
    def test_check_and_reset_protocols(self, monitor):
        name = "HighPrioProtocol.sub1"
        sub_name = "sub1"
        # Simulate a protocol that finished 1 hour ago, reset time is 30 mins (1800s)
        finished_time = datetime.now() - timedelta(hours=1)
        reset_seconds = 1800

        monitor.protocols_to_reset.add((name, finished_time, reset_seconds))
        monitor.completed_protocols.add(name)

        # Blackboard setup to verify the boolean flag reset
        monitor.blackboard.set(f"{sub_name}_done", {"some_key": True})

        # Action
        monitor.check_and_reset_protocols()

        # Assertions
        assert len(monitor.protocols_to_reset) == 0
        assert name not in monitor.completed_protocols
        bb_val = monitor.blackboard.get(f"{sub_name}_done")
        assert bb_val["some_key"] is False

    def test_mark_completed_instant_vs_periodic(self, monitor):
        """Test that mark_completed handles 'instant' and 'periodic' correctly based on mocked YAML."""

        # Dynamically inject a protocol definition into the internal dict for this test
        monitor.protocols_yaml["protocols"]["ManualTest"] = {
            "periodic_sub": {
                "high_level": {"reset_pattern": {"type": "periodic", "minutes": 10}}
            },
            "instant_sub": {"high_level": {"reset_pattern": {"type": "instant"}}},
        }

        # Test Periodic
        monitor.mark_completed("ManualTest.periodic_sub")
        assert "ManualTest.periodic_sub" in monitor.completed_protocols
        # Should be added to reset queue
        monitor.mark_completed("ManualTest.instant_sub")
        assert "ManualTest.instant_sub" not in monitor.completed_protocols

    def test_normalize_success_on_formats(self, monitor):
        """Verifies the 3 supported formats for success_on."""
        # Format 1: Legacy single dict
        f1 = monitor.normalize_success_on({"state": "S", "value": "V"})
        assert f1["mode"] == "all"

        # Format 2: 'all' key
        f2 = monitor.normalize_success_on({"all": [{"state": "S", "value": "V"}]})
        assert f2["mode"] == "all"

        # Format 3: 'any' key
        f3 = monitor.normalize_success_on({"any": [{"state": "S", "value": "V"}]})
        assert f3["mode"] == "any"

    ## 2. Yield & Pending Wait Lifecycle
    def test_wait_pending_cycle(self, monitor):
        """Tests adding to pending, monitoring success_on, and removing."""
        bb = py_trees.blackboard.Blackboard()
        name = "ComplexSuccessProtocol.sub1"

        # 1. Add to Blackboard (simulate robot yielding)
        bb.set(
            "wait_requests",
            {name: {"seconds": 60, "timestamp": datetime.now().timestamp()}},
        )
        monitor.collect_wait_requests()

        assert name in monitor.pending_waits
        assert name in monitor.monitor_state_success

        # 2. Simulate Success Condition met
        monitor.robot_interface.state.get.side_effect = (
            lambda k: True if k == "button_a" else False
        )
        monitor.check_success_on_conditions()

        # 3. Verify it was removed from pending because button_a was True
        assert name not in monitor.pending_waits
        assert name not in monitor.completed_protocols

    def test_resume_by_time(self, monitor):
        """Tests that a protocol resumes if its wait time expires."""
        name = "HighPrioProtocol.sub1"
        # Set resume time to 5 seconds ago
        monitor.pending_waits[name] = datetime.now() - timedelta(seconds=5)
        monitor.completed_protocols.add(name)

        resumed = monitor.wait_resumed(name)
        assert resumed is True
        assert name not in monitor.pending_waits
        assert name not in monitor.completed_protocols

    ## 3. Reset Patterns & Completion
    def test_periodic_reset_logic(self, monitor):
        """Tests that periodic protocols reset after the specified time."""
        monitor.protocols_yaml["protocols"]["PeriodicProtocol"] = {
            "sub1": {
                "high_level": {"reset_pattern": {"type": "periodic", "minutes": 30}}
            },
            "instant_sub": {"high_level": {"reset_pattern": {"type": "instant"}}},
        }

        name = "PeriodicProtocol.sub1"
        monitor.blackboard.set(f"sub1_done", {"val": True})

        # Mark as completed (30 min reset)
        monitor.mark_completed(name)
        assert name in monitor.completed_protocols

        # Manipulate the timestamp in the reset queue to 31 minutes ago
        old_time = datetime.now() - timedelta(minutes=31)
        # Re-set the tuple in the set: (name, finished_time, reset_seconds)
        monitor.protocols_to_reset = {(name, old_time, 1800)}

        monitor.check_and_reset_protocols()

        # Verify it's ready to trigger again
        assert name not in monitor.completed_protocols
        assert monitor.blackboard.get("sub1_done")["val"] is False

    def test_instant_reset_logic(self, monitor):
        """Tests that 'instant' type protocols never stay in completed_protocols."""
        # Inject an instant protocol
        monitor.protocols_yaml["protocols"]["InstantTest"] = {
            "sub": {"high_level": {"reset_pattern": {"type": "instant"}}}
        }

        monitor.mark_completed("InstantTest.sub")
        # Should NOT be in completed_protocols because it's instant
        assert "InstantTest.sub" not in monitor.completed_protocols

    def test_daily_reset(self, monitor):
        """Tests the logic that resets all protocols on a new day."""
        monitor.blackboard.set("test_done", {"status": True})
        # We don't need to wait for midnight; we can call the helper directly
        monitor.reset_all_protocol_dones()

        assert monitor.blackboard.get("test_done")["status"] is False

    def test_stop_monitor(self, monitor):
        """Verifies the stop flag correctly shuts down the loop."""
        assert monitor.stop_flag is False
        monitor.stop_monitor()
        assert monitor.stop_flag is True
