import unittest
from unittest.mock import MagicMock, patch, PropertyMock
import rclpy
from smart_home_pytree.protocol_orchestrator import ProtocolOrchestrator
from unittest.mock import MagicMock, call
import threading
import time
    
class TestProtocolOrchestrator(unittest.TestCase):
    def setUp(self):
        """Runs before every single test method."""
        # 1. Patch rclpy so we don't actually start ROS nodes
        self.rclpy_patcher = patch('smart_home_pytree.protocol_orchestrator.rclpy')
        self.mock_rclpy = self.rclpy_patcher.start()
        # Mock rclpy.ok() to return False initially so your __init__ tries to init, 
        # but does nothing because it's a mock.
        self.mock_rclpy.ok.return_value = False

        # 2. Patch threading to prevent real threads from spawning in __init__
        self.threading_patcher = patch('smart_home_pytree.protocol_orchestrator.threading')
        self.mock_threading = self.threading_patcher.start()
       
        # 3. Patch your custom modules (TriggerMonitor, HumanInterface, etc)
        # We replace the CLASSES with Mocks, so when Orchestrator calls HumanInterface(), it gets a fake object.
        self.monitor_patcher = patch('smart_home_pytree.protocol_orchestrator.TriggerMonitor')
        self.mock_monitor_class = self.monitor_patcher.start()
       
        self.human_patcher = patch('smart_home_pytree.protocol_orchestrator.HumanInterface')
        self.mock_human_class = self.human_patcher.start()

        self.robot_patcher = patch('smart_home_pytree.protocol_orchestrator.get_robot_interface')
        self.mock_get_robot = self.robot_patcher.start()      
        
    def tearDown(self):
        """Runs after every test to clean up mocks."""
        patch.stopall()

    def test_initialization(self):
        """Test if the orchestrator initializes without crashing."""
        orch = ProtocolOrchestrator(debug=True)
        
        # Assertions
        self.assertTrue(orch.rclpy_initialized_here)
        # Verify TriggerMonitor was started (via the mock thread)
        self.mock_threading.Thread.assert_called() 
        print("\n[Test] Initialization passed.")
        
    def test_reconcile_protocols_starts_best_candidate(self):
        """Test if the logic picks the highest priority protocol."""
        orch = ProtocolOrchestrator(debug=True)
        
        # SETUP THE SCENARIO
        # 1. Mock the trigger monitor to return two satisfied protocols
        # Tuple format from your code: ("Name", priority_int) where lower is better
        mock_monitor_instance = self.mock_monitor_class.return_value
        mock_monitor_instance.get_satisfied.return_value = [
            ("CoffeeProtocol.coffee", 10),
            ("XReminderProtocol.medicine_am", 1) 
        ]

        # 2. Mock start_protocol because we don't want to actually run the PyTree
        orch.start_protocol = MagicMock()

        # EXECUTE
        orch._reconcile_protocols()

        # VERIFY
        # It should have picked Priority 1 (HighPriorityTask)
        orch.start_protocol.assert_called_once_with(("XReminderProtocol.medicine_am", 1))
        print("[Test] Priority logic passed.")


    # def test_interrupt_stops_real_loop(self):
    #     """
    #     Integration-style test:
    #     1. Loop runs in background.
    #     2. We let it start a protocol.
    #     3. We trigger the human interrupt event.
    #     4. We verify the protocol was stopped.
    #     """
    #     # 1. Setup Orchestrator
    #     orch = ProtocolOrchestrator(debug=True)
        
    #     # --- MOCKS FOR DEPENDENCIES ---
    #     # We mock the monitor to return a valid task so the loop *starts* something.
    #     orch.trigger_monitor = MagicMock()
    #     orch.trigger_monitor.get_satisfied.return_value = [("CleaningProtocol.living_room", 5)]
        
    #     # Mock the actual tree execution methods to track calls and avoid real threads spawning
    #     orch.stop_protocol = MagicMock()
    #     orch.start_protocol = MagicMock()

    #     # CRITICAL: Since we mocked start_protocol, we must manually simulate 
    #     # what it does (setting running_tree) so the loop knows something is running.
    #     def start_side_effect(candidate):
    #         print(f"[Test] Loop requested start of {candidate}")
    #         orch.running_tree = {"name": "living_room", "priority": 1, "tree": MagicMock()}
    #     orch.start_protocol.side_effect = start_side_effect

    #     # 2. START THE LOOP (Real Thread)
    #     # We run the actual orchestrator_loop in a daemon thread so it runs in parallel.
    #     loop_thread = threading.Thread(target=orch.orchestrator_loop, daemon=True)
    #     loop_thread.start()

    #     # 3. PHASE 1: START PROTOCOL
    #     print("\n[Test] Phase 1: Waking up loop to start protocol...")
    #     orch.orchestrator_wakeup.set()
        
    #     # Wait briefly for the loop to process
    #     time.sleep(0.1) 
        
    #     # Check: Did it try to start the protocol?
    #     orch.start_protocol.assert_called()
    #     assert orch.running_tree is not None, "Protocol should be running now"

    #     # 4. PHASE 2: TRIGGER INTERRUPT
    #     print("[Test] Phase 2: Triggering Human Interrupt...")
    #     orch.human_interrupt_event.set()  # Real event set
    #     orch.orchestrator_wakeup.set()    # Wake up the loop to see the event

    #     # Wait briefly for the loop to react
    #     time.sleep(0.1)

    #     # 5. VERIFY STOP
    #     print("[Test] Phase 3: Verifying stop...")
    #     orch.stop_protocol.assert_called_once()
        
    #     # 6. CLEANUP (Graceful Shutdown)
    #     # We must break the orchestrator out of the interrupt trap loop
    #     orch.stop_flag = True
    #     orch.human_interrupt_event.clear() # Clear interrupt so it exits the inner loop
    #     orch.orchestrator_wakeup.set()     # Wake it up one last time to see stop_flag
    #     loop_thread.join(timeout=1.0)
        
    #     print("[Test] DONE. Loop exited cleanly.")