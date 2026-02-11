#!/usr/bin/env python3
'''
ROS2 node for human-robot interaction using speech.
Publishes user speech transcripts, robot replies, and voice status.
Requires aalap library for dialogue management.

It also implements an action server(/ask_question) to ask yes/no questions to the user.

Author: Moniruzzaman Akash
Date: Dec 14, 2025
'''

import queue
import time
import multiprocessing as mp
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String, Empty

# Install aalap from https://github.com/MnAkash/aalap.git
from aalap.dialogue_manager import DialogManager
from shr_msgs.action import QuestionRequest

# Sentinel used to short-circuit the ask_question flow when a cancel is requested
_CANCELLED = "__cancelled__"


class ShrHumanInteractionNode(Node):
    """
    Publishes:
      /voice/status : std_msgs/String  (IDLE/LISTENING/RECORDING/TRANSCRIBING/THINKING/SPEAKING/...)
      /voice/user   : std_msgs/String  (all transcribed user text)
      /voice/robot  : std_msgs/String  (all robot speech that is executed)
    Subscribes:
      /voice/speak          : std_msgs/String  (Listens all speak requests)
      /voice/system_trigger : std_msgs/Empty   (Programmatically trigger a wakeword session when idle)
      /voice/stop_session   : std_msgs/Empty   (Force-stop an active session and TTS)

    Action Servers:
      /ask_question : shr_msgs/QuestionRequest
        - Asks a yes/no question to the user via speech.
        - Waits for user response and returns "yes", "no", "other", or "".
        - If cancelled, stops listening and returns "".
    """

    def __init__(self):
        super().__init__("human_interaction")
        self._action_cb_group = ReentrantCallbackGroup()

        # ---- Parameters (override via launch/CLI) ----
        self.declare_parameter("model", "small.en")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("wakeword_keywords", "hey_jarvis")
        self.declare_parameter("vad_silero_threshold", 0.5)
        self.declare_parameter("no_speech_timeout_ms", 3000)
        self.declare_parameter("no_speech_timeout_ms_action_server", 8000)
        self.declare_parameter("ask_attempts", 3)

        # If you have model paths, provide a list via YAML/launch; empty => None
        self.declare_parameter("wakeword_model_paths", [])  # list[str]

        # How frequently to drain queues and publish
        self.declare_parameter("publish_hz", 20.0)

        self._model: str = self.get_parameter("model").get_parameter_value().string_value
        self._device: str = self.get_parameter("device").get_parameter_value().string_value
        self._wakeword_keywords: str = (
            self.get_parameter("wakeword_keywords").get_parameter_value().string_value
        )
        self._vad_thresh: float = (
            self.get_parameter("vad_silero_threshold").get_parameter_value().double_value
        )
        self._no_speech_timeout_ms: int = self.get_parameter("no_speech_timeout_ms").get_parameter_value().integer_value
        self._ask_no_speech_timeout_ms: int = (
            self.get_parameter("no_speech_timeout_ms_action_server").get_parameter_value().integer_value
        )
        self._ask_attempts: int = self.get_parameter("ask_attempts").get_parameter_value().integer_value

        self._wakeword_model_paths = self.get_parameter("wakeword_model_paths").value
        if isinstance(self._wakeword_model_paths, list) and len(self._wakeword_model_paths) == 0:
            self._wakeword_model_paths = None

        publish_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self._publish_period = 1.0 / max(1.0, publish_hz)
        self._latest_status: Optional[str] = None

        # ---- Publishers ----
        self.pub_status = self.create_publisher(String, "/voice/status", 10)
        self.pub_user   = self.create_publisher(String, "/voice/user", 10)
        self.pub_robot  = self.create_publisher(String, "/voice/robot", 10)
        self.sub_speak  = self.create_subscription(String, "/voice/speak", self._on_speak_request, 10)

        # ---- Thread-safe queues fed by DialogManager callbacks ----
        self._status_q: "queue.Queue[str]" = queue.Queue()
        self._user_q: "queue.Queue[str]" = queue.Queue(maxsize=20)
        self._robot_q: "queue.Queue[str]" = queue.Queue(maxsize=20)
        self._question_response_q: "queue.Queue[str]" = queue.Queue(maxsize=10)
        self._question_active = False
        self._question_cancel_event = threading.Event()

        # ---- Control topics (volatile, keep only freshest) ----
        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_system_trigger = self.create_subscription(
            Empty, "/voice/system_trigger", self._on_system_trigger, volatile_qos
        )
        self.sub_stop_session = self.create_subscription(
            Empty, "/voice/stop_session", self._on_stop_session, volatile_qos
        )

        # ---- Create DialogManager (library) ----
        def _on_transcript(text: str):
            # Called from DialogManager internals; do not publish directly from here
            self._user_q.put(text) # Put in user speech queue to publish to topic /voice/user
            if self._question_active:
                try:
                    self._question_response_q.put_nowait(text) # When question answer action is serving, queue there as well.
                except queue.Full:
                    pass

        def _on_status(status: str):
            self._latest_status = status
            self._status_q.put(status)

        def _external_policy(user_text: str) -> str:
            """
            This is where to call LLM / rule policy. Blocking is fine.
            We also enqueue the robot reply so ROS can publish it safely from the timer thread.

            return of this function is played back by the robot without explicit speak() call.
            If "" skips speaking.
            """

            if self._question_active:
                return ""
            else:
                if self._dm.get_session_trigger_reason() == DialogManager.WAKEWORD_TRIGGER:
                    self._dm.deactivate_wakeword_session()
            # TODO: replace this with your real LLM call
            # time.sleep(0.2)
            # robot_reply = f"You said: {user_text}"

            # Enqueue for publishing on /voice/robot
            # self._robot_q.put(robot_reply)
            return ""

        self._dm = DialogManager(
            model=self._model,
            device=self._device,
            on_transcript=_on_transcript,
            on_status=_on_status,
            external_policy=_external_policy,
            wakeword_keywords=self._wakeword_keywords,
            wakeword_arm_thresh=0.1,
            wakeword_model_paths=self._wakeword_model_paths,
            vad_silero_threshold=float(self._vad_thresh),
            no_speech_timeout= self._no_speech_timeout_ms
        )

        # Start the interaction system
        self._dm.start()

        self.get_logger().info(
            f"shr_human_interaction started (model={self._model}, device={self._device}, "
            f"wakeword='{self._wakeword_keywords}', vad={self._vad_thresh})"
        )

        # Timer: drain queues and publish on ROS topics
        self._timer = self.create_timer(self._publish_period, self._drain_and_publish)
        self._ask_question_server = ActionServer(
            self,
            QuestionRequest,
            "/ask_question",
            execute_callback=self._execute_question_request,
            goal_callback=self._on_question_goal,
            cancel_callback=self._on_question_cancel,
            callback_group=self._action_cb_group,
        )

    def _publish_str(self, pub, text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def speak(self, text: str):
        """Speak via DialogManager and mirror the text on /voice/robot."""
        clean = text.strip()
        if not clean:
            return
        try:
            self._dm.speak(clean)
            self._publish_str(self.pub_robot, clean)
        except Exception as e:
            self.get_logger().warn(f"Failed to speak text '{clean}': {e}")

    def _on_speak_request(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        try:
            self.speak(text)
        except Exception as e:
            self.get_logger().warn(f"Failed to speak incoming /voice/speak text: {e}")

    def _drain_and_publish(self):
        # Drain all available items each tick to avoid lag
        try:
            while True:
                status = self._status_q.get_nowait()
                self._publish_str(self.pub_status, status)
        except queue.Empty:
            pass

        try:
            while True:
                user_text = self._user_q.get_nowait()
                self._publish_str(self.pub_user, user_text)
        except queue.Empty:
            pass

        try:
            while True:
                robot_text = self._robot_q.get_nowait()
                self._publish_str(self.pub_robot, robot_text)
        except queue.Empty:
            pass

    def destroy_node(self):
        # Clean shutdown
        try:
            self._ask_question_server.destroy()
        except Exception:
            pass
        try:
            if self._dm is not None:
                self._dm.stop()
        except Exception as e:
            self.get_logger().warn(f"DialogManager stop error: {e}")
        return super().destroy_node()

    #===================================================================================
    ## =================== Action server callbacks for /ask_question ===================
    #===================================================================================
    def _on_question_goal(self, goal_request):
        if self._question_active:
            self.get_logger().warn("Rejecting ask_question goal: another question is already active.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _stop_tts(self):
        try:
            self._dm.tts_player.stop()
        except Exception:
            pass

    def _stop_session(self):
        self._question_cancel_event.set()
        try:
            self._question_response_q.put_nowait(_CANCELLED)
        except queue.Full:
            pass
        self._stop_tts()
        try:
            self._dm.deactivate_wakeword_session()
        except Exception:
            pass
        self._set_no_speech_timeout(self._no_speech_timeout_ms)

    def _set_no_speech_timeout(self, timeout_ms: int):
        try:
            self._dm.no_speech_timeout = int(timeout_ms)
        except Exception:
            pass

    def _is_session_active(self) -> bool:
        return self._latest_status not in (None, DialogManager.IDLE)

    def _on_system_trigger(self, msg: Empty):
        if not self._is_session_active():
            self._dm.trigger_wakeword()

    def _on_stop_session(self, msg: Empty):
        if self._is_session_active():
            self._stop_session()

    def _on_question_cancel(self, goal_handle):
        self._stop_session()
        return CancelResponse.ACCEPT

    def _clear_question_queue(self):
        try:
            while True:
                self._question_response_q.get_nowait()
        except queue.Empty:
            pass

    def _cancel_requested(self, goal_handle) -> bool:
        return self._question_cancel_event.is_set() or goal_handle.is_cancel_requested

    def _cancel_goal(self, goal_handle):
        self._stop_session()
        goal_handle.canceled()
        result = QuestionRequest.Result()
        result.response = ""
        return result

    def _wait_for_status(self, desired: set[str], timeout_s: float, goal_handle=None) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if goal_handle is not None and self._cancel_requested(goal_handle):
                return False
            if self._latest_status in desired:
                return True
            time.sleep(0.05)
        return False

    def _normalize_yes_no(self, text: str) -> Optional[str]:
        cleaned = text.strip().lower()
        yes_tokens = {"yes", "yeah", "yep", "yup", "sure", "affirmative", "correct", "ok", "okay", "aye"}
        no_tokens = {"no", "nope", "nah", "negative", "not", "stop", "cancel", "never"}
        punct_map = {p: " " for p in ",.!?;:"}
        for token in cleaned.translate(str.maketrans(punct_map)).split():
            t = token.strip(".,!?;:")
            if t in yes_tokens:
                return "yes"
            if t in no_tokens:
                return "no"
        cleaned = cleaned.strip(".,!?;:")
        if cleaned in yes_tokens:
            return "yes"
        if cleaned in no_tokens:
            return "no"
        return "other"

    def _wait_for_yes_no(self, goal_handle, timeout_s: float) -> Optional[str]:
        deadline = time.time() + timeout_s
        saw_other = False
        while True:
            if self._cancel_requested(goal_handle):
                return _CANCELLED
            now = time.time()
            remaining = deadline - now
            try:
                if remaining > 0:
                    text = self._question_response_q.get(timeout=min(0.25, remaining))
                else:
                    text = self._question_response_q.get_nowait()
            except queue.Empty:
                text = None
            if text:
                if text == _CANCELLED:
                    return _CANCELLED
                parsed = self._normalize_yes_no(text)
                if parsed in ("yes", "no"):
                    return parsed
                saw_other = True
                # Keep listening in the same attempt; a valid yes/no may arrive next.
                continue

            if remaining <= 0:
                # If transcription is still running, keep waiting for the final text.
                if self._latest_status == DialogManager.TRANSCRIBING:
                    time.sleep(0.05)
                    continue
                if saw_other:
                    return "other"
                return None

    def _execute_question_request(self, goal_handle):
        question_text = goal_handle.request.question or ""
        question = question_text.strip()
        if question and not question.endswith("?"):
            question = f"{question}?"
        prompt = f"{question} Please answer yes or no.".strip()

        self._question_active = True
        self._question_cancel_event.clear()
        self._set_no_speech_timeout(self._ask_no_speech_timeout_ms)
        self._clear_question_queue()
        feedback = QuestionRequest.Feedback()
        feedback.running = True
        goal_handle.publish_feedback(feedback)

        response: Optional[str] = None
        try:
            for attempt in range(max(1, self._ask_attempts)):
                if self._cancel_requested(goal_handle):
                    return self._cancel_goal(goal_handle)

                self._wait_for_status({DialogManager.IDLE}, timeout_s=5.0, goal_handle=goal_handle)
                if self._cancel_requested(goal_handle):
                    return self._cancel_goal(goal_handle)

                # Drop stale transcripts from previous attempts before speaking.
                self._clear_question_queue()
                self.speak(prompt)
                if self._cancel_requested(goal_handle):
                    return self._cancel_goal(goal_handle)

                # Ensure we're back to idle after TTS before opening the mic session.
                self._wait_for_status({DialogManager.IDLE}, timeout_s=5.0, goal_handle=goal_handle)
                if self._cancel_requested(goal_handle):
                    return self._cancel_goal(goal_handle)

                self._dm.trigger_wakeword()
                listening_started = self._wait_for_status(
                    {DialogManager.LISTENING, DialogManager.RECORDING},
                    timeout_s=5.0,
                    goal_handle=goal_handle,
                )
                if self._cancel_requested(goal_handle):
                    return self._cancel_goal(goal_handle)
                if not listening_started:
                    prompt = f"I couldn't start listening. {question} Please answer yes or no."
                    continue

                response = self._wait_for_yes_no(goal_handle, timeout_s=self._ask_no_speech_timeout_ms / 1000.0 + 0.1)
                if response == _CANCELLED:
                    return self._cancel_goal(goal_handle)

                if response == "other":
                    self._dm.deactivate_wakeword_session()
                    prompt = f"Sorry. I only want a yes or no. {question}"
                elif response == None:
                    prompt = f"I couldn't hear anything. {question} Please answer yes or no."
                    continue
                else:
                    break  # got valid yes/no

            self._dm.deactivate_wakeword_session()

            if self._cancel_requested(goal_handle):
                return self._cancel_goal(goal_handle)

            result = QuestionRequest.Result()
            result.response = response or ""
            goal_handle.succeed()
            feedback.running = False
            goal_handle.publish_feedback(feedback)
            return result
        finally:
            self._question_active = False
            self._clear_question_queue()
            self._set_no_speech_timeout(self._no_speech_timeout_ms)
            try:
                self._dm.deactivate_wakeword_session()
            except Exception:
                pass


def main():
    # Your example sets spawn (important when libs use multiprocessing). :contentReference[oaicite:3]{index=3}
    try:
        mp.set_start_method("spawn", force=True)
    except RuntimeError:
        pass

    rclpy.init()
    node = ShrHumanInteractionNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
