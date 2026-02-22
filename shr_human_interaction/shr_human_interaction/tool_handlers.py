"""
Centralized tool handler functions.

Handlers are ROS-capable and manage their own node/publishers internally.
"""

from __future__ import annotations

import os, time
import threading
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional
from shr_msgs.action import DockingRequest
from shr_msgs.srv import Speak
# from opennav_docking_msgs.action import DockRobot


import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String


def get_handler(handler_name: str):
    return HANDLERS.get(handler_name)


def handle_speak(args: Dict[str, Any], config: Dict[str, Any], source_text: str) -> None:
    text = str(args.get("text", "")).strip()
    if not text:
        return
    service_name = str(config.get("service_name", "/voice/speak")).strip() or "/voice/speak"
    client = _get_service_client(service_name, Speak)
    if client is None:
        log("warn", f"Speak service client not available for {service_name}")
        return
    wait_s = float(config.get("wait_for_service_s", 3.0))
    if not client.wait_for_service(timeout_sec=wait_s):
        log("warn", f"Speak service not available: {service_name}")
        return

    req = Speak.Request()
    req.text = text
    future = client.call_async(req)

    result_timeout_s = float(config.get("service_timeout_s", 35.0))
    deadline = time.time() + result_timeout_s
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)
    if not future.done():
        log("warn", f"Speak service timeout: {service_name}")
        return
    try:
        result = future.result()
    except Exception as exc:
        log("error", f"Speak service call failed: {exc}")
        return
    if not result.success:
        log("warn", f"Speak service returned failure for text: {text}")


def handle_listen(args: Dict[str, Any], config: Dict[str, Any], source_text: str) -> None:
    topic = "/voice/system_trigger"
    _publish_empty(topic)


def handle_dock_robot(args: Dict[str, Any], config: Dict[str, Any], source_text: str) -> None:
    action_name = str(config.get("action_name", "/dock_robot")).strip() or "/dock_robot"
    action_type = DockingRequest
    
    client = _get_action_client(action_name, action_type)
    if client is None:
        log("warn", f"Docking action client not available for {action_name}")
        return

    wait_s = float(config.get("wait_for_server_s", 3.0))
    if not client.wait_for_server(timeout_sec=wait_s):
        log("warn", f"Docking action server not available: {action_name}")
        return

    goal_msg = action_type.Goal()
    _apply_goal_fields(goal_msg, args)

    try:
        send_goal_future = client.send_goal_async(goal_msg)
    except Exception as exc:
        log("error", f"Failed to send dock goal: {exc}")
        return

    def _on_goal_response(fut):
        try:
            goal_handle = fut.result()
        except Exception as exc:
            log("error", f"Dock goal response error: {exc}")
            return
        if not goal_handle.accepted:
            log("warn", "Dock goal rejected")
            return
        log("info", "Dock goal accepted")

        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(_on_result)
        except Exception as exc:
            log("warn", f"Dock result wait error: {exc}")

    def _on_result(fut):
        try:
            result = fut.result()
        except Exception as exc:
            log("error", f"Dock result error: {exc}")
            return
        log("info", f"Dock result: {result}")

    send_goal_future.add_done_callback(_on_goal_response)


def handle_append_lines_file(args: Dict[str, Any], config: Dict[str, Any], source_text: str) -> None:
    path = str(config.get("path", "")).strip()
    if not path:
        log("warn", "append_lines_file missing path")
        return
    path = os.path.expanduser(path)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    lines_field = str(config.get("lines_field", "items")).strip() or "items"
    raw_lines = args.get(lines_field, [])
    lines = _normalize_lines(raw_lines)
    if not lines:
        return

    with open(path, "a", encoding="utf-8") as file:
        for line in lines:
            file.write(f"{line}\n")

    log("info", f"Appended {len(lines)} lines to {path}")


def _normalize_lines(raw: Any) -> List[str]:
    if raw is None:
        return []
    if isinstance(raw, str):
        return [raw.strip()] if raw.strip() else []
    if isinstance(raw, Iterable):
        lines: List[str] = []
        for item in raw:
            text = str(item).strip()
            if text:
                lines.append(text)
        return lines
    text = str(raw).strip()
    return [text] if text else []


def log(level: str, message: str) -> None:
    logger = _get_logger()
    if logger is None:
        print(f"[{level.upper()}] {message}")
        return
    text = str(message)
    try:
        if level == "debug":
            logger.debug(text)
        elif level == "warn" or level == "warning":
            logger.warn(text)
        elif level == "error":
            logger.error(text)
        else:
            logger.info(text)
    except Exception:
        print(f"[{level.upper()}] {message}")


HANDLERS = {
    "speak": handle_speak,
    "listen": handle_listen,
    "dock_robot": handle_dock_robot,
    "append_lines_file": handle_append_lines_file,
}


_NODE: Optional["Node"] = None
_EXECUTOR: Optional["MultiThreadedExecutor"] = None
_SPIN_THREAD: Optional[threading.Thread] = None
_PUBLISHERS: Dict[str, "rclpy.publisher.Publisher"] = {}
_EMPTY_PUBLISHERS: Dict[str, "rclpy.publisher.Publisher"] = {}
_ACTION_CLIENTS: Dict[str, Any] = {}
_SERVICE_CLIENTS: Dict[str, Any] = {}


def _init_ros_node() -> None:
    global _NODE, _EXECUTOR, _SPIN_THREAD
    if _NODE is not None:
        return
    if not rclpy.ok():
        rclpy.init(args=None)
    _NODE = Node("tool_handlers")
    _EXECUTOR = MultiThreadedExecutor()
    _EXECUTOR.add_node(_NODE)
    _SPIN_THREAD = threading.Thread(target=_EXECUTOR.spin, daemon=True)
    _SPIN_THREAD.start()


def _get_logger() -> Optional[Any]:
    if _NODE is None:
        _init_ros_node()
    if _NODE is None:
        return None
    return _NODE.get_logger()


def _get_publisher(topic: str):
    if _NODE is None:
        _init_ros_node()
    if _NODE is None:
        return None
    pub = _PUBLISHERS.get(topic)
    if pub is None:
        pub = _NODE.create_publisher(String, topic, 10)
        _PUBLISHERS[topic] = pub
    return pub


def _publish_string(topic: str, text: str) -> None:
    pub = _get_publisher(topic)
    if pub is None:
        log("warn", f"No ROS publisher available for {topic}; dropping: {text}")
        return
    msg = String()
    msg.data = text
    pub.publish(msg)


def _get_empty_publisher(topic: str):
    if _NODE is None:
        _init_ros_node()
    if _NODE is None:
        return None
    pub = _EMPTY_PUBLISHERS.get(topic)
    if pub is None:
        pub = _NODE.create_publisher(Empty, topic, 10)
        _EMPTY_PUBLISHERS[topic] = pub
    return pub


def _publish_empty(topic: str) -> None:
    pub = _get_empty_publisher(topic)
    if pub is None:
        log("warn", f"No ROS publisher available for {topic}; dropping Empty trigger")
        return
    pub.publish(Empty())


def _get_action_client(action_name: str, action_type):
    if _NODE is None:
        _init_ros_node()
    if _NODE is None:
        return None
    key = f"{action_name}:{action_type.__name__}"
    client = _ACTION_CLIENTS.get(key)
    if client is None:
        client = ActionClient(_NODE, action_type, action_name)
        _ACTION_CLIENTS[key] = client
    return client


def _get_service_client(service_name: str, service_type):
    if _NODE is None:
        _init_ros_node()
    if _NODE is None:
        return None
    key = f"{service_name}:{service_type.__name__}"
    client = _SERVICE_CLIENTS.get(key)
    if client is None:
        client = _NODE.create_client(service_type, service_name)
        _SERVICE_CLIENTS[key] = client
    return client


def _apply_goal_fields(goal, args: Dict[str, Any]) -> None:
    if not isinstance(args, dict):
        return
    if "dock_type" in args and hasattr(goal, "dock_type"):
        setattr(goal, "dock_type", args.get("dock_type"))
    if "max_docking_time" in args and hasattr(goal, "max_docking_time"):
        setattr(goal, "max_docking_time", args.get("max_docking_time"))
    goal_overrides = args.get("goal")
    if isinstance(goal_overrides, dict):
        for key, value in goal_overrides.items():
            if hasattr(goal, key):
                setattr(goal, key, value)
