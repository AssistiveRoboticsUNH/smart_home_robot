"""
Non-ROS command interpreter that runs in its own worker thread.

Design goals:
- Always enqueue text quickly (non-blocking) and return immediately.
- Drain the queue in a background thread and interpret commands.
- Call an LLM (OpenRouter) with tool calls to trigger actions.
- Optionally speak via tool handlers (e.g., publish to /voice/speak).
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
import urllib.error
import urllib.request
import socket
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    import yaml
except Exception:  # pragma: no cover - handled at runtime
    yaml = None

from . import tool_handlers


@dataclass
class InterpreterConfig:
    """Configuration for CommandInterpreter."""

    openrouter_api_key: Optional[str] = None
    openrouter_model: str = "openai/gpt-4o-mini"
    openrouter_base_url: str = "https://openrouter.ai/api/v1/chat/completions"
    openrouter_app_url: Optional[str] = None
    openrouter_app_name: Optional[str] = None
    http_timeout_s: float = 20.0
    llm_max_retries: int = 2
    llm_min_interval_s: float = 0.5
    llm_backoff_base_s: float = 0.75
    llm_backoff_max_s: float = 6.0
    max_queue_size: int = 50
    drop_oldest_on_full: bool = True
    tool_config_path: Optional[str] = None


class CommandInterpreter:
    """
    Queue-based command interpreter. Non-ROS by design.

    Usage (typical):
        # tool_handlers.set_tool_context(speak_cb=..., action_cb=..., logger=...)
        interpreter = CommandInterpreter(logger=node.get_logger())
        interpreter.start()
        interpreter.enqueue(user_text)
    """

    def __init__(
        self,
        config: Optional[InterpreterConfig] = None,
        logger: Optional[Any] = None,
    ) -> None:
        self._config = config or InterpreterConfig()
        if self._config.openrouter_api_key is None:
            self._config.openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        if not self._config.openrouter_model:
            self._config.openrouter_model = os.getenv(
                "OPENROUTER_MODEL", "openai/gpt-5-mini"
            )
        if self._config.openrouter_app_url is None:
            self._config.openrouter_app_url = os.getenv("OPENROUTER_APP_URL")
        if self._config.openrouter_app_name is None:
            self._config.openrouter_app_name = os.getenv("OPENROUTER_APP_NAME")

        self._logger = logger

        self._queue: "queue.Queue[str]" = queue.Queue(maxsize=self._config.max_queue_size)
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._rate_lock = threading.Lock()
        self._last_request_ts = 0.0

        self._tools: Dict[str, Dict[str, Any]] = {}
        self._openrouter_tool_specs = []
        tool_config_path = self._resolve_tool_config_path()
        self._load_tool_config(tool_config_path)

        self._system_prompt = (
            "You are a command interpreter for a home robot. "
            "Map one user utterance into concrete tool calls. "
            "High-priority rules: "
            "1) Prefer tool calls over plain text whenever possible. "
            "2) You may call multiple tools in one response. "
            "3) If you call any action tool, also call speak to acknowledge what you are doing. "
            "4) If you ask a question, request clarification, or expect any follow-up from the user, you MUST call listen after speak in the same response. "
            "5) If the command is unclear or incomplete, call speak with a brief clarification question, then call listen. "
            "6) Only skip listen when the task is fully complete and no user response is needed. "
            "Safety: do not provide medical or legal advice; if asked, use speak to decline briefly and call listen if the user may continue. "
            "Output policy: use tool calls as the primary output."
        )

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop,
            name="command-interpreter",
            daemon=True,
        )
        self._thread.start()
        self._log("info", "CommandInterpreter started")

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        self._log("info", "CommandInterpreter stopped")

    def enqueue(self, text: str) -> bool:
        clean = (text or "").strip()
        if not clean:
            return False
        try:
            self._queue.put_nowait(clean)
            return True
        except queue.Full:
            if self._config.drop_oldest_on_full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(clean)
                    self._log("warn", "Command queue full; dropped oldest item")
                    return True
                except queue.Full:
                    self._log("warn", "Command queue still full; dropping new item")
                    return False
            self._log("warn", "Command queue full; dropping new item")
            return False

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                text = self._queue.get(timeout=0.25)
            except queue.Empty:
                continue

            try:
                self._handle_text(text)
            except Exception as exc:
                self._log("error", f"Interpreter error: {exc}")

    def _handle_text(self, text: str) -> None:
        # LLM is always invoked for interpretation.
        self._call_openrouter(text)

    def _throttle(self) -> None:
        min_interval = max(0.0, self._config.llm_min_interval_s)
        if min_interval <= 0.0:
            return
        with self._rate_lock:
            now = time.monotonic()
            elapsed = now - self._last_request_ts
            wait = min_interval - elapsed
            if wait > 0:
                time.sleep(wait)
            self._last_request_ts = time.monotonic()

    def _backoff_sleep(self, attempt: int) -> None:
        base = max(0.0, self._config.llm_backoff_base_s)
        max_s = max(base, self._config.llm_backoff_max_s)
        if base <= 0.0:
            return
        delay = min(max_s, base * (2 ** attempt))
        if delay > 0:
            time.sleep(delay)

    @staticmethod
    def _is_retryable_http_status(status: int) -> bool:
        if status in (408, 429):
            return True
        if 500 <= status < 600:
            return True
        return False

    def _call_openrouter(self, text: str) -> bool:
        if not self._config.openrouter_api_key:
            self._log("error", "OPENROUTER_API_KEY is missing; cannot call LLM.")
            return False

        payload = {
            "model": self._config.openrouter_model,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": text},
            ],
            "tools": self._openrouter_tool_specs,
            "tool_choice": "auto",
            "temperature": 0.2,
        }
        headers = {
            "Authorization": f"Bearer {self._config.openrouter_api_key}",
            "Content-Type": "application/json",
        }
        if self._config.openrouter_app_url:
            headers["HTTP-Referer"] = self._config.openrouter_app_url
        if self._config.openrouter_app_name:
            headers["X-Title"] = self._config.openrouter_app_name

        max_retries = max(0, self._config.llm_max_retries)
        for attempt in range(max_retries + 1):
            self._throttle()
            try:
                request = urllib.request.Request(
                    self._config.openrouter_base_url,
                    data=json.dumps(payload).encode("utf-8"),
                    headers=headers,
                    method="POST",
                )
                with urllib.request.urlopen(
                    request, timeout=self._config.http_timeout_s
                ) as response:
                    body = response.read().decode("utf-8")
                data = json.loads(body)
            except urllib.error.HTTPError as exc:
                if (
                    self._is_retryable_http_status(exc.code)
                    and attempt < max_retries
                ):
                    self._log(
                        "warn",
                        f"OpenRouter HTTP {exc.code}; retrying ({attempt + 1}/{max_retries})",
                    )
                    self._backoff_sleep(attempt)
                    continue
                err_body = exc.read().decode("utf-8", errors="ignore") if exc.fp else ""
                self._log("error", f"OpenRouter HTTP error: {exc} {err_body}")
                return False
            except (urllib.error.URLError, socket.timeout, TimeoutError) as exc:
                if attempt < max_retries:
                    self._log(
                        "warn",
                        f"OpenRouter network error: {exc}; retrying ({attempt + 1}/{max_retries})",
                    )
                    self._backoff_sleep(attempt)
                    continue
                self._log("error", f"OpenRouter network error: {exc}")
                return False
            except Exception as exc:
                self._log("error", f"OpenRouter request failed: {exc}")
                return False

            message = self._extract_message(data)
            if not message:
                return False

            tool_calls = message.get("tool_calls", [])
            if tool_calls:
                for call in tool_calls:
                    self._handle_tool_call(call, source_text=text)
                return True

            content = (message.get("content") or "").strip()
            if content:
                self._maybe_speak(content)
                return True

            return False

    def _extract_message(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        choices = data.get("choices", [])
        if not choices:
            return None
        message = choices[0].get("message", {})
        return message

    def _resolve_tool_config_path(self) -> str:
        if self._config.tool_config_path:
            return self._config.tool_config_path
        env_path = os.getenv("SHR_TOOL_CONFIG")
        if env_path:
            return env_path
        base_dir = os.path.dirname(__file__)
        return os.path.join(base_dir, "tool_configs", "tools.yaml")

    def _load_tool_config(self, path: str) -> None:
        if yaml is None:
            self._log("error", "PyYAML is not installed; cannot load tool config")
            return
        if not os.path.exists(path):
            self._log("error", f"Tool config not found: {path}")
            return

        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file) or {}

        tools = data.get("tools", [])
        if not isinstance(tools, list):
            self._log("error", "Tool config 'tools' must be a list")
            return

        self._tools.clear()
        self._openrouter_tool_specs = []
        for entry in tools:
            if not isinstance(entry, dict):
                continue
            if entry.get("enabled", True) is False:
                continue

            name = str(entry.get("name", "")).strip()
            handler = str(entry.get("handler", "")).strip()
            if not name or not handler:
                continue

            description = str(entry.get("description", "")).strip()
            parameters = entry.get("parameters") or {"type": "object", "properties": {}}
            config = entry.get("config") or {}

            self._tools[name] = {
                "handler": handler,
                "config": config,
            }
            self._openrouter_tool_specs.append(
                {
                    "type": "function",
                    "function": {
                        "name": name,
                        "description": description,
                        "parameters": parameters,
                    },
                }
            )

        self._log("info", f"Loaded {len(self._tools)} tools from {path}")

    def _handle_tool_call(self, call: Dict[str, Any], source_text: str) -> None:
        function = call.get("function", {})
        name = function.get("name", "")
        raw_args = function.get("arguments", "{}")

        tool = self._tools.get(name)
        if not tool:
            self._log("warn", f"Unknown tool call: {name}")
            return

        try:
            args = json.loads(raw_args) if isinstance(raw_args, str) else raw_args
        except json.JSONDecodeError:
            self._log("warn", f"Invalid tool args for {name}: {raw_args}")
            args = {}

        self._log("info", f"Tool call: name={name}, args={args}")

        handler_name = tool.get("handler", "")
        handler_fn = tool_handlers.get_handler(handler_name)
        if handler_fn is None:
            self._log("warn", f"Unhandled tool handler: {handler_name}")
            return

        handler_fn(args or {}, tool.get("config") or {}, source_text)

    def _maybe_speak(self, text: str) -> None:
        if not text:
            return
        tool_handlers.handle_speak({"text": text}, {}, "")

    def _log(self, level: str, message: str) -> None:
        if self._logger is None:
            print(f"[{level.upper()}] {message}")
            return
        text = str(message)
        try:
            if level == "debug":
                self._logger.debug(text)
            elif level == "warn" or level == "warning":
                self._logger.warn(text)
            elif level == "error":
                self._logger.error(text)
            else:
                self._logger.info(text)
        except Exception:
            print(f"[{level.upper()}] {message}")
