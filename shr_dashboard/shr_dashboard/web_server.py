#!/usr/bin/env python3
"""Web server for the SHR operator dashboard.

V1 scope:
- blank home page shell
- Protocol Designer for GenericProtocol entries in house YAML
- schema validation and safe save with backup

Designed to run on the robot and be accessed over the local network.
"""

from __future__ import annotations

import argparse
import json
import os
import time as _time
from datetime import datetime
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_from_directory
from ament_index_python.packages import get_package_share_directory

from .config_service import (
    DashboardConfigError,
    backup_and_save_config,
    build_dashboard_payload,
    dump_config_yaml,
    resolve_house_yaml_path,
    validate_config_payload,
)


def _resolve_static_dir() -> Path:
    """
    Resolve the static asset directory for both installed ROS package runs and source runs.
    """
    try:
        share_dir = Path(get_package_share_directory("shr_dashboard"))
        candidate = share_dir / "static"
        if candidate.exists():
            return candidate
    except Exception:
        # Fall back to source-tree local static path for direct python execution.
        pass

    return Path(__file__).parent / "static"


def create_app() -> Flask:
    static_dir = _resolve_static_dir()
    app = Flask(__name__, static_folder=str(static_dir), static_url_path="/static")

    @app.route("/")
    def index():
        return send_from_directory(app.static_folder, "index.html")

    @app.route("/health")
    def health():
        return {"ok": True}

    # ---- Robot State API (reads JSON file written by robot_interface) ----
    _state_file = Path(os.getenv("SHR_STATE_FILE", "/tmp/shr_robot_state.json"))

    @app.route("/api/robot-state", methods=["GET"])
    def api_robot_state():
        """Return the latest robot state snapshot (written by robot_interface every 2s)."""
        try:
            if not _state_file.exists():
                return jsonify({"ok": False, "error": "State file not available yet."}), 503
            data = json.loads(_state_file.read_text())
            return jsonify({"ok": True, "state": data})
        except Exception as exc:
            return jsonify({"ok": False, "error": f"robot-state failed: {exc}"}), 500

    @app.route("/favicon.ico")
    def favicon():
        return ("", 204)

    @app.route("/api/dashboard/bootstrap", methods=["GET"])
    def api_bootstrap():
        try:
            ctx = resolve_house_yaml_path(request.args.get("yaml_path"))
            payload = build_dashboard_payload(ctx)
            return jsonify({"ok": True, **payload})
        except DashboardConfigError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except Exception as exc:
            return jsonify({"ok": False, "error": f"bootstrap failed: {exc}"}), 500

    @app.route("/api/dashboard/validate", methods=["POST"])
    def api_validate():
        body = request.get_json(force=True, silent=True) or {}
        data = body.get("config")
        if not isinstance(data, dict):
            return jsonify({"ok": False, "error": "Body must contain object field 'config'"}), 400
        try:
            validate_config_payload(data)
            return jsonify(
                {
                    "ok": True,
                    "yaml_preview": dump_config_yaml(data),
                }
            )
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.route("/api/dashboard/save", methods=["POST"])
    def api_save():
        body = request.get_json(force=True, silent=True) or {}
        data = body.get("config")
        if not isinstance(data, dict):
            return jsonify({"ok": False, "error": "Body must contain object field 'config'"}), 400
        try:
            ctx = resolve_house_yaml_path(body.get("yaml_path"))
            save_info = backup_and_save_config(ctx, data)
            payload = build_dashboard_payload(ctx)
            return jsonify({"ok": True, "save": save_info, **payload})
        except DashboardConfigError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except Exception as exc:
            return jsonify({"ok": False, "error": f"save failed: {exc}"}), 500

    # ---- Protocol History & State API ----

    def _get_tracker():
        """Lazy-init a read-only ProtocolTracker instance for the web server."""
        if not hasattr(app, "_protocol_tracker"):
            from smart_home_pytree.protocol_tracker import ProtocolTracker
            app._protocol_tracker = ProtocolTracker()
        return app._protocol_tracker

    def _parse_ymd(value: str, field_name: str) -> str:
        """Validate YYYY-MM-DD and return normalized value."""
        try:
            parsed = datetime.strptime(value, "%Y-%m-%d")
            return parsed.strftime("%Y-%m-%d")
        except ValueError as exc:
            raise ValueError(f"Invalid {field_name} '{value}'. Expected YYYY-MM-DD.") from exc

    @app.route("/api/protocol-history", methods=["GET"])
    def api_protocol_history():
        """Return protocol log entries with optional filters.

        Query params:
          date      – YYYY-MM-DD (single-day mode; default: today)
          date_from – YYYY-MM-DD (range mode; requires date_to)
          date_to   – YYYY-MM-DD (range mode; requires date_from)
          all       – true/1/yes to return all dates
          status    – completed | failed | preempted | yielded (optional)
          protocol  – substring match on protocol name (optional)
        """
        tracker = _get_tracker()
        status_filter = request.args.get("status", "").strip().lower()
        protocol_filter = request.args.get("protocol", "").strip().lower()
        include_yielded = status_filter == "yielded"
        view_mode = request.args.get("view", "compact").strip().lower() or "compact"
        if view_mode not in {"compact", "all_runs"}:
            return jsonify({"ok": False, "error": f"Invalid view '{view_mode}'. Use compact or all_runs."}), 400
        today = datetime.now().strftime("%Y-%m-%d")
        all_flag = request.args.get("all", "").strip().lower() in {"1", "true", "yes"}
        date_from_raw = request.args.get("date_from", "").strip()
        date_to_raw = request.args.get("date_to", "").strip()

        if all_flag and (date_from_raw or date_to_raw):
            return jsonify({"ok": False, "error": "Use either all=true or date_from/date_to, not both."}), 400

        try:
            mode = "single"
            date_str = request.args.get("date", today).strip() or today
            date_from = None
            date_to = None

            if all_flag:
                mode = "all"
                if view_mode == "all_runs":
                    entries = tracker.get_all_logs()
                else:
                    entries = tracker.get_compact_log_all(include_yielded=include_yielded)
            elif date_from_raw or date_to_raw:
                if not date_from_raw or not date_to_raw:
                    return jsonify(
                        {"ok": False, "error": "Both date_from and date_to are required for range mode."}
                    ), 400
                mode = "range"
                date_from = _parse_ymd(date_from_raw, "date_from")
                date_to = _parse_ymd(date_to_raw, "date_to")
                if date_from > date_to:
                    return jsonify({"ok": False, "error": "date_from cannot be after date_to."}), 400
                if view_mode == "all_runs":
                    entries = tracker.get_log_by_date_range(date_from, date_to)
                else:
                    entries = tracker.get_compact_log_by_date_range(
                        date_from,
                        date_to,
                        include_yielded=include_yielded,
                    )
            else:
                date_str = _parse_ymd(date_str, "date")
                if view_mode == "all_runs":
                    entries = tracker.get_log_by_date(date_str)
                else:
                    entries = tracker.get_compact_log_by_date(
                        date_str,
                        include_yielded=include_yielded,
                    )

            # Apply optional filters

            if status_filter:
                entries = [e for e in entries if e.get("status", "").lower() == status_filter]
            if protocol_filter:
                entries = [e for e in entries if protocol_filter in e.get("protocol", "").lower()]

            query = {
                "mode": mode,
                "date": date_str if mode == "single" else None,
                "date_from": date_from if mode == "range" else None,
                "date_to": date_to if mode == "range" else None,
                "all": mode == "all",
                "view": view_mode,
            }
            return jsonify({"ok": True, "query": query, "entries": entries, "count": len(entries)})
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except Exception as exc:
            return jsonify({"ok": False, "error": f"history failed: {exc}"}), 500

    @app.route("/api/protocol-state", methods=["GET"])
    def api_protocol_state():
        """Return current state of all protocols."""
        try:
            tracker = _get_tracker()
            states = tracker.get_all_states()
            return jsonify({"ok": True, "states": states, "count": len(states)})
        except Exception as exc:
            return jsonify({"ok": False, "error": f"state failed: {exc}"}), 500

    @app.route("/api/protocol-summary", methods=["GET"])
    def api_protocol_summary():
        """Return lightweight counts of today's protocol runs by status.

        Query params:
          date – YYYY-MM-DD (default: today)
        """
        today = datetime.now().strftime("%Y-%m-%d")
        date_str = request.args.get("date", today).strip() or today
        try:
            date_str = _parse_ymd(date_str, "date")
            tracker = _get_tracker()
            entries = tracker.get_compact_log_by_date(date_str, include_yielded=False)
            counts = {"completed": 0, "failed": 0, "preempted": 0}
            for e in entries:
                s = e.get("status", "").lower()
                if s in counts:
                    counts[s] += 1
            counts["total"] = len(entries)
            return jsonify({"ok": True, "date": date_str, **counts})
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except Exception as exc:
            return jsonify({"ok": False, "error": f"summary failed: {exc}"}), 500

    @app.route("/api/protocol-state/<path:protocol>", methods=["PATCH"])
    def api_protocol_state_update(protocol: str):
        """Update a single protocol's state from the dashboard.

        Body JSON fields (all optional — only provided fields are applied):
          state        – idle | cooldown | waiting
          eligible_at  – ISO datetime string or null to clear
          resume_at    – ISO datetime string or null to clear
        """
        body = request.get_json(force=True, silent=True) or {}
        allowed_states = {"idle", "cooldown", "waiting"}
        new_state = body.get("state")
        if new_state and new_state not in allowed_states:
            return jsonify(
                {"ok": False, "error": f"Invalid state '{new_state}'. Allowed: {', '.join(sorted(allowed_states))}"}
            ), 400

        try:
            tracker = _get_tracker()
            existing = tracker.get_state(protocol)
            if existing is None:
                return jsonify({"ok": False, "error": f"Protocol '{protocol}' not found in state table"}), 404

            kwargs: dict = {}
            if new_state:
                kwargs["state"] = new_state
                # When switching to idle, clear timing fields and step progress
                if new_state == "idle":
                    kwargs["started_at"] = None
                    kwargs["eligible_at"] = None
                    kwargs["resume_at"] = None
                    kwargs["total_steps"] = None
                    kwargs["completed_step"] = 0
                    kwargs["run_session_id"] = None
            if "eligible_at" in body:
                kwargs["eligible_at"] = body["eligible_at"]  # string or None
            if "resume_at" in body:
                kwargs["resume_at"] = body["resume_at"]

            tracker.upsert_state(protocol, **kwargs)
            updated = tracker.get_state(protocol)
            return jsonify({"ok": True, "protocol": protocol, "state": updated})
        except Exception as exc:
            return jsonify({"ok": False, "error": f"update failed: {exc}"}), 500

    @app.route("/api/protocol-events")
    def api_protocol_events():
        """SSE stream that pushes new data when protocol_state or protocol_log changes.

        The client receives:
          event: state   – when protocol_state.updated_at changes
          event: log     – when a new protocol_log row appears for the requested date

        Query params:
          date – YYYY-MM-DD (default: today) – which day's log to watch
        """
        date_str = request.args.get("date", datetime.now().strftime("%Y-%m-%d"))

        def stream():
            tracker = _get_tracker()
            last_state_ts = tracker.get_latest_update_time() or ""
            last_log_id = tracker.get_latest_log_id(date_str)

            while True:
                _time.sleep(2)  # poll interval

                try:
                    new_state_ts = tracker.get_latest_update_time() or ""
                    new_log_id = tracker.get_latest_log_id(date_str)

                    if new_state_ts != last_state_ts:
                        last_state_ts = new_state_ts
                        states = tracker.get_all_states()
                        payload = json.dumps({"states": states})
                        yield f"event: state\ndata: {payload}\n\n"

                    if new_log_id != last_log_id:
                        last_log_id = new_log_id
                        entries = tracker.get_compact_log_by_date(date_str)
                        payload = json.dumps({"entries": entries, "date": date_str})
                        yield f"event: log\ndata: {payload}\n\n"

                except GeneratorExit:
                    return
                except Exception:
                    # Connection likely closed; silently stop
                    return

        return Response(
            stream(),
            mimetype="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "X-Accel-Buffering": "no",  # Nginx: disable buffering
                "Connection": "keep-alive",
            },
        )

    @app.errorhandler(404)
    def not_found(_exc):
        if request.path.startswith("/api/"):
            return jsonify({"ok": False, "error": f"Unknown API route: {request.path}"}), 404
        # SPA hash routing keeps everything under /. For direct path refreshes, serve index.
        return send_from_directory(app.static_folder, "index.html")

    return app


def main(args=None):
    parser = argparse.ArgumentParser(description="SHR Dashboard web server")
    parser.add_argument("--host", default=os.getenv("SHR_DASHBOARD_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.getenv("SHR_DASHBOARD_PORT", "5080")))
    parser.add_argument("--debug", action="store_true")
    parsed = parser.parse_args(args=args)

    app = create_app()
    app.run(host=parsed.host, port=parsed.port, debug=parsed.debug, use_reloader=False)


if __name__ == "__main__":
    main()
