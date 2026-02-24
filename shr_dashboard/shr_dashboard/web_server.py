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
import os
from pathlib import Path

from flask import Flask, jsonify, request, send_from_directory
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
