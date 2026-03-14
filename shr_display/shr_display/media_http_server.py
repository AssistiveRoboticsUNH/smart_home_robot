"""Read-only HTTP server for tablet media downloads."""

from __future__ import annotations

import json
import mimetypes
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path, PurePosixPath
from typing import Any
from urllib.parse import unquote, urlparse


def require_shr_user_dir() -> Path:
    raw = os.getenv("SHR_USER_DIR")
    if not raw:
        raise RuntimeError("SHR_USER_DIR is not set. Configure a user profile before starting shr_display.")
    return Path(raw).expanduser().resolve()


def get_videos_root() -> Path:
    return (require_shr_user_dir() / "videos").resolve()


def resolve_video_relative_path(relative_path: str, videos_root: Path | None = None) -> Path:
    if videos_root is None:
        videos_root = get_videos_root()
    safe_path = PurePosixPath(relative_path)
    if safe_path.is_absolute() or ".." in safe_path.parts or str(safe_path) in {"", "."}:
        raise ValueError(f"Invalid video relative path: {relative_path}")

    resolved = (videos_root / safe_path.as_posix()).resolve()
    try:
        resolved.relative_to(videos_root)
    except ValueError as exc:
        raise ValueError(f"Video path escapes videos root: {relative_path}") from exc
    return resolved


def build_video_manifest(videos_root: Path | None = None) -> list[dict[str, Any]]:
    if videos_root is None:
        videos_root = get_videos_root()
    if not videos_root.exists():
        return []

    entries: list[dict[str, Any]] = []
    for path in sorted(videos_root.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(videos_root).as_posix()
        stat = path.stat()
        entries.append(
            {
                "relative_path": relative,
                "file_name": path.name,
                "size_bytes": stat.st_size,
                "modified_time": stat.st_mtime,
            }
        )
    return entries


def _make_handler(videos_root: Path, logger):
    class MediaRequestHandler(BaseHTTPRequestHandler):
        server_version = "SHRMediaHTTP/1.0"

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/media/manifest/videos":
                self._send_manifest()
                return

            if parsed.path.startswith("/media/videos/"):
                relative = unquote(parsed.path[len("/media/videos/"):])
                self._send_video_file(relative)
                return

            self.send_error(HTTPStatus.NOT_FOUND, "Unknown media endpoint")

        def _send_manifest(self):
            payload = json.dumps(
                {"videos": build_video_manifest(videos_root)},
                indent=2,
            ).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _send_video_file(self, relative_path: str):
            try:
                file_path = resolve_video_relative_path(relative_path, videos_root)
            except ValueError as exc:
                self.send_error(HTTPStatus.BAD_REQUEST, str(exc))
                return

            if not file_path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND, "Video not found")
                return

            mime_type, _ = mimetypes.guess_type(str(file_path))
            data = file_path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", mime_type or "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            if logger is not None:
                logger.debug("[MediaHttpServer] " + (fmt % args))

    return MediaRequestHandler


class MediaHttpServer:
    """Small read-only HTTP server exposing SHR user videos."""

    def __init__(self, port: int, logger):
        self.port = int(port)
        self.logger = logger
        self.videos_root = get_videos_root()
        self._server = None
        self._thread = None

    def start(self):
        handler = _make_handler(self.videos_root, self.logger)
        self._server = ThreadingHTTPServer(("0.0.0.0", self.port), handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="shr-display-media-http",
            daemon=True,
        )
        self._thread.start()
        self.logger.info(
            f"Media HTTP server serving {self.videos_root} on /media/videos at port {self.port}"
        )

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
