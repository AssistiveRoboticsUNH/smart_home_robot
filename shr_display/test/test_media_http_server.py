from __future__ import annotations

from pathlib import Path

import pytest

from shr_display.media_http_server import build_video_manifest, resolve_video_relative_path


def test_resolve_video_relative_path_preserves_nested_paths(tmp_path):
    videos_root = tmp_path / "videos"
    videos_root.mkdir()

    resolved = resolve_video_relative_path("exercise/arms/demo.mp4", videos_root)
    assert resolved == (videos_root / "exercise" / "arms" / "demo.mp4").resolve()


def test_resolve_video_relative_path_rejects_parent_escape(tmp_path):
    videos_root = tmp_path / "videos"
    videos_root.mkdir()

    with pytest.raises(ValueError):
        resolve_video_relative_path("../secret.mp4", videos_root)


def test_build_video_manifest_recurses_relative_paths(tmp_path):
    videos_root = tmp_path / "videos"
    target = videos_root / "exercise" / "arms" / "demo.mp4"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"demo")

    manifest = build_video_manifest(videos_root)
    assert manifest == [
        {
            "relative_path": "exercise/arms/demo.mp4",
            "file_name": "demo.mp4",
            "size_bytes": 4,
            "modified_time": target.stat().st_mtime,
        }
    ]
