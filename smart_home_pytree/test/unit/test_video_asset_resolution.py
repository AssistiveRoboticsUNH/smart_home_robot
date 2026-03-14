from __future__ import annotations

from pathlib import Path

import pytest

from smart_home_pytree.utils import (
    normalize_media_asset_reference,
    validate_media_asset_exists,
)


def test_normalize_media_asset_reference_accepts_relative_path(monkeypatch, tmp_path):
    user_dir = tmp_path / "user"
    (user_dir / "videos").mkdir(parents=True)
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    assert normalize_media_asset_reference("exercise/demo.mp4", "video") == "exercise/demo.mp4"


def test_normalize_media_asset_reference_converts_absolute_path_under_user_dir(monkeypatch, tmp_path):
    user_dir = tmp_path / "user"
    videos_dir = user_dir / "videos"
    target = videos_dir / "exercise" / "demo.mp4"
    target.parent.mkdir(parents=True)
    target.write_bytes(b"demo")
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    assert normalize_media_asset_reference(str(target), "video") == "exercise/demo.mp4"


def test_normalize_media_asset_reference_rejects_escape(monkeypatch, tmp_path):
    user_dir = tmp_path / "user"
    (user_dir / "videos").mkdir(parents=True)
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    with pytest.raises(ValueError):
        normalize_media_asset_reference("../secret.mp4", "video")


def test_validate_media_asset_exists_returns_relative_and_absolute(monkeypatch, tmp_path):
    user_dir = tmp_path / "user"
    video_path = user_dir / "videos" / "nested" / "demo.mp4"
    video_path.parent.mkdir(parents=True)
    video_path.write_bytes(b"demo")
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    relative, resolved = validate_media_asset_exists("nested/demo.mp4", "video")
    assert relative == "nested/demo.mp4"
    assert resolved == Path(video_path).resolve()
