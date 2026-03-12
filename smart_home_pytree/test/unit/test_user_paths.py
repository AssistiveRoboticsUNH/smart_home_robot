from pathlib import Path

import pytest

from smart_home_pytree import utils


def test_get_house_yaml_path_uses_shr_user_dir(monkeypatch, tmp_path):
    user_dir = tmp_path / "ola"
    config_path = user_dir / "configs" / "house_config.yaml"
    config_path.parent.mkdir(parents=True)
    config_path.write_text("locations: {}\nprotocols: {}\n", encoding="utf-8")

    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    assert utils.get_house_yaml_path() == str(config_path.resolve())


def test_get_house_yaml_path_requires_shr_user_dir(monkeypatch):
    monkeypatch.delenv("SHR_USER_DIR", raising=False)
    with pytest.raises(RuntimeError, match="SHR_USER_DIR"):
        utils.get_house_yaml_path()


def test_resolve_media_path_uses_user_media_dir(monkeypatch, tmp_path):
    user_dir = tmp_path / "ola"
    (user_dir / "audios").mkdir(parents=True)
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    resolved = utils.resolve_media_path("food_reminder.mp3", "audio")
    assert resolved == str((user_dir / "audios" / "food_reminder.mp3").resolve())


def test_resolve_config_file_path_finds_package_config(monkeypatch, tmp_path):
    user_dir = tmp_path / "ola"
    (user_dir / "configs").mkdir(parents=True)
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    resolved = utils.resolve_config_file_path("exercise_routine_test.yaml")
    assert resolved is not None
    assert Path(resolved).name == "exercise_routine_test.yaml"
    assert Path(resolved).is_file()
