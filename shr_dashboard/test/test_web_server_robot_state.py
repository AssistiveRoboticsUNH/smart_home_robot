from shr_dashboard import web_server


class _DummyRobotState:
    def snapshot(self):
        return {"charging": True, "robot_location": "living_room"}


class _DummyRobotInterface:
    state = _DummyRobotState()


def test_robot_state_api_reads_live_robot_interface(monkeypatch):
    def _fake_start(app):
        app._robot_interface = _DummyRobotInterface()
        app._robot_executor = None
        app._robot_spin_thread = None
        app._rclpy_initialized_here = False
        app._robot_state_error = None

    monkeypatch.setattr(web_server, "_start_robot_state_client", _fake_start)

    app = web_server.create_app()
    client = app.test_client()
    response = client.get("/api/robot-state")

    assert response.status_code == 200
    assert response.get_json() == {
        "ok": True,
        "state": {"charging": True, "robot_location": "living_room"},
    }


def test_robot_state_api_returns_503_when_robot_state_fails(monkeypatch):
    def _fake_start(app):
        app._robot_interface = None
        app._robot_executor = None
        app._robot_spin_thread = None
        app._rclpy_initialized_here = False
        app._robot_state_error = "boom"

    monkeypatch.setattr(web_server, "_start_robot_state_client", _fake_start)

    app = web_server.create_app()
    client = app.test_client()
    response = client.get("/api/robot-state")

    assert response.status_code == 503
    assert response.get_json() == {"ok": False, "error": "boom"}
