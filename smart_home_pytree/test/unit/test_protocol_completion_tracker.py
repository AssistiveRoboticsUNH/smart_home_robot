from __future__ import annotations

from datetime import datetime

from smart_home_pytree.protocol_tracker import ProtocolTracker


def test_protocol_completion_events_are_consumed_per_target(tmp_path, monkeypatch):
    user_dir = tmp_path / "user"
    (user_dir / "database").mkdir(parents=True)
    monkeypatch.setenv("SHR_USER_DIR", str(user_dir))

    tracker = ProtocolTracker()
    try:
        tracker.record_protocol_completion_event(
            source_protocol="GenericProtocol.bathroom_reminder",
            source_run_session_id="session-1",
            status="completed",
            completed_at=datetime.now(),
        )

        target_a = "GenericProtocol.bathroom_thanks"
        target_b = "GenericProtocol.bathroom_followup"

        rows_a = tracker.get_unconsumed_protocol_completion_events(target_protocol=target_a)
        rows_b = tracker.get_unconsumed_protocol_completion_events(target_protocol=target_b)

        assert len(rows_a) == 1
        assert len(rows_b) == 1
        assert rows_a[0]["id"] == rows_b[0]["id"]

        tracker.consume_protocol_completion_events(
            target_protocol=target_a,
            event_ids=[rows_a[0]["id"]],
        )

        assert tracker.get_unconsumed_protocol_completion_events(target_protocol=target_a) == []
        rows_b_after = tracker.get_unconsumed_protocol_completion_events(target_protocol=target_b)
        assert len(rows_b_after) == 1
        assert rows_b_after[0]["id"] == rows_b[0]["id"]
    finally:
        tracker.close()
