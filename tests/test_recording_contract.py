"""Exercise recording decisions without starting the recording worker."""

from types import SimpleNamespace

import pytest

from recorder import RecorderController, RecordSettings, check_dataset_name


@pytest.mark.parametrize("state", ["SAVING", "DISCARDING", "FINALIZING", "CLEANING_UP", "RECORDING", "RESETTING", "AWAIT_SAVE_DECISION", "STARTING", "COUNTDOWN"])
def test_new_stage_is_blocked_while_previous_stage_is_unfinished(state):
    recorder = RecorderController()
    recorder._set(state=state)
    assert not recorder.can_start_new_stage


@pytest.mark.parametrize("state", ["IDLE", "DONE", "ERROR"])
def test_terminal_stage_is_startable_only_after_worker_exits(state):
    recorder = RecorderController()
    recorder._set(state=state)
    assert recorder.can_start_new_stage
    recorder._thread = SimpleNamespace(is_alive=lambda: True)
    assert not recorder.can_start_new_stage


@pytest.mark.parametrize("method,decision", [
    ("save_and_continue", "save_continue"),
    ("save_and_finish", "save_finish"),
    ("discard_and_retake", "discard_retake"),
    ("discard_and_finish", "discard_finish"),
])
def test_confirmation_decisions_are_accepted_only_at_confirmation(method, decision):
    recorder = RecorderController()
    recorder._set(state="RECORDING")
    getattr(recorder, method)()
    assert recorder._episode_decision is None
    recorder._set(state="AWAIT_SAVE_DECISION")
    getattr(recorder, method)()
    assert recorder._episode_decision == decision


def test_finishing_a_stage_preserves_episode_save_confirmation():
    recorder = RecorderController()
    recorder._set(state="RECORDING")
    recorder.request_finish_stage()
    assert recorder.status().finish_requested
    assert recorder._events["exit_early"]
    assert not recorder._events["stop_recording"]
    assert recorder._episode_decision is None


def test_existing_dataset_is_rejected_without_starting_worker(tmp_path):
    (tmp_path / "task_001").mkdir()
    valid, _, suggestion = check_dataset_name(str(tmp_path), "task_001")
    assert not valid and suggestion == "task_002"
    recorder = RecorderController()
    settings = RecordSettings(str(tmp_path), "task_001", "example/task_001", "pick", 30, 10)
    started, _ = recorder.start(settings)
    assert not started
    assert recorder._thread is None


@pytest.mark.parametrize("name", ["", "  ", "a/b", "a\\b"])
def test_invalid_dataset_names_are_rejected(tmp_path, name):
    valid, _, _ = check_dataset_name(str(tmp_path), name)
    assert not valid


def test_homing_placeholder_remains_non_actuating():
    ok, message = RecorderController().arm_home()
    assert not ok
    assert "not implemented" in message


def test_collection_defaults_remain_30_hz_and_local_only(tmp_path):
    settings = RecordSettings(str(tmp_path), "task_001", "example/task_001", "pick", 30, 10)
    assert settings.fps == 30
    assert settings.video
    assert not settings.push_to_hub
