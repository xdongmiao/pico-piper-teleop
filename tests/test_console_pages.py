"""Render the real pages with background hardware probing disabled in tests."""

from pathlib import Path

import pytest
from streamlit.testing.v1 import AppTest

import runtime
from process_manager import ProcessManager

APP = Path(__file__).resolve().parents[1] / "pico_piper_streamlit_app"


@pytest.fixture(autouse=True)
def no_hardware_workers(monkeypatch):
    monkeypatch.setattr(runtime.AppRuntime, "ensure_background", lambda self: None)
    monkeypatch.setattr(ProcessManager, "_external_pids", lambda self, spec: [])


def test_main_page_renders_theme_and_original_device_controls():
    app = AppTest.from_file(str(APP / "app.py"), default_timeout=30).run()
    assert not app.exception
    assert any("PICO" in item.value and "capture-masthead" in item.value for item in app.markdown)
    labels = {button.label for button in app.button}
    assert {"启用设备", "设备失能"} <= labels


def test_collection_page_renders_original_configuration_and_controls():
    app = AppTest.from_file(str(APP / "pages/2_Data_Collection.py"), default_timeout=30).run()
    assert not app.exception
    labels = {button.label for button in app.button}
    assert {"开始阶段采集", "结束当前条并进入确认", "丢弃当前条重采", "结束阶段采集", "机械臂复位"} <= labels
    assert {field.label for field in app.text_input} == {"保存根目录", "Dataset name", "Task"}
    by_label = {button.label: button for button in app.button}
    assert by_label["结束当前条并进入确认"].disabled
    assert by_label["丢弃当前条重采"].disabled


def test_results_page_retains_dataset_validation_entry():
    app = AppTest.from_file(str(APP / "pages/3_Collection_Results.py"), default_timeout=30).run()
    assert not app.exception
    assert [field.label for field in app.text_input] == ["Dataset path"]
    assert "校验数据集" in {button.label for button in app.button}
