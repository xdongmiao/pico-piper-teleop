# Capture console

The Streamlit console runs the existing Pico/PiPER LeRobot recording workflow.

- [Installation and local camera configuration](../docs/INSTALL.md)
- [Operation guide and checker limits (中文)](../docs/OPERATIONS.md)
- [Runtime architecture and behavior boundaries](../docs/ARCHITECTURE.md)
- [Source preservation checks](../docs/behavior-baseline.sha256.json)

Start with `bash /projects/pico_piper_streamlit_app/run_streamlit.sh` in the prepared robot environment and open http://localhost:8501.

The three pages are device setup/scanning, data collection, and collection results. One stage produces one LeRobot dataset. Completed episodes pass through reset waiting and save/discard confirmation; the separate in-recording discard-and-retake control immediately clears the current buffer. The arm-homing button reports that homing is unimplemented. Reset waiting does not home or pause the arms.

Recording targets 30 Hz; live panels refresh every 0.5 s. The idle preview loop targets 20 Hz and resizes frames to 320 × 240; its camera request is 640 × 480 at 30 fps. Quality verdicts advise the operator and do not disable saving. The results checker summarizes available files and applies limited rules; its success flag does not certify complete dataset integrity.

Presentation lives in `console_theme.py`, `console.css`, and `.streamlit/config.toml`. Explanatory page text follows the implementation; widget keys, decisions, process management, recorder, telemetry and dataset-checking logic are preserved. Runtime camera settings belong in ignored `camera_config.json`; create it from the example only if it does not already exist, then configure the local rig.
