# Capture console

The Streamlit console runs the existing Pico/PiPER LeRobot recording workflow.

- [Installation and local camera configuration](../docs/INSTALL.md)
- [Runtime architecture and behavior boundaries](../docs/ARCHITECTURE.md)
- [Source preservation checks](../docs/behavior-baseline.sha256.json)

Start with `bash /projects/pico_piper_streamlit_app/run_streamlit.sh` in the prepared robot environment and open http://localhost:8501.

The three original pages are device setup/scanning, data collection, and collection results. One stage produces one LeRobot dataset; each episode is confirmed before save/discard. The arm-homing button is still an unimplemented placeholder and does not send motion commands.

Presentation lives in `console_theme.py`, `console.css`, and `.streamlit/config.toml`. The page widgets, process management, recorder, telemetry, and dataset validation retain the existing implementation. Runtime camera settings belong in ignored `camera_config.json`, copied from the example and configured for the local rig.
