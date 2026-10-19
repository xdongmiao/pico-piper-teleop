"""Import the UI helpers without importing or starting the robot stack."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pico_piper_streamlit_app"))
