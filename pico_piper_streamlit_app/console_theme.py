"""Presentation-only styling shared by the existing Streamlit pages."""

from pathlib import Path

import streamlit as st


def apply_console_theme() -> None:
    """Apply the console skin without changing widgets or runtime state."""
    css = Path(__file__).with_name("console.css").read_text(encoding="utf-8")
    st.markdown(f"<style>{css}</style>", unsafe_allow_html=True)
    st.markdown(
        '<div class="capture-masthead">'
        '<div class="capture-brand"><span class="capture-mark">P</span>'
        '<div><strong>PICO <span>×</span> PiPER</strong>'
        '<small>TELEOPERATION / DATA CAPTURE</small></div></div>'
        '<span class="capture-edition">CAPTURE CONSOLE <b>01</b></span>'
        '</div>',
        unsafe_allow_html=True,
    )
