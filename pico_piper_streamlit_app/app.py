from __future__ import annotations

import streamlit as st

from runtime_state import get_runtime
from console_theme import apply_console_theme


st.set_page_config(page_title="Pico × PiPER | Capture Console", layout="wide")


def main() -> None:
    apply_console_theme()
    get_runtime().ensure_background()
    if hasattr(st, "Page") and hasattr(st, "navigation"):
        pages = [
            st.Page("pages/1_Device_Setup.py", title="设备启动与扫描"),
            st.Page("pages/2_Data_Collection.py", title="数据采集"),
            st.Page("pages/3_Collection_Results.py", title="采集结果"),
        ]
        st.navigation(pages).run()
    else:
        st.title("Pico Piper Capture Console")
        st.info("Use the pages in the sidebar. Upgrade Streamlit for st.navigation support.")


if __name__ == "__main__":
    main()
