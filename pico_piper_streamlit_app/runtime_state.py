from __future__ import annotations

import atexit

import streamlit as st

from runtime import create_runtime


@st.cache_resource
def get_runtime():
    runtime = create_runtime()
    atexit.register(runtime.shutdown)
    return runtime
