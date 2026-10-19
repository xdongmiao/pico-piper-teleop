from __future__ import annotations

import streamlit as st

from dataset_validate import validate_dataset


st.title("采集结果")

default_path = st.session_state.get("last_dataset_path", "/projects/lerobot/NX/test/pico_piper_10_smoke")
dataset_path = st.text_input("Dataset path", value=default_path)

if st.button("校验数据集", type="primary"):
    result = validate_dataset(dataset_path)
    if result["ok"]:
        st.success("Dataset validation passed.")
    else:
        st.error("Dataset validation failed.")
        for error in result["errors"]:
            st.write(error)

    st.subheader("Info")
    st.json(result.get("info", {}))
    st.subheader("Features")
    st.json(result.get("features", {}))
    st.subheader("Episode counts")
    st.json(result.get("episode_counts", {}))
    st.subheader("Timing / validity stats")
    st.json(result.get("stats", {}))
    st.subheader("Videos")
    st.dataframe(result.get("videos", []), width="stretch")
