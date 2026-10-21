from __future__ import annotations

import streamlit as st

from dataset_validate import validate_dataset


st.title("采集结果")
st.caption("本页汇总元数据、行数、视频和质量统计，不逐条核对完整性。多文件视频可能触发数量误报；单个视频探测失败请查看 Videos 的 error 列。")

default_path = st.session_state.get("last_dataset_path", "/projects/lerobot/NX/test/pico_piper_10_smoke")
dataset_path = st.text_input("Dataset path", value=default_path)

if st.button("校验数据集", type="primary"):
    result = validate_dataset(dataset_path)
    if result["ok"]:
        st.success("有限检查通过；请继续核对统计与视频错误。")
    else:
        st.error("有限检查发现错误。")
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
