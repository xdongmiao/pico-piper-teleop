#!/usr/bin/env bash
set -euo pipefail

cd /projects/pico_piper_streamlit_app
source /projects/lerobot_v051_py312/bin/activate
streamlit run app.py \
  --server.address 0.0.0.0 \
  --server.port 8501 \
  --server.headless true
