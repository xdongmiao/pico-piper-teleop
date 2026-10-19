from __future__ import annotations

import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def validate_dataset(root: str | Path) -> dict[str, Any]:
    root = Path(root)
    out: dict[str, Any] = {"root": str(root), "ok": False, "errors": []}
    info_path = root / "meta" / "info.json"
    if not info_path.exists():
        out["errors"].append(f"Missing {info_path}")
        return out
    info = json.loads(info_path.read_text())
    out["info"] = {
        "robot_type": info.get("robot_type"),
        "total_episodes": info.get("total_episodes"),
        "total_frames": info.get("total_frames"),
        "fps": info.get("fps"),
    }
    features = info.get("features", {})
    out["features"] = {
        "state_shape": features.get("observation.state", {}).get("shape"),
        "action_shape": features.get("action", {}).get("shape"),
        "images": [key for key, value in features.items() if key.startswith("observation.images")],
    }

    parquet_paths = sorted((root / "data").glob("chunk-*/*.parquet"))
    out["parquet_files"] = len(parquet_paths)
    if parquet_paths:
        table = pa.concat_tables([pq.read_table(path) for path in parquet_paths])
        out["parquet_rows"] = table.num_rows
        out["episode_counts"] = _episode_counts(table)
        out["stats"] = _state_stats(table, features.get("observation.state", {}).get("names") or [])

    video_paths = sorted((root / "videos").glob("observation.images.*/*/*.mp4"))
    out["video_count"] = len(video_paths)
    out["videos"] = [_ffprobe(root, path) for path in video_paths]
    expected_video_count = len(out["features"]["images"])
    if expected_video_count and out["video_count"] != expected_video_count:
        out["errors"].append(f"Expected {expected_video_count} videos, found {out['video_count']}")
    out["ok"] = not out["errors"]
    return out


def _episode_counts(table: pa.Table) -> dict[str, int]:
    episodes = table.column("episode_index").combine_chunks().to_pylist()
    counts: dict[str, int] = {}
    for episode in episodes:
        key = str(int(episode))
        counts[key] = counts.get(key, 0) + 1
    return counts


def _state_stats(table: pa.Table, names: list[str]) -> dict[str, dict[str, float]]:
    if "observation.state" not in table.column_names:
        return {}
    state = np.asarray(table.column("observation.state").combine_chunks().to_pylist(), dtype=np.float32)
    wanted = [
        "loop_dt_ms",
        "schedule_lag_ms",
        "cam_top_age_ms",
        "cam_left_wrist_age_ms",
        "cam_right_wrist_age_ms",
        "cam_top_valid",
        "cam_left_wrist_valid",
        "cam_right_wrist_valid",
        "left_joint_age_ms",
        "right_joint_age_ms",
        "xr_age_ms",
        "left_action_age_ms",
        "right_action_age_ms",
        "bridge_valid",
        "left_control_active",
        "right_control_active",
        "left_action_from_ik",
        "right_action_from_ik",
        "left_action_from_hold",
        "right_action_from_hold",
        "left_action_semantic_valid",
        "right_action_semantic_valid",
        "action_semantic_valid",
    ]
    stats = {}
    for name in wanted:
        if name not in names:
            continue
        values = state[:, names.index(name)]
        finite = values[np.isfinite(values) & (values < 999999)]
        if finite.size == 0:
            continue
        stats[name] = {
            "min": round(float(finite.min()), 3),
            "mean": round(float(finite.mean()), 3),
            "max": round(float(finite.max()), 3),
        }
    return stats


def _ffprobe(root: Path, path: Path) -> dict[str, Any]:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,avg_frame_rate,duration,nb_frames",
        "-of",
        "csv=p=0",
        str(path),
    ]
    result = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    values = result.stdout.strip().split(",") if result.returncode == 0 else []
    return {
        "path": str(path.relative_to(root)),
        "width": values[0] if len(values) > 0 else None,
        "height": values[1] if len(values) > 1 else None,
        "avg_frame_rate": values[2] if len(values) > 2 else None,
        "duration": values[3] if len(values) > 3 else None,
        "nb_frames": values[4] if len(values) > 4 else None,
        "error": result.stderr.strip() if result.returncode != 0 else None,
    }
