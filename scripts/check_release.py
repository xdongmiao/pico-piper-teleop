"""Hardware-free integrity and public-source hygiene checks."""

from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
FIRST_PARTY = ("pico_piper_streamlit_app", "lerobot_robot_pico_piper", "lerobot_teleoperator_pico_vr")


def normalized_digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> int:
    errors = []
    baseline = json.loads((ROOT / "docs/behavior-baseline.sha256.json").read_text(encoding="utf-8"))
    for relative, expected in baseline.items():
        path = ROOT / relative
        if not path.is_file() or normalized_digest(path) != expected:
            errors.append(f"Behavior baseline changed: {relative}")

    compiled = 0
    for folder in FIRST_PARTY:
        for path in (ROOT / folder).rglob("*.py"):
            if "__pycache__" not in path.parts:
                try:
                    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
                    compiled += 1
                except SyntaxError as exc:
                    errors.append(f"Python syntax error: {path.relative_to(ROOT)}: {exc}")

    if (ROOT / "pico_tele_agx/src/arm_interfaces").exists():
        errors.append("Unlicensed, unused message package must not be distributed")
    if (ROOT / ".streamlit/config.toml").read_bytes() != (ROOT / "pico_piper_streamlit_app/.streamlit/config.toml").read_bytes():
        errors.append("Root and application theme configurations differ")

    result = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, capture_output=True, check=False)
    if result.returncode:
        errors.append("Run this check from a Git checkout")
        paths = []
    else:
        paths = [ROOT / name for name in result.stdout.decode("utf-8").split("\0") if name]
        if not paths:
            errors.append("No tracked files found; stage the release source before checking")
        tracked = {path.relative_to(ROOT).as_posix() for path in paths}
        for relative in baseline:
            if relative not in tracked:
                errors.append(f"Behavior source not tracked: {relative}")

    credentials = [
        re.compile(r"gh[pousr]_[A-Za-z0-9]{36,}"),
        re.compile(r"github_pat_[A-Za-z0-9_]{40,}"),
        re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
        re.compile(r"AKIA[A-Z0-9]{16}"),
    ]
    text_suffixes = {".py", ".cpp", ".h", ".sh", ".json", ".toml", ".yaml", ".yml", ".md", ".mdx", ".xml", ".txt"}
    forbidden_suffixes = {".deb", ".gz", ".tgz", ".zip", ".bag", ".db3", ".mcap", ".parquet", ".pt", ".pth", ".safetensors", ".so", ".dll"}
    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        if not path.is_file():
            errors.append(f"Missing tracked file: {relative}")
            continue
        with path.open("rb") as source:
            if source.read(64).startswith(b"version https://git-lfs.github.com/spec/v1"):
                errors.append(f"Unresolved Git LFS pointer: {relative}")
        if path.stat().st_size > 50_000_000:
            errors.append(f"Oversized source file: {relative}")
        if path.suffix.lower() in forbidden_suffixes or relative.endswith("/camera_config.json"):
            errors.append(f"Runtime artifact/local configuration tracked: {relative}")
        if path.suffix.lower() not in text_suffixes:
            continue
        content = path.read_text(encoding="utf-8", errors="replace")
        if any(pattern.search(content) for pattern in credentials):
            errors.append(f"Potential credential in: {relative}")
        if ("/home/" + "zme") in content:
            errors.append(f"Original machine path in: {relative}")
        if relative.startswith(FIRST_PARTY) or "/" not in relative:
            if re.search(r"CH\d{8,}", content):
                errors.append(f"Original device identity in: {relative}")

    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Verified {len(baseline)} behavior baselines, {compiled} Python modules, {len(paths)} tracked source files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
