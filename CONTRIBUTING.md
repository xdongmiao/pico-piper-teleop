# Contributing

Open an issue describing the environment, expected behavior, and a reproducible example. For robot issues include ROS distribution, SDK versions and relevant topic/quality diagnostics, with local identities and secrets removed.

Use Python 3.12 for the collection stack. Install `requirements-dev.txt`, then run `python scripts/check_release.py` and `python -m pytest tests -q`. Native/ROS changes additionally require a workspace build and operator-supervised hardware validation; record what was actually tested in the pull request.

Keep presentation changes in `console_theme.py`, `console.css` and the theme configuration where possible. Preserve widget keys, recording decisions, topic names, timing, no-op `send_action`, and dataset semantics unless a change explicitly requires otherwise. When changing a baseline file, explain whether it changes behavior or only display text, verify that distinction, and update its hash intentionally.

Keep the English/Chinese root READMEs, [installation guide](docs/INSTALL.md), [operation guide](docs/OPERATIONS.md), [architecture/source map](docs/ARCHITECTURE.md), component READMEs and UI explanations aligned. Distinguish requested capture rates from browser refresh, freshness diagnostics from action selection, and dataset inspection from full integrity validation. Keep runtime claims linked to the responsible source; document what checks actually enforce. Upstream component READMEs do not replace the integration guide.

Do not commit recordings, weights, virtual environments, Docker exports, SDK binaries, local camera JSON, credentials or private network/device identities. Preserve upstream licenses and notices. Contributions to integration code use Apache-2.0; changes inside vendored components must respect the component's existing license.
