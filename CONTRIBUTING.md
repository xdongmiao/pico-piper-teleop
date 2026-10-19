# Contributing

Open an issue describing the environment, expected behavior, and a reproducible example. For robot issues include ROS distribution, SDK versions and relevant topic/quality diagnostics, with local identities and secrets removed.

Use Python 3.12 for the collection stack. Install `requirements-dev.txt`, then run `python scripts/check_release.py` and `python -m pytest tests -q`. Native/ROS changes additionally require a workspace build and operator-supervised hardware validation; record what was actually tested in the pull request.

Keep presentation changes in `console_theme.py`, `console.css` and the theme configuration where possible. Preserve widget keys, recording decisions, topic names, timing, no-op `send_action`, and dataset semantics unless a change explicitly requires otherwise. If changing a behavior baseline file, explain the behavior change and update its hash intentionally.

Do not commit recordings, weights, virtual environments, Docker exports, SDK binaries, local camera JSON, credentials or private network/device identities. Preserve upstream licenses and notices. Contributions to integration code use Apache-2.0; changes inside vendored components must respect the component's existing license.
