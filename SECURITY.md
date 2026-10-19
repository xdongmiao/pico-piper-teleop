# Security

Report vulnerabilities through this repository's GitHub private vulnerability reporting when available, or contact the maintainer through their GitHub profile before sharing exploit details. Do not include secrets, private camera/device identifiers, or access tokens in public issues.

The Streamlit console can start/stop robot services and write datasets. Starting the control stack can enable robot movement; the arm-homing button itself remains an unimplemented, non-actuating placeholder. Use the console on a trusted robot workstation/network. The existing TCP bridge is unauthenticated and intended for loopback use; retain its loopback binding. The application is not a multi-user authenticated remote robot-control service.

Offline CI and release checks do not move hardware. Do not run control or recovery commands against a live rig without an operator managing that rig.
