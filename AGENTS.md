# Project Rules

- Known-good baseline: `src/my_epuck_project/launch/robot_d500_launch_port23000.py`
- Do not overwrite the known-good baseline.
- Use small isolated changes only.
- Webots default port 1234 is blocked; use port 23000.
- WSL reaches Windows Webots at 127.0.0.1:23000.
- Use official webots_ros2 patterns: WebotsLauncher, WebotsController, WaitForControllerConnection.
- No central coordinator/brain for the thesis.
- PC/RViz is visualization only.
- Each robot must eventually have independent namespace, odom, scan, cmd_vel, map, and TF tree.
- Never create TF loops between base_footprint and base_link.
- Before editing code, inspect and explain the minimal diff.
- After editing, build only my_epuck_project and provide exact test commands.
