# Validation safety rules

This checkout is the only workspace authorized for Webots/ROS 2 validation.
The original checkout at `/home/arash/webots_ws` is dirty and must never be
sourced, built, or used as the runtime package prefix for this validation.

Before any ROS or Webots launch, read:

`docs/HOST_SAFETY_INCIDENT_2026-08-24.md`

The incident documented there is a hard preflight blocker. Never trust a run
merely because its command contains `--workspace`: `AMENT_PREFIX_PATH`,
`PYTHONPATH`, `CMAKE_PREFIX_PATH`, `COLCON_PREFIX_PATH`,
`RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI` determine which code and middleware
actually run.

Required preflight checks:

```bash
env -i HOME=/home/arash PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /home/arash/webots_ws_clean_validation_20260823/install/setup.bash
    source /home/arash/webots_ws_clean_validation_20260823/scripts/ros2_wsl_cyclonedds_loopback.sh
    test "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp
    test "$CYCLONEDDS_URI" = file:///home/arash/webots_ws_clean_validation_20260823/config/cyclonedds/wsl_loopback.xml
    test "$(ros2 pkg prefix my_epuck_project)" = /home/arash/webots_ws_clean_validation_20260823/install/my_epuck_project
    python3 -c "import my_epuck_project; print(my_epuck_project.__file__)"
  '
```

Do not launch if any check fails. Do not use Fast DDS as a silent fallback for
this simulation. A full Windows reboot and a healthy host baseline are required
after a Windows nonpaged-pool incident. WSL's reported available memory is not
a substitute for the Windows host counters.
