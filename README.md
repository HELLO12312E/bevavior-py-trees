# bevavior-py-trees

This repository contains ROS 2 Python packages that use [py_trees](https://github.com/splintered-reality/py_trees) for behavior-tree-based robot control.

## Packages

### `bt_interaction_nav`

A reactive behavior tree that:

1. **Approaches person A** (detected on `/person` topic) once per run using a proportional `/cmd_vel` controller.
2. **Waits 10 seconds** after reaching person A (within `target_distance`).
3. **Navigates to goal B** using Nav2 `NavigateToPose`.

Person A is only approached once per run (tracked via `is_person_a_met` blackboard flag). After the interaction, the robot resumes navigating to goal B uninterrupted.

#### Tree structure

```
Selector(memory=False)                    [root]
├─ Sequence(memory=False)                 [person interaction]
│   ├─ CheckPersonA   – person A visible?
│   ├─ NotMetYet      – not yet met?
│   ├─ ApproachPerson – drive toward A
│   ├─ WaitSeconds(10)
│   └─ SetMetFlag     – mark as met
└─ NavigateToPoseB    – Nav2 goal B
```

#### Build & run

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select bt_interaction_nav
source install/setup.bash
ros2 launch bt_interaction_nav run.launch.py
```

Configure the goal, topic names and PID gains in `bt_interaction_nav/config/params.yaml`.

> **TODO:** replace the placeholder person-message import in `bt_interaction_nav/bt_interaction_nav/adapters.py`
> with the real message type (package + `msg.Person`) once it is known.

---

## Other notes

chạy realsese: ros2 launch realsense2_camera rs_launch.py, nếu bật point cloud: ros2 param set /camera/camera pointcloud__neon_.enable true
