# UR letter writer

This ROS 2 Humble package uses MoveIt 2 to draw the letter **D** with the
`tool0` frame of a simulated UR3 or UR3e.

The letter is defined as two strokes in a vertical YZ plane:

1. A vertical stem from top to bottom.
2. A half-ellipse from the top of the stem to its bottom.

Between the strokes, the tool is lifted along the plane normal. Green RViz
markers show the intended strokes and orange markers show the measured
`tool0` trace.

The initial approach is planned against a collision floor. The writer samples
several collision-checked plans, rejects a plan when any joint accumulates more
than `max_joint_travel`, and executes the plan with the least total joint
motion. Each drawing motion must also produce a complete Cartesian path, stay
inside the joint limits, and remain below the configured maximum joint step.

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur_simulation_gz ur_letter_writer
source install/setup.bash
```

Run UR3e, Gazebo, MoveIt, RViz, and the writer node:

```bash
ros2 launch ur_letter_writer write_letter_d.launch.py
```

RViz starts with `rviz/letter_show.rviz` by default. A different configuration
can be selected with `rviz_config_file:=/absolute/path/to/config.rviz`.
The `Letter Path` display is preconfigured for `/letter_path_markers`. A
dedicated marker node keeps the intended path and topic available until the
launch is stopped, independently of whether the drawing node succeeds.

For a headless run without Gazebo and RViz windows:

```bash
ros2 launch ur_letter_writer write_letter_d.launch.py \
  gazebo_gui:=false launch_rviz:=false
```

To use the UR3 instead:

```bash
ros2 launch ur_letter_writer write_letter_d.launch.py ur_type:=ur3
```

If the initial drawing pose is not reachable, adjust `plane_x`, `center_y`,
`center_z`, or the fixed orientation in `config/letter_d.yaml`. Add a Marker
display in RViz with topic `/letter_path_markers` to see the intended and
measured paths.
