# UR3e LLM Control — Milestone 2

Package này tạo môi trường thao tác cho UR3e từ một nguồn cấu hình duy nhất:
`config/scene.yaml`.

Scene hiện gồm:

- một bàn thao tác;
- `red_cube`, `yellow_cube`, `blue_cube`;
- `zone_a`, `zone_b`, `zone_c`;
- collision floor, table và cube trong MoveIt Planning Scene;
- marker tên và vị trí của các zone trong RViz.

## Build

```bash
cd ~/Interaction/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur_simulation_gz ur3_llm_control
source install/setup.bash
```

## Chạy

```bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

Để kiểm thử không mở giao diện:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  gazebo_gui:=false launch_rviz:=false
```
