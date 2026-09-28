# Kỹ năng robot UR3e — Milestone 3

Package này xây dựng bốn kỹ năng điều khiển trên MoveIt 2:

- `home()` đưa robot về cấu hình home;
- `move_above(object)` di chuyển `tool0` tới pose an toàn phía trên cube;
- `pick(object)` hạ theo Cartesian, đóng gripper logic, attach cube và nâng lên;
- `place(object, zone)` di chuyển tới vùng đích, hạ cube, detach và đồng bộ
  pose cuối giữa MoveIt Planning Scene với Gazebo.

Các lệnh được nhận qua action `/execute_skill`, khai báo trong
`action/ExecuteSkill.action`. Kết quả gồm mã và tên trạng thái `SUCCESS`,
`FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED`,
`EXECUTION_FAILED` hoặc `INVALID_SKILL`.

Quỹ đạo được kiểm tra giới hạn khớp, bước nhảy khớp, tổng góc di chuyển và va
chạm trước khi gửi tới `joint_trajectory_controller`. Các chuyển động hạ/nâng
dùng Cartesian path. Nếu một bước thất bại, demo không gửi bước kế tiếp.

## Biên dịch

```bash
cd ~/Interaction/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control ur3_robot_skills
source install/setup.bash
```

## Chạy toàn bộ

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py
```

Launch file khởi động UR3e, Gazebo, MoveIt, RViz, scene, skill server và demo
cố định `home → move_above(red_cube) → pick(red_cube) → place(red_cube,
zone_b)`. Thành công được xác nhận bằng dòng:

```text
MILESTONE 3 SUCCESS: red_cube was placed in zone_b
```

Để chỉ khởi động action server mà không chạy demo tự động, dùng launch
argument `run_demo:=false`.
