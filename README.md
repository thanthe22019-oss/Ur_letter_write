# Bài thực hành UR3e với ROS 2 và MoveIt 2

Repository hiện gồm ba nội dung:

- `ur_letter_writer`: điều khiển UR3e vẽ chữ D bằng MoveIt 2;
- `ur3_llm_control`: môi trường bàn, ba cube và ba vùng đích để phát triển
  điều khiển robot bằng ngôn ngữ tự nhiên;
- `ur3_robot_skills`: các kỹ năng MoveIt 2 có kiểm tra va chạm để robot về
  home, tiếp cận, gắp và đặt cube.

## Bài vẽ chữ D

Package ROS 2 Humble này sử dụng MoveIt 2 để điều khiển khung `tool0` của
robot UR3 hoặc UR3e mô phỏng vẽ chữ **D**.

Chữ D được tạo bởi hai nét trong mặt phẳng YZ thẳng đứng:

1. Nét thẳng đứng từ trên xuống dưới.
2. Nét cong nửa elip từ đầu trên đến đầu dưới của nét thẳng.

Giữa hai nét, đầu công tác được nâng khỏi mặt phẳng vẽ trước khi chuyển sang
vị trí mới. Các Marker màu xanh lá biểu diễn quỹ đạo dự kiến và Marker màu
cam biểu diễn quỹ đạo thực tế của `tool0`.

## Biên dịch

Chạy tại thư mục gốc của repository:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur_simulation_gz ur_letter_writer
source install/setup.bash
```

## Chạy chương trình

Lệnh dưới đây khởi động UR3e, Gazebo, MoveIt, RViz và node vẽ chữ D:

```bash
ros2 launch ur_letter_writer write_letter_d.launch.py
```

## Môi trường điều khiển bằng ngôn ngữ tự nhiên

Milestone 2 tạo một bàn thao tác, `red_cube`, `yellow_cube`, `blue_cube` và
ba vùng `zone_a`, `zone_b`, `zone_c`. Gazebo và MoveIt Planning Scene cùng
đọc pose, kích thước và màu sắc từ `ur3_llm_control/config/scene.yaml`.

Biên dịch package:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur_simulation_gz ur3_llm_control
source install/setup.bash
```

Khởi động toàn bộ UR3e, Gazebo, MoveIt, RViz và scene:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py
```

## Milestone 3: kỹ năng gắp và đặt

Package `ur3_robot_skills` cung cấp action `/execute_skill` với các kỹ năng
`home`, `move_above`, `pick` và `place`. Mỗi kỹ năng trả về trạng thái rõ ràng:
`SUCCESS`, `FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED` hoặc
`EXECUTION_FAILED`. Chương trình dừng ngay khi một bước lập kế hoạch hay thực
thi thất bại.

Biên dịch hai package của Milestone 2 và 3:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control ur3_robot_skills
source install/setup.bash
```

Khởi động toàn bộ mô phỏng và chạy tự động chuỗi
`home → move_above(red_cube) → pick(red_cube) → place(red_cube, zone_b)`:

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py
```

Khi hoàn thành, terminal hiển thị:

```text
MILESTONE 3 SUCCESS: red_cube was placed in zone_b
```
