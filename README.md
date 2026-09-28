# Bài thực hành UR3e với ROS 2 và MoveIt 2

Repository hiện gồm hai nội dung:

- `ur_letter_writer`: điều khiển UR3e vẽ chữ D bằng MoveIt 2;
- `ur3_llm_control`: môi trường bàn, ba cube và ba vùng đích để phát triển
  điều khiển robot bằng ngôn ngữ tự nhiên.

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
