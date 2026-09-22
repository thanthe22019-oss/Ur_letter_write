# Robot UR viết chữ D

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
