# Kỹ năng robot UR3e — Milestone 3

Package này xây dựng bốn kỹ năng điều khiển trên MoveIt 2:

- `home()` đưa robot về cấu hình reset gập an toàn, tránh singularity;
- `move_above(object)` di chuyển `tool0` tới pose an toàn phía trên cube;
- `pick(object)` hạ theo Cartesian, đóng gripper logic, attach cube và nâng lên;
- `place(object, zone)` nâng cube lên mặt phẳng chuyển tiếp an toàn, di chuyển tới
  vùng đích, hạ cube, detach và đồng bộ
  pose cuối giữa MoveIt Planning Scene với Gazebo.

Các lệnh được nhận qua action `/execute_skill`, khai báo trong
`action/ExecuteSkill.action`. Kết quả gồm mã và tên trạng thái `SUCCESS`,
`FAILED`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED`,
`EXECUTION_FAILED` hoặc `INVALID_SKILL`.

Quỹ đạo được kiểm tra giới hạn khớp, bước nhảy khớp, tổng góc di chuyển và va
chạm trước khi gửi tới `joint_trajectory_controller`. Nghiệm IK được quy đổi
về biểu diễn gần trạng thái khớp hiện tại; mỗi trajectory giới hạn tổng hành
trình 5.0 rad cho khớp wrist và 5.5 rad cho các khớp còn lại để loại nhánh gần
một vòng 360 độ. Skill `home`
dùng named state `test_configuration` của UR làm safe-home vì state `home` thẳng
mặc định có singularity và tạo quãng quay cổ tay lớn sau khi thả ở zone B. Các
chuyển động hạ/nâng dùng Cartesian path. Nếu Cartesian descent ở mặt phẳng chuyển tiếp
không khả thi, MoveIt replanning tới cùng pose rồi mới hạ tiếp. Nếu một bước
thất bại, executor không gửi bước kế tiếp.

Ba cube nằm trên hàng `x = 0.20 m` và đối diện zone cùng màu trên hàng
`x = 0.32 m`: đỏ/A cùng `y = 0.18 m`, vàng/B cùng `y = 0.06 m`, xanh/C cùng
`y = -0.06 m`. Các tọa độ này được giữ cố định. Khả năng tiếp cận được xử lý
bằng orientation gắp theo từng object, pose thả thẳng đứng và độ nâng chuyển
tiếp theo object trong `config/robot_skills.yaml`. Executor truyền nội bộ zone
của bước `place` kế tiếp vào `pick`; skill server thử các nghiệm IK và chọn nhánh
có Cartesian transfer an toàn tới đúng zone đó. JSON plan vẫn giữ dạng
`pick -> place -> home`.

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

Launch file khởi động UR3e, Gazebo, MoveIt, RViz, scene, skill server và mặc
định chạy `home → move_above(red_cube) → pick(red_cube) → place(red_cube,
zone_a)`. Thành công được xác nhận bằng dòng:

```text
MILESTONE 3 SUCCESS: red_cube was placed in zone_a
```

Có thể chọn cube và zone khác ngay trên lệnh launch. Ví dụ chuyển
`yellow_cube` tới `zone_a`:

```bash
ros2 launch ur3_robot_skills robot_skills.launch.py \
  demo_object:=yellow_cube demo_zone:=zone_a
```

Để chỉ khởi động action server mà không chạy demo tự động, dùng launch
argument `run_demo:=false`.


## Chạy câu lệnh tự nhiên end-to-end

Tạo key tại [Google AI Studio](https://aistudio.google.com/app/apikey), sau đó
nhập key ẩn trong terminal và chạy launch tổng:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY

ros2 launch ur3_robot_skills end_to_end.launch.py \
  command:='Đưa khối đỏ sang vùng B.'
```

Gemini với model `gemini-3.5-flash-lite` là mặc định. Dùng `planner:=mock` để
kiểm tra offline hoặc `planner:=openai model:=gpt-4o-mini` nếu muốn dùng lại
OpenAI. Mọi chế độ đều sinh JSON, validate toàn bộ plan, rồi mới gọi action
`/execute_skill`.
