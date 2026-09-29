# Điều khiển UR3e bằng LLM và MoveIt 2

Project ROS 2 mô phỏng UR3e trong Gazebo, nhận câu lệnh tự nhiên qua Gemini,
chuyển câu lệnh thành JSON plan đã giới hạn, kiểm tra plan rồi thực thi bằng
MoveIt 2. Bài được cá nhân hóa theo MSSV `23020730`.

## Chức năng chính

- tạo scene gồm bàn, ba khối màu và ba vùng đích;
- hỗ trợ các skill `pick`, `place` và `home` qua ROS 2 Action;
- chỉ cho phép LLM sinh plan mức cao, không cho LLM gửi góc khớp hoặc trajectory;
- kiểm tra object, zone và thứ tự skill trước khi robot chuyển động;
- đưa robot về `home` sau mỗi cặp gắp/thả để hạn chế đổi nhánh IK và quay cổ tay;
- chọn nghiệm IK theo cả pose gắp và vùng thả kế tiếp để tránh chuyển động gần 360°.

Ánh xạ cá nhân hóa:

| Vật | Vùng đích |
|---|---|
| `red_cube` | `zone_a` |
| `yellow_cube` | `zone_b` |
| `blue_cube` | `zone_c` |

## Yêu cầu

- Ubuntu 22.04 và ROS 2 Humble;
- Universal Robots ROS 2 Gazebo Simulation;
- MoveIt 2 và `ros2_control`;
- Python package `google-genai` nếu chạy với Gemini API.

## Biên dịch

```bash
cd ~/Interaction/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash

python3 -m pip install --user google-genai
colcon build --symlink-install \
  --packages-select ur3_llm_control ur3_robot_skills
source install/setup.bash
```

## Chạy toàn bộ

Tạo API key tại [Google AI Studio](https://aistudio.google.com/app/apikey), sau
đó nhập key ẩn trong terminal:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
```

Lệnh dưới đây khởi động Gazebo, MoveIt, RViz, scene, robot skill server, gọi
Gemini, validate plan và thực thi toàn bộ chuỗi cá nhân hóa:

```bash
ros2 launch ur3_robot_skills end_to_end.launch.py \
  command:='Sắp xếp tất cả các khối theo mã sinh viên của tôi.'
```

Khi chạy thành công, terminal kết thúc bằng:

```text
TASK SUCCESS
```

Không ghi API key vào README, YAML, launch file hoặc source code. Nếu chỉ cần
kiểm tra pipeline offline, thêm `planner:=mock` vào lệnh chạy tổng.

## Luồng xử lý

```text
Câu lệnh tự nhiên
  -> Gemini / Mock planner
  -> JSON plan
  -> Plan Validator
  -> Skill Executor
  -> ROS 2 Action /execute_skill
  -> MoveIt 2
  -> joint_trajectory_controller
  -> UR3e trong Gazebo
```

MoveIt 2 chịu trách nhiệm tính IK, kiểm tra collision, giới hạn khớp và tạo
trajectory. Cube được attach/detach trong Planning Scene và chỉ đồng bộ pose
cuối sang Gazebo sau khi thả.

## Cấu trúc chính

```text
ur3_llm_control/       Planner, validator, scene và cấu hình MSSV
ur3_robot_skills/      Action server, MoveIt skills và launch tổng
```

Chi tiết từng thành phần nằm tại
[`ur3_llm_control/README.md`](ur3_llm_control/README.md) và
[`ur3_robot_skills/README.md`](ur3_robot_skills/README.md).

## Kiểm thử

```bash
colcon test --packages-select ur3_llm_control ur3_robot_skills
colcon test-result --verbose
```

Phiên bản hiện tại đã vượt qua `88/88` test và đã chạy thành công chuỗi
`red_cube -> zone_a`, `yellow_cube -> zone_b`, `blue_cube -> zone_c`.
