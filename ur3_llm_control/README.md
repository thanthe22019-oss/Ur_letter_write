# UR3e LLM Control — Scene, Validator và Skill Executor

Package này cung cấp tầng điều phối an toàn cho UR3e. Luồng hiện tại là:

```text
Câu lệnh tự nhiên
  -> Gemini API (mặc định), OpenAI API hoặc MockLLMPlanner
  -> JSON plan có Structured Outputs
  -> task_validator.py
  -> skill_executor.py
  -> robot_skills.py
  -> /execute_skill
  -> MoveIt 2
  -> joint_trajectory_controller
```

JSON chỉ được chọn `pick`, `place`, `home`. Nó không thể gửi joint position,
trajectory, velocity, torque, effort hoặc lệnh controller. MoveIt 2 là thành
phần duy nhất lập trajectory.

## Bố cục source

```text
ur3_llm_control/
├── config/
│   ├── scene.yaml
│   ├── student_config.yaml
│   └── milestone5_plan.json
├── launch/
│   └── llm_robot.launch.py
└── ur3_llm_control/
    ├── scene_config.py
    ├── scene_manager.py
    ├── student_config.py
    ├── task_validator.py
    ├── llm_planner.py
    ├── planner_contract.py
    ├── gemini_planner.py
    ├── openai_planner.py
    ├── skill_executor.py
    └── robot_skills.py
```

Milestone 6 dùng `llm_planner.py` làm mock planner xác định. Milestone 7 dùng
`gemini_planner.py` gọi Gemini API với Structured Outputs; OpenAI được giữ làm
lựa chọn phụ. Milestone 8 mở rộng `execute_plan` thành bộ điều phối hoàn chỉnh;
mọi nhánh đều bắt buộc đi qua Planner, Validator rồi mới tới Executor.

`robot_skills.py` chỉ chuyển một `PlanStep` đã validate sang ba field của
action `ExecuteSkill`. Chuyển động thật nằm trong package
`ur3_robot_skills`, nơi MoveIt kiểm tra IK, collision, giới hạn khớp và lập
trajectory.

## Scene và cá nhân hóa

`config/scene.yaml` là nguồn tọa độ duy nhất cho bàn, ba cube và ba zone.
Với MSSV `23020730`, `P = 30 mod 6 = 0`:

- `red_cube -> zone_a`;
- `yellow_cube -> zone_b`;
- `blue_cube -> zone_c`.

Các cube nằm trên hàng `x = 0.20 m`, đối diện zone cùng màu trên hàng
`x = 0.32 m`. Việc sửa IK và trajectory không thay đổi các tọa độ này.

## Build

```bash
cd ~/Interaction/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select ur3_llm_control ur3_robot_skills
source install/setup.bash
```

## Milestone 4 — kiểm tra plan

```bash
ros2 run ur3_llm_control validate_plan --json \
  '{"plan":[{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_a"},{"skill":"home"}]}'
```

Kết quả hợp lệ:

```text
VALIDATION: SUCCESS
1. pick(red_cube)
2. place(red_cube, zone_a)
3. home()
```

## Milestone 5 — chạy plan tĩnh

Lệnh tổng sau khởi động UR3e, Gazebo, MoveIt, scene, action server và tự chạy
`config/milestone5_plan.json`:

```bash
ros2 launch ur3_robot_skills validated_plan.launch.py
```

Chạy không giao diện để test:

```bash
ros2 launch ur3_robot_skills validated_plan.launch.py \
  gazebo_gui:=false launch_rviz:=false
```

Có thể dùng một file JSON khác:

```bash
ros2 launch ur3_robot_skills validated_plan.launch.py \
  plan_file:=/duong/dan/plan.json
```

Executor validate toàn bộ file trước khi kết nối robot, gọi từng skill theo
thứ tự và dừng ngay ở kết quả đầu tiên khác `SUCCESS`. Kết quả thành công:

```text
pick(red_cube) .......................... SUCCESS
place(red_cube, zone_a) ................. SUCCESS
home() .................................. SUCCESS

TASK SUCCESS
```

Robot skill không dùng vòng lặp `/set_pose` để ép cube chạy theo
end-effector. Cube được attach/detach trong MoveIt Planning Scene và chỉ đồng
bộ pose cuối sang Gazebo sau khi thả, tránh hiện tượng cube giật liên tục giữa
đầu gắp và mặt bàn.


## Milestone 6 — Mock LLM Planner

Mock planner nhận câu tự nhiên, sinh JSON thuần và tự đưa JSON qua Validator
trước khi trả kết quả. Ba câu bắt buộc:

```bash
ros2 run ur3_llm_control mock_plan   'Đưa vật màu đỏ sang vùng B.'

ros2 run ur3_llm_control mock_plan   'Hãy lấy khối màu vàng và đặt nó vào ô A.'

ros2 run ur3_llm_control mock_plan   'Move the blue cube to zone C.'
```

Lệnh nâng cao theo MSSV:

```bash
ros2 run ur3_llm_control mock_plan   'Arrange all objects according to my student ID.'
```

Với MSSV `23020730`, planner tính bằng code và sinh thứ tự:

```text
pick(red_cube) -> place(red_cube, zone_a) -> home()
pick(yellow_cube) -> place(yellow_cube, zone_b) -> home()
pick(blue_cube) -> place(blue_cube, zone_c) -> home()
```

Validator bắt buộc mỗi cặp `pick -> place` phải theo ngay bằng `home`. Nhờ đó
lượt gắp tiếp theo luôn bắt đầu từ cùng một cấu hình khớp an toàn, thay vì từ
tư thế gập còn lại sau khi thả khối trước. Skill `home` sử dụng safe-home gập
không singularity (`test_configuration` trong SRDF của UR).

Chạy trực tiếp toàn bộ plan cá nhân hóa:

```bash
ros2 launch ur3_robot_skills validated_plan.launch.py \
  plan_file:=$(ros2 pkg prefix ur3_llm_control)/share/ur3_llm_control/config/mssv_23020730_plan.json
```

Prompt an toàn nằm trong `prompts/planner_prompt.txt`. Nếu planner hoặc
Validator lỗi, robot không nhận bất kỳ action goal nào.

## Milestone 7 — Gemini API Planner

Tạo key tại [Google AI Studio](https://aistudio.google.com/app/apikey). Nhập key
ẩn trong terminal để không lưu key vào history, YAML, launch file hay source:

```bash
read -rsp "Nhập Gemini API key: " GEMINI_API_KEY
echo
export GEMINI_API_KEY
export GEMINI_MODEL="gemini-3.5-flash-lite"   # tùy chọn
```

Cài SDK nếu máy chưa có và build lại workspace:

```bash
python3 -m pip install --user google-genai
cd ~/Interaction/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select ur3_llm_control ur3_robot_skills
source install/setup.bash
```

Chỉ gọi Gemini để sinh và validate plan, chưa điều khiển robot:

```bash
ros2 run ur3_llm_control gemini_plan \
  'Đưa khối đỏ sang vùng B.'
```

Client yêu cầu JSON theo schema đóng rồi tiếp tục đưa kết quả qua
`task_validator.py`. Structured Outputs không thay thế lớp kiểm tra an toàn.
Khi mạng hoặc API lỗi, planner dừng. Chỉ bật mock fallback khi chủ động muốn
kiểm tra offline:

```bash
ros2 run ur3_llm_control gemini_plan \
  --fallback-to-mock \
  'Arrange all objects according to my student ID.'
```

OpenAI planner cũ vẫn dùng được bằng lệnh `openai_plan` và biến
`OPENAI_API_KEY`, nhưng không còn là lựa chọn mặc định.

## Milestone 8 — chạy end-to-end từ câu tự nhiên

Lệnh tổng khởi động Gazebo, MoveIt, RViz, scene, skill server, gọi Gemini,
validate JSON rồi thực thi từng skill. `planner:=gemini` và model bên dưới đã
là mặc định nên có thể bỏ hai tham số đó:

```bash
ros2 launch ur3_robot_skills end_to_end.launch.py \
  command:='Arrange all objects according to my student ID.' \
  planner:=gemini \
  model:=gemini-3.5-flash-lite
```

Kiểm tra toàn bộ pipeline không tốn API bằng mock planner:

```bash
ros2 launch ur3_robot_skills end_to_end.launch.py \
  command:='Arrange all objects according to my student ID.' \
  planner:=mock
```

Có thể bật fallback rõ ràng bằng `fallback_to_mock:=true`. Terminal sẽ in theo
thứ tự `USER COMMAND`, `LLM PLAN`, `VALIDATION`, `EXECUTION` và kết thúc bằng
`TASK SUCCESS` hoặc `TASK FAILED`.
