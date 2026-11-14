# Agriculture_Ws — 묘목심기 로봇

| 패키지 | 하는 일 |
|---|---|
| seedling_llm | `voice_node`+`llm_node`: 웨이크워드(HELLO KERNEL) → 바로 실행 · `voice_llm_node`: UI 채팅·마이크 → 승인 카드 → 실행 (OpenAI) |
| vision | `depth_grid_node`: 빈 칸 판별 → `/vision/planting_slots` (S1~S4) |
| seedling_control | 중앙관제 `control_node`, `mock_robot_node`, `tts_node`, 전체 launch |
| robot_control | 두산 로봇 `seedling_robot` (+ move_basic 등) |
| seedling_ui | 관제 웹페이지 v2 (`ui_server` → http://localhost:8000, 브라우저 자동으로 열림) |

| UI 자리 | 비전 칸 | 실제 위치 |
|---|---|---|
| F (아랫줄 왼쪽) | S1 | C-CENTER |
| G (윗줄 왼쪽) | S2 | D-CENTER |
| I (아랫줄 오른쪽) | S3 | E-CENTER |
| H (윗줄 오른쪽) | S4 | F-CENTER |

자리를 말하지 않으면 F → G → I → H (C→D→E→F-CENTER) 순서로 빈 곳만 채움.

## 빌드
```bash
cd ~/Agriculture_Ws
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
colcon build --symlink-install && source install/setup.bash
```
(처음 한 번) `sudo apt install ros-jazzy-rosbridge-server`

## 실행
```bash
# 두산 드라이버를 먼저 켠 뒤
ros2 launch seedling_control seedling.launch.py
```
- 기본: 실제 로봇 + 카메라 · `use_mock:=true`: 가짜 로봇 · `fake_vision:=true`: 카메라 없이
- 그리퍼 노드는 e0509_simple bringup 드라이버가 켜요. dsr_bringup2 드라이버를 쓸 때만 `gripper:=true`
- 카메라 D455F(시리얼 261822308878)도 같이 켜져요. 따로 켰으면 `camera:=false`
- 로봇만 움직이지 않고 계산만: `ros2 run robot_control seedling_robot --ros-args -p check_only:=true`

## 흐름
"HELLO KERNEL" → "묘목 심어" → HOME 에서 비전 확인 → 빈 칸을 C→D→E→F-CENTER 순으로
→ 묘목 하나마다 C·D·E 순서로 (20cm 위 → 잡기 → 20cm 위 → MID → 빈 칸 20cm 위 → 놓기 → MID)
→ 다 끝나면 HOME 에서 다시 확인. 빈 칸이 없으면 HOME 대기.
잡기 전에는 빠르게, 묘목을 들고 있을 때는 천천히. 묘목 C·D·E 를 다시 채우면 UI에서 기록 초기화.

## 기타
- `data/baseline_depth.npz`: 빈 판 기준 사진 (`scripts/capture_baseline_depth.py` 로 다시 찍음)
- `seedling_ui/extra/v2_original/`: 별 님 v2 원본 (`seedling_controller.py` 는 control_node 가 대신함 — 같이 켜지 마세요)
- `seedling_ui/extra/old_v1/`: 예전 UI
