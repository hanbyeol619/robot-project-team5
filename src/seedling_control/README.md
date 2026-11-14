# seedling_control — 묘목심기 중앙관제 노드

```
voice_node ─/voice/text─▶ llm_node ─/llm/command─┐
관제 UI(rosbridge) ──────────────/ui/command─────┼─▶ control_node ─/robot/task──▶ 로봇 노드 (지금은 mock_robot_node)
depth_grid_node ─────/vision/planting_slots──────┘        │       ◀─/robot/result─┘
                                                          ├─/robot/stop─▶ 로봇 노드, tts_node
                                                          ├─/ui/status──▶ 관제 UI
                                                          └─/tts/text───▶ tts_node ─▶ 스피커
```

## 실행

```bash
cd ~/Agruculture_Ws/src && cp -r <받은폴더>/seedling_control .
cd ~/Agruculture_Ws && colcon build --symlink-install && source install/setup.bash

# 1) 카메라·로봇 없이 흐름만 확인 (가짜 비전 + 가짜 로봇)
ros2 launch seedling_control seedling.launch.py fake_vision:=true

# 2) 실제 카메라(depth_grid_node) + 가짜 로봇   ※ RealSense 노드는 따로 실행
ros2 launch seedling_control seedling.launch.py

# 3) 로봇 노드 완성 후
ros2 launch seedling_control seedling.launch.py use_mock:=false
```

말 없이 시험: `ros2 topic pub --once /llm/command std_msgs/String "{data: '{\"action\":\"plant\"}'}"`

> ⚠ `ui_command_node.py`(별 님 파일)와 동시에 켜지 마세요. 둘 다 `/ui/command`를 받아서 명령이 두 번 처리됩니다. control_node가 그 역할을 대신합니다.

## 작업 흐름

**plant** (음성 "심어줘" 또는 UI 식재 버튼)
1. 로봇 `home` → 팔이 시야에서 빠진 뒤 `vision_settle_sec` 이후의 새 비전 결과를 기다림
2. `EMPTY` 이고 `valid_fraction ≥ min_valid_fraction` 인 칸을 S1→S4 순서로 선택 (음성 1그루, UI는 count 만큼)
3. 칸마다 `dig → plant → water → press` 를 로봇에 하나씩 보내고, 단계마다 TTS 안내
4. `home` → 다시 촬영해 해당 칸이 `OCCUPIED` 인지 확인

**harvest**: `harvest` → `home` (열매 비전이 없어서 위치는 로봇 노드가 고정 좌표로 처리)

**stop** ("멈춰" / UI 비상정지): 대기열 비우고 `/robot/stop {"mode":"stop"}` 발행. 기본은 래치 — UI에서 비상정지 해제 전까지 새 작업 거부 (`stop_latch:=false` 면 자동 해제)

작업 중에 새 명령이 오면 거부하고 TTS로 "지금 다른 작업 중이에요" 안내.

## 🤖 로봇 노드 규약 (로봇 담당자는 이것만 맞추면 됨)

모두 `std_msgs/String` + JSON. `mock_robot_node.py`가 그대로 예시입니다.

### `/robot/task` (control → robot)
```json
{"id": "a1b2c3d4", "action": "dig", "speed": 0.6,
 "slot": "S2", "camera_xyz_m": [0.1, -0.1, 0.6], "frame_id": "camera_color_optical_frame"}
```
| action | 의미 | 추가 필드 |
|---|---|---|
| `home` | 홈 자세 (카메라 시야에서 빠지기) | – |
| `dig` / `plant` / `water` / `press` | 땅 파기 / 묘목 심기 / 물 주기 / 다지기 | `slot`, `camera_xyz_m` |
| `harvest` | 열매 수확 후 수확함에 놓기 | `berry_ids` (없으면 null) |
| `gripper` | 그리퍼 | `state`: `open`/`close` |

- `camera_xyz_m` 은 **카메라 좌표(m)** 입니다. 로봇 베이스 좌표 아님. 로봇 노드에서 hand-eye 변환하거나, 구역이 4개 고정이면 `slot`(S1~S4)별로 티칭 좌표를 쓰는 게 제일 간단합니다.
- 한 번에 하나씩만 보냅니다. 끝나면 반드시 결과를 보내주세요.

### `/robot/result` (robot → control)
```json
{"id": "a1b2c3d4", "ok": true}
{"id": "a1b2c3d4", "ok": false, "error": "IK 실패"}
```
`id`는 받은 그대로. 응답이 `timeout_<action>` 초(기본 30~120초) 안에 없으면 실패 처리.

### `/robot/stop` (control → robot)
`{"mode": "stop"}` 즉시 정지(DSR `move_stop`) · `{"mode": "pause"}` / `{"mode": "resume"}` (`move_pause` / `move_resume`).
stop 받으면 진행 중이던 task는 `ok:false` 로 답하면 됩니다.

## 파라미터 (control_node)
| 이름 | 기본 | 설명 |
|---|---|---|
| `min_valid_fraction` | 0.90 | depth 유효 비율 기준 |
| `vision_settle_sec` | 2.5 | home 이후 비전 갱신 대기 |
| `vision_timeout_sec` | 8.0 | 비전 결과 최대 대기 |
| `verify_after_plant` | true | 심은 뒤 재촬영 확인 |
| `stop_latch` | true | 정지 후 UI 해제 필요 |
| `timeout_<action>` | 10~120 | 로봇 응답 대기 |
| `ui_cell_count` | 6 | UI 칸 수 (S1~S4 → 0~3번, 4·5번은 unknown) |

## TTS
`tts_node`: OpenAI `gpt-4o-mini-tts`, 키는 환경변수 또는 `seedling_llm/resource/.env` 재사용.
말하는 동안 `/tts/state = speaking` 발행 → voice_node에서 이걸 받아 웨이크워드를 잠깐 끄면 자기 목소리에 반응하는 걸 막을 수 있음.
