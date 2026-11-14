"""묘목심기 전체 실행.

  # 실제 로봇 + 그리퍼 + 카메라 (기본) — 두산 드라이버(dsr_bringup2 mode:=real)가 먼저 켜져 있어야 함
  ros2 launch seedling_control seedling.launch.py

  # 로봇은 안 움직이고 흐름만 (가짜 로봇)
  ros2 launch seedling_control seedling.launch.py use_mock:=true

  # 카메라도 없이
  ros2 launch seedling_control seedling.launch.py use_mock:=true camera:=false fake_vision:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')
    fake_vision = LaunchConfiguration('fake_vision')
    voice = LaunchConfiguration('voice')
    tts = LaunchConfiguration('tts')
    ui = LaunchConfiguration('ui')

    real_vision = PythonExpression(["'", fake_vision, "' != 'true'"])
    mock_vision = PythonExpression(["'", fake_vision, "' == 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false', description='true = 가짜 로봇 (실제 로봇 안 움직임)'),
        DeclareLaunchArgument('fake_vision', default_value='false', description='카메라 없이 가짜 비전'),
        DeclareLaunchArgument('voice', default_value='true', description='voice_node + llm_node'),
        DeclareLaunchArgument('tts', default_value='true', description='tts_node'),
        DeclareLaunchArgument('ui', default_value='true', description='rosbridge (관제 UI용)'),
        DeclareLaunchArgument('gripper', default_value='false',
                              description='그리퍼 노드 켜기 (dsr_bringup2 드라이버일 때만 true)'),
        DeclareLaunchArgument('camera', default_value='true', description='RealSense D455F 같이 켜기 (따로 켰으면 false)'),
        DeclareLaunchArgument('camera_serial', default_value="'261822308878'",
                              description='D455F 시리얼 (USB3 에 꽂힌 것, 기준사진을 찍은 카메라)'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={'serial_no': LaunchConfiguration('camera_serial'),
                              'align_depth.enable': 'true', 'enable_color': 'true',
                              'enable_depth': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('camera'))),

        Node(package='seedling_control', executable='control_node', output='screen',
             parameters=[{'tts_enabled': tts}]),

        Node(package='seedling_control', executable='mock_robot_node', output='screen',
             condition=IfCondition(use_mock),
             parameters=[{'fake_vision': mock_vision}]),

        # 실제 로봇 (두산 드라이버가 먼저 켜져 있어야 함)
        Node(package='robot_control', executable='seedling_robot', output='screen',
             condition=UnlessCondition(use_mock)),
        # 실제 그리퍼 (RH-P12-RN-A, /dsr01/gripper/open|close)
        # e0509_simple bringup 드라이버는 그리퍼 노드를 이미 켜요 → 기본은 안 켬.
        # dsr_bringup2 드라이버를 쓸 때만 gripper:=true
        Node(package='e0509_simple', executable='gripper_node', namespace='dsr01', output='screen',
             parameters=[{'mode': 'real'}], condition=IfCondition(LaunchConfiguration('gripper'))),

        Node(package='vision', executable='depth_grid_node', output='screen',
             condition=IfCondition(real_vision)),

        Node(package='seedling_llm', executable='voice_node', output='screen',
             parameters=[{'threshold': 0.2}],   # 원래 0.5 — 이 마이크에서는 0.3~0.4 정도라 낮춤
             condition=IfCondition(voice)),
        Node(package='seedling_llm', executable='llm_node', output='screen',
             condition=IfCondition(voice)),
        # 관제 UI 채팅·마이크 버튼 → 해석 → 승인 카드 → 실행 (UI v2)
        Node(package='seedling_llm', executable='voice_llm_node', output='screen',
             condition=IfCondition(ui)),

        Node(package='seedling_control', executable='tts_node', output='screen',
             condition=IfCondition(tts)),

        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             condition=IfCondition(ui)),
        # 관제 UI 웹페이지 → 브라우저에서 http://localhost:8000 열기
        Node(package='seedling_ui', executable='ui_server', output='screen',
             condition=IfCondition(ui)),
    ])
