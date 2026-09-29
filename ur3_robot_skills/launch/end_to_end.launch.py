"""Launch UR3e simulation and execute one natural-language command."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    skill_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3_robot_skills'),
                    'launch',
                    'robot_skills.launch.py',
                ]
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'run_demo': 'false',
        }.items(),
    )

    executor = Node(
        package='ur3_robot_skills',
        executable='execute_plan',
        name='llm_skill_executor',
        output='screen',
        arguments=[
            '--command',
            LaunchConfiguration('command'),
            '--planner',
            LaunchConfiguration('planner'),
            '--model',
            LaunchConfiguration('model'),
            '--fallback-to-mock',
            LaunchConfiguration('fallback_to_mock'),
            '--server-wait-seconds',
            LaunchConfiguration('server_wait_seconds'),
            '--step-timeout-seconds',
            LaunchConfiguration('step_timeout_seconds'),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'command',
                default_value=(
                    'Arrange all objects according to my student ID.'
                ),
            ),
            DeclareLaunchArgument(
                'planner',
                default_value='gemini',
                choices=['gemini', 'openai', 'mock'],
            ),
            DeclareLaunchArgument(
                'model',
                default_value='gemini-3.5-flash-lite',
            ),
            DeclareLaunchArgument(
                'fallback_to_mock',
                default_value='false',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur3e',
                choices=['ur3', 'ur3e'],
            ),
            DeclareLaunchArgument(
                'gazebo_gui',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument('server_wait_seconds', default_value='120.0'),
            DeclareLaunchArgument('step_timeout_seconds', default_value='240.0'),
            skill_server,
            executor,
        ]
    )
