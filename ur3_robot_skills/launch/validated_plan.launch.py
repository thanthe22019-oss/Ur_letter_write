"""Launch the simulated workcell and execute a validated static plan."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    plan_file = LaunchConfiguration('plan_file')

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
        name='skill_executor',
        output='screen',
        arguments=[
            '--file',
            plan_file,
            '--server-wait-seconds',
            LaunchConfiguration('server_wait_seconds'),
            '--step-timeout-seconds',
            LaunchConfiguration('step_timeout_seconds'),
        ],
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                'plan_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('ur3_llm_control'),
                        'config',
                        'milestone5_plan.json',
                    ]
                ),
                description=(
                    'JSON plan that must pass Plan Validator before execution.'
                ),
            ),
            DeclareLaunchArgument(
                'server_wait_seconds',
                default_value='120.0',
            ),
            DeclareLaunchArgument(
                'step_timeout_seconds',
                default_value='240.0',
            ),
            skill_server,
            executor,
        ]
    )
