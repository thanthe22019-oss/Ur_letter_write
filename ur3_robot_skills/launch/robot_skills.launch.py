"""Launch the Milestone 3 scene, skill server, and optional fixed demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ur3_llm_control.scene_config import load_scene


def _create_skill_nodes(context):
    scene_path = context.perform_substitution(LaunchConfiguration('scene_file'))
    scene = load_scene(scene_path)

    scene_parameters = {
        'world_name': scene.world_name,
        'valid_objects': list(scene.objects.keys()),
        'valid_zones': list(scene.zones.keys()),
        'use_sim_time': True,
    }
    for name, zone in scene.zones.items():
        scene_parameters[f'zones.{name}.pose'] = list(zone.pose)
        scene_parameters[f'zones.{name}.size'] = list(zone.size)

    skill_server = Node(
        package='ur3_robot_skills',
        executable='robot_skill_server',
        name='robot_skill_server',
        output='screen',
        parameters=[
            LaunchConfiguration('skill_config_file'),
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml']
            ),
            scene_parameters,
        ],
    )

    demo = Node(
        package='ur3_robot_skills',
        executable='robot_skill_demo',
        name='robot_skill_demo',
        output='screen',
        parameters=[{'server_wait_seconds': 120.0, 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('run_demo')),
    )
    return [skill_server, demo]


def generate_launch_description():
    scene_file = LaunchConfiguration('scene_file')

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur3_llm_control'), 'launch', 'llm_robot.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'scene_file': scene_file,
        }.items(),
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
                'run_demo',
                default_value='true',
                choices=['true', 'false'],
                description=(
                    'Run home -> move_above(red_cube) -> pick(red_cube) '
                    '-> place(red_cube, zone_b).'
                ),
            ),
            DeclareLaunchArgument(
                'scene_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_llm_control'), 'config', 'scene.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'skill_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_robot_skills'), 'config', 'robot_skills.yaml']
                ),
            ),
            baseline,
            OpaqueFunction(function=_create_skill_nodes),
        ]
    )
