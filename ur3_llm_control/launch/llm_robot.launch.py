"""Launch the UR3e baseline and the YAML-defined Milestone 2 scene."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ur3_llm_control.scene_config import box_sdf, gazebo_boxes, load_scene


def _spawn_scene(context):
    scene_file = context.perform_substitution(LaunchConfiguration('scene_file'))
    scene = load_scene(scene_file)
    spawn_nodes = []
    for box in gazebo_boxes(scene):
        spawn_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{box.name}',
                output='screen',
                arguments=[
                    '-string',
                    box_sdf(box),
                    '-name',
                    box.name,
                    '-allow_renaming',
                    'false',
                    '-x',
                    str(box.pose[0]),
                    '-y',
                    str(box.pose[1]),
                    '-z',
                    str(box.pose[2]),
                    '-R',
                    str(box.pose[3]),
                    '-P',
                    str(box.pose[4]),
                    '-Y',
                    str(box.pose[5]),
                ],
            )
        )
    # ros_gz_sim/create applies its initial_pose over the pose embedded in SDF.
    # Spawn the table first and give Gazebo time to register its collision
    # geometry before releasing the dynamic cubes.
    return [
        TimerAction(period=2.0, actions=[spawn_nodes[0]]),
        TimerAction(period=3.0, actions=spawn_nodes[1:]),
    ]


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    scene_file = LaunchConfiguration('scene_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    simulation_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_simulation_gz'), 'launch', 'ur_sim_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'gazebo_gui': gazebo_gui,
            'launch_rviz': 'false',
            'launch_servo': 'false',
        }.items(),
    )
    simulation_group = GroupAction(actions=[simulation_and_moveit], scoped=True)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_llm_scene',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml']
            ),
            {'use_sim_time': True},
        ],
        condition=IfCondition(launch_rviz),
    )

    scene_manager = Node(
        package='ur3_llm_control',
        executable='scene_manager',
        name='scene_manager',
        output='screen',
        parameters=[{'scene_file': scene_file, 'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type',
                default_value='ur3e',
                choices=['ur3', 'ur3e'],
                description='Universal Robots model; the assignment defaults to UR3e.',
            ),
            DeclareLaunchArgument(
                'gazebo_gui',
                default_value='true',
                choices=['true', 'false'],
                description='Start Gazebo with its graphical interface.',
            ),
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='true',
                choices=['true', 'false'],
                description='Start RViz with MotionPlanning and scene markers.',
            ),
            DeclareLaunchArgument(
                'scene_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_llm_control'), 'config', 'scene.yaml']
                ),
                description='Single source of truth for objects, zones, and collision geometry.',
            ),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ur3_llm_control'), 'rviz', 'scene.rviz']
                ),
                description='RViz configuration for the Milestone 2 scene.',
            ),
            simulation_group,
            rviz,
            scene_manager,
            OpaqueFunction(function=_spawn_scene),
        ]
    )
