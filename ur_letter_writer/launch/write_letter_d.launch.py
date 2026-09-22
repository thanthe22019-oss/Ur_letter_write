from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    config_file = LaunchConfiguration("config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    simulation_and_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": gazebo_gui,
            # The assignment launches RViz below with its own configuration.
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    # Keep launch arguments set by the included UR launch files from leaking
    # back into this launch context (notably launch_rviz:=false on Humble).
    simulation_group = GroupAction(actions=[simulation_and_moveit], scoped=True)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
            ),
            {"use_sim_time": True},
        ],
        condition=IfCondition(launch_rviz),
    )

    letter_writer = Node(
        package="ur_letter_writer",
        executable="write_letter_d",
        name="letter_writer_session",
        output="screen",
        parameters=[
            config_file,
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
            ),
            {"use_sim_time": True},
        ],
    )

    letter_path_marker = Node(
        package="ur_letter_writer",
        executable="letter_path_marker",
        name="letter_path_marker",
        output="screen",
        parameters=[config_file, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="Universal Robots model used for the assignment.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_letter_writer"), "config", "letter_d.yaml"]
                ),
                description="YAML file containing letter placement and planning parameters.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                choices=["true", "false"],
                description="Start Gazebo with its graphical interface.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                choices=["true", "false"],
                description="Start RViz with the MoveIt display.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ur_letter_writer"), "rviz", "letter_show.rviz"]
                ),
                description="RViz configuration used by the letter-writing demo.",
            ),
            simulation_group,
            rviz,
            letter_path_marker,
            letter_writer,
        ]
    )
