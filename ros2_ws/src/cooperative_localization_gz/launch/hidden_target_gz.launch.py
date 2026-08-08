from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("cooperative_localization_gz")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    world = LaunchConfiguration("world")
    config = LaunchConfiguration("config")
    use_gui = LaunchConfiguration("use_gui")
    run_batch = LaunchConfiguration("run_batch")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([pkg_share, "worlds", "hidden_target.sdf"]),
        ),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution([pkg_share, "config", "hidden_target_gz.yaml"]),
        ),
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument("run_batch", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
            ),
            condition=UnlessCondition(use_gui),
            launch_arguments={
                "gz_args": ["-r -s ", world],
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])
            ),
            condition=IfCondition(use_gui),
            launch_arguments={
                "gz_args": ["-r ", world],
            }.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        Node(
            package="cooperative_localization_gz",
            executable="measurement_emulator_node",
            name="measurement_emulator_node",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="cooperative_localization_gz",
            executable="hidden_target_experiment_node",
            name="hidden_target_experiment_node",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="cooperative_localization_gz",
            executable="hidden_target_batch_runner",
            name="hidden_target_batch_runner",
            parameters=[config, {"run_full_suite": run_batch}],
            output="screen",
        ),
    ])
