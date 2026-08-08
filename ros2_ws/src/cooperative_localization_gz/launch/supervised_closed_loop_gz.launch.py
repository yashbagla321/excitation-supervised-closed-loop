"""Launch the excitation-supervised closed-loop experiment through Gazebo.

Starts gz-sim with the hidden-target world, bridges clock / velocity command
/ odometry / set_pose between ROS 2 and Gazebo, and runs the supervised
closed-loop node software-in-the-loop. The whole launch shuts down when the
node finishes its packet budget, so scripted batch runs can invoke this
launch once per seed:

    ros2 launch cooperative_localization_gz supervised_closed_loop_gz.launch.py \
        seed:=7 output_name:=closed_loop_gz_seed7.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
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
    seed = LaunchConfiguration("seed")
    sensing_delay_packets = LaunchConfiguration("sensing_delay_packets")
    spread_threshold = LaunchConfiguration("spread_threshold")
    output_dir = LaunchConfiguration("output_dir")
    output_name = LaunchConfiguration("output_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([pkg_share, "worlds", "hidden_target.sdf"]),
        ),
        DeclareLaunchArgument(
            "config",
            default_value=PathJoinSubstitution(
                [pkg_share, "config", "supervised_closed_loop_gz.yaml"]
            ),
        ),
        DeclareLaunchArgument("use_gui", default_value="false"),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("sensing_delay_packets", default_value="1"),
        DeclareLaunchArgument("spread_threshold", default_value="0.16"),
        DeclareLaunchArgument("output_dir", default_value="results/ros_gz"),
        DeclareLaunchArgument("output_name", default_value="closed_loop_gz_run.csv"),
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
            name="closed_loop_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/model/vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/world/hidden_target_validation/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            ],
            output="screen",
        ),
        Node(
            package="cooperative_localization_gz",
            executable="supervised_closed_loop_node",
            name="supervised_closed_loop_node",
            parameters=[
                config,
                {
                    "use_sim_time": True,
                    "seed": seed,
                    "sensing_delay_packets": sensing_delay_packets,
                    "spread_threshold": spread_threshold,
                    "output_dir": output_dir,
                    "output_name": output_name,
                },
            ],
            output="screen",
            # The node ends the experiment by exiting; take the simulator and
            # the bridge down with it so scripted runs terminate cleanly.
            on_exit=Shutdown(),
        ),
    ])
