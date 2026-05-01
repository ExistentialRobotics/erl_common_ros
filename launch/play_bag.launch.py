import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node


def _launch_setup(context):
    bag_dir = context.launch_configurations["bag_dir"]
    rate = context.launch_configurations["rate"]
    read_ahead_queue_size = context.launch_configurations["read_ahead_queue_size"]
    qos_path = context.launch_configurations["qos_profile_overrides_path"]
    topics_file = context.launch_configurations["topics_file"]
    clock_rate = context.launch_configurations["clock_rate"]
    start_time_from_tf = context.launch_configurations["start_time_from_tf"]

    cmd = [
        "ros2",
        "bag",
        "play",
        bag_dir,
        "--read-ahead-queue-size",
        read_ahead_queue_size,
        "--rate",
        rate,
    ]

    if qos_path:
        cmd += ["--qos-profile-overrides-path", qos_path]

    if topics_file:
        with open(topics_file, "r") as f:
            topics = [line.strip() for line in f if line.strip() and not line.startswith("#")]
        if topics:
            cmd += ["--topics"] + topics

    rosbag_play = ExecuteProcess(cmd=cmd, output="screen")

    clock_node = Node(
        package="erl_common_ros",
        executable="clock_node",
        name="clock_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "clock_rate": float(clock_rate),
                "start_time_from_tf": start_time_from_tf.lower() in ("true", "1", "yes"),
            }
        ],
    )

    return [rosbag_play, clock_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("erl_common_ros")
    default_qos = os.path.join(pkg_share, "config", "qos_override.yaml")

    arguments = [
        DeclareLaunchArgument(
            "bag_dir",
            description="Directory containing the rosbag to play",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="1.0",
            description="Playback rate (e.g., 0.5 for half speed, 2.0 for double speed)",
        ),
        DeclareLaunchArgument(
            "read_ahead_queue_size",
            default_value="1000",
            description="Number of messages to read ahead from the rosbag",
        ),
        DeclareLaunchArgument(
            "qos_profile_overrides_path",
            default_value=default_qos,
            description="Path to QOS profile overrides YAML (empty string to disable)",
        ),
        DeclareLaunchArgument(
            "topics_file",
            default_value="",
            description="Path to a text file listing topics to play, one per line (empty to play all)",
        ),
        DeclareLaunchArgument(
            "clock_rate",
            default_value="100.0",
            description="Clock node publish rate in Hz",
        ),
        DeclareLaunchArgument(
            "start_time_from_tf",
            default_value="true",
            description="Read clock start time from /tf",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
