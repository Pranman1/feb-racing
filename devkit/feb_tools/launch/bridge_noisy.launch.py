"""The AutoDRIVE bridge with its lidar and IMU routed through the sensor_noise node.
Arguments: range_sigma, beam_dropout, scan_dropout, latency, imu_sigma (see sensor_noise.py)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {"range_sigma": "0.02", "beam_dropout": "0.02", "scan_dropout": "0.0", "latency": "0.0", "imu_sigma": "0.02"}


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            Node(package="autodrive_roboracer", executable="autodrive_bridge", name="autodrive_bridge",
                 emulate_tty=True, output="screen",
                 remappings=[("/autodrive/roboracer_1/lidar", "/autodrive/raw/lidar"),
                             ("/autodrive/roboracer_1/imu", "/autodrive/raw/imu")]),
            Node(package="feb_tools", executable="sensor_noise", name="sensor_noise", output="screen",
                 parameters=[{k: LaunchConfiguration(k) for k in ARGS}]),
        ])
