import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("feb_driver"), "config", "driver.yaml")
    return LaunchDescription([
        Node(package="feb_driver", executable="reactive_driver", name="reactive_driver",
             parameters=[params], output="screen", emulate_tty=True),
    ])
