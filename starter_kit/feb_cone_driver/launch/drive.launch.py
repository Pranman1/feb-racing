import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("feb_cone_driver"), "config", "cone_driver.yaml")
    return LaunchDescription([
        Node(package="feb_cone_driver", executable="cone_driver", name="cone_driver",
             parameters=[params, {"use_sim_time": True}], output="screen", emulate_tty=True),
    ])
