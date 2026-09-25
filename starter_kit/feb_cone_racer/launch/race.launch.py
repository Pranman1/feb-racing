import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

# scipy and CasADi are not in the devkit image: install them once into stack/.pydeps
# (see install_deps.sh) and the node imports them from there.
DEPS = "/home/autodrive_devkit/src/stack/.pydeps"


def generate_launch_description():
    params = os.path.join(get_package_share_directory("feb_cone_racer"), "config", "racer.yaml")
    return LaunchDescription([
        SetEnvironmentVariable("PYTHONPATH", DEPS + ":" + os.environ.get("PYTHONPATH", "")),
        Node(package="feb_cone_racer", executable="racer", name="racer",
             parameters=[params, {"use_sim_time": True}], output="screen", emulate_tty=True),
    ])
