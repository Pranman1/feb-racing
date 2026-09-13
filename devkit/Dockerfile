# FEB devkit: ROS 2 Humble + the AutoDRIVE RoboRacer bridge (BSD-2, AutoDRIVE Ecosystem).
# Same layout and entrypoint contract as the official RoboRacer Sim Racing League image,
# so a container built for FEB races also runs in the league and vice versa.
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-humble-tf-transformations \
        ros-humble-cv-bridge \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY autodrive_devkit /home/autodrive_devkit/src/autodrive_devkit
RUN cd /home/autodrive_devkit && . /opt/ros/humble/setup.sh && colcon build

COPY entrypoint.sh /home/autodrive_devkit.sh
WORKDIR /home/autodrive_devkit
EXPOSE 4567
ENTRYPOINT ["/home/autodrive_devkit.sh"]
