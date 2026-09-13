# Your submission image: the FEB devkit plus your stack, started from the entrypoint.
#   docker build -t <you>/feb-racer:v1 -f starter_kit/Dockerfile stack/
FROM ghcr.io/pranman1/feb-devkit:latest
COPY . /home/autodrive_devkit/src/stack
RUN cd /home/autodrive_devkit && . /opt/ros/humble/setup.sh && colcon build --packages-skip autodrive_roboracer feb_tools
ENV FEB_LAUNCH="ros2 launch feb_driver drive.launch.py"
