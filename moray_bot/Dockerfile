FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-nav2* \
    ros-humble-robot-localization \
    ros-humble-turtlebot3-gazebo \
    git \
    python3-pip 

# Install python dependencies
RUN pip install pyproj

# paths
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

# Clone the repo
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/src
RUN git clone https://github.com/jacodd/moray_bot.git
WORKDIR /root/ws/
RUN colcon build
RUN echo "source /root/ws/install/setup.bash" >> ~/.bashrc

# Run the simulation
# CMD ["bash", "-c", "source ~/.bashrc && ros2 launch moray_bot gazebo_gps_world.launch.py && ros2 launch moray_bot gnss_waypoint_follower.launch.py"]
# CMD ["bash", "-c", "source ~/.bashrc && ros2 run moray_bot follower"]
