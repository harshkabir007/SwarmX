# SwarmX development / demo image: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + rmw_zenoh
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
      ros-jazzy-rmw-zenoh-cpp ros-jazzy-tf-transformations \
      python3-pytest python3-transforms3d python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /swarmx
COPY ros2_ws/src /swarmx/ros2_ws/src
RUN . /opt/ros/jazzy/setup.sh && cd /swarmx/ros2_ws && colcon build --symlink-install

RUN echo 'source /opt/ros/jazzy/setup.bash && source /swarmx/ros2_ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
