ARG ROS_DISTRO=humble
FROM husarnet/ros:${ROS_DISTRO}-ros-core

WORKDIR /ros2_ws

# Same stuff as in Dockerfile.dev, but as a single RUN command to reduce the number and size of layers:
COPY rosbot_xl_docking src/rosbot_xl_docking
RUN apt-get update --fix-missing && \
    apt upgrade -y && \
    apt-get install -y \
        ros-dev-tools && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log && \
    # Additional cleanup
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "rosbot_xl_docking", "docking.launch.py"]
