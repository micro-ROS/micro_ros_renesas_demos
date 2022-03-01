FROM ros:galactic

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
&&  apt install -y ros-$ROS_DISTRO-rviz2 \
&&  apt install -y ros-$ROS_DISTRO-joint-state-publisher \
&&  apt install -y ros-$ROS_DISTRO-rmw-fastrtps-cpp \
&&  apt remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
&&  rm -rf /var/lib/apt/lists/*

WORKDIR /renesas
COPY ./src ./src

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  rosdep update \
&&  rosdep install --from-paths src --ignore-src -r -y \
&&  colcon build --symlink-install

ENV DEBIAN_FRONTEND dialog
ENV DISPLAY unix:0
COPY disable_fastdds_shm.xml /tmp/
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]