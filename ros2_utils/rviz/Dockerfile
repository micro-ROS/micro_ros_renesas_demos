FROM ros:humble

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
&&  apt install -y ros-$ROS_DISTRO-rviz2 \
&&  apt install -y ros-$ROS_DISTRO-joint-state-publisher \
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

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]