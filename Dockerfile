FROM ros:galactic-ros-base

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

ADD . src/

RUN /bin/bash -c "source /opt/ros/galactic/setup.bash \
    && colcon build"

ENTRYPOINT /bin/bash -c "source install/setup.bash && ros2 run stewart stewart_kinematics_node"
