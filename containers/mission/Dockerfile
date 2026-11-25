ARG BASE_IMAGE
# One pinned ROS/Gazebo toolchain serves the mission and simulator-side bridge.
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev ros-jazzy-cv-bridge ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/mission_ws
ARG MSGS_REV
ARG INTERFACE_REV
RUN git clone https://github.com/PX4/px4_msgs.git src/px4_msgs \
    && git -C src/px4_msgs checkout "${MSGS_REV}" \
    && git clone https://github.com/Auterion/px4-ros2-interface-lib.git src/px4-ros2-interface-lib \
    && git -C src/px4-ros2-interface-lib checkout "${INTERFACE_REV}" \
    && rm -rf src/px4_msgs/msg src/px4_msgs/srv
# These definitions are staged from the exact PX4 revision running in SITL.
COPY px4_msgs/ src/px4_msgs/
COPY src/ src/
ARG BUILD_JOBS=4
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}" CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} colcon build \
       --executor sequential --packages-up-to aruco_tracker mission_demo \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && source install/setup.bash \
    && cmake -S src/mission_demo/test -B /tmp/mission-demo-tests -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/mission-demo-tests -j${BUILD_JOBS} \
    && ctest --test-dir /tmp/mission-demo-tests --output-on-failure
COPY runtime/ /opt/mission/
COPY install-survey.sh /tmp/install-survey.sh
RUN test -s /opt/mission/models/coco-yolo26n.onnx && bash /tmp/install-survey.sh
COPY dashboard/ /opt/camera-view/
COPY notices/ /usr/share/doc/px4-mission-demo/
COPY build-info.json /opt/mission/build-info.json
RUN chmod -R a+rX /opt/mission /opt/mission_ws \
    && chmod +x /opt/mission/entrypoint.sh
ENV HOME=/tmp/mission-home ROS_LOG_DIR=/tmp/ros-log
WORKDIR /tmp
USER 1001
ENTRYPOINT ["/opt/mission/entrypoint.sh"]
CMD ["mission"]
