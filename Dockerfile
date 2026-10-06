ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

WORKDIR /motkin-ws

ADD motkin-ws.repos .

RUN vcs import --input motkin-ws.repos

RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt update \
 && apt install -y pipx python3-rosdoc2 \
 && rosdep install --from-paths src --ignore-src -y

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build

ENV LIBGL_ALWAYS_SOFTWARE=1

ENTRYPOINT ["/bin/bash", "-c", "source ./install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
