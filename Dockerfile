# 1. Base Image: ROS 2 Humble (Desktop version includes visualization tools)
FROM osrf/ros:humble-desktop-full

# 2. Set environment variables to non-interactive (prevents installation prompts)
ENV DEBIAN_FRONTEND=noninteractive

# Set the terminal to support 256 colors
ENV TERM xterm-256color

# 3. Update and Install Essential Robotics Tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-joint-state-publisher-gui \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 4. Create the Workspace
WORKDIR /root/dev_ws
COPY src ./src

# 5. Build the workspace (initially empty)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 6. Add sourcing to bashrc so we don't have to type it every time
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/dev_ws/install/setup.bash" >> /root/.bashrc

# 7. Set entrypoint
CMD ["/bin/bash"]