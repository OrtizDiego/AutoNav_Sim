# 0. YOLOv8n ONNX export (throwaway stage, keeps torch out of the image)
# Ultralytics only publishes the .pt weights; the ONNX file person_tracker
# loads has to be exported. The build fails here rather than shipping an
# image without a model. 320 px input: a quarter of the 640 px work, and the
# camera is 320x240; person_tracker reads the size from the model.
FROM ubuntu:22.04 AS yolo_export
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir ultralytics==8.4.172 onnx
WORKDIR /export
RUN yolo export model=yolov8n.pt format=onnx imgsz=320 opset=12 \
    && python3 -c "import onnx; onnx.checker.check_model('yolov8n.onnx')"

# 1. Base Image: ROS 2 Humble (Desktop version includes visualization tools)
FROM osrf/ros:humble-desktop-full

# 2. Set environment variables to non-interactive (prevents installation prompts)
ENV DEBIAN_FRONTEND=noninteractive

# Set the terminal to support 256 colors
ENV TERM xterm-256color

# 3. Update and Install Essential Robotics Tools
# numpy<2: onnxruntime would otherwise pull NumPy 2, which breaks the
# system cv2 and cv_bridge (built against NumPy 1.x) for every vision node.
# python3-pip: the ROS base image has no pip, so pip3 failed with 127.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-py-trees-ros \
    mesa-utils \
    wget \
    git \
    nano \
    && pip3 install "numpy<2" onnxruntime-gpu \
    && rm -rf /var/lib/apt/lists/*

COPY --from=yolo_export /export/yolov8n.onnx /root/models/yolov8n.onnx

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