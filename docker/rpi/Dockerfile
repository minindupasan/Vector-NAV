# ============================================================================
# VECTOR NAV — Raspberry Pi 5 ROS2 Humble container
#
# Provides: motor control (GPIO), IMU (I2C), camera (libcamera/v4l2),
#           encoder odometry, Madgwick-filtered IMU, EKF fusion.
#
# Build:  docker build -t vector-control docker/rpi/
# Run:    docker compose -f docker/rpi/docker-compose.yaml up
# ============================================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
SHELL ["/bin/bash", "-c"]

# ── System dependencies ─────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # I2C / GPIO kernel interface
    i2c-tools \
    # Camera (V4L2 + libcamera userland)
    libcamera-dev \
    libcamera-tools \
    v4l-utils \
    # Required by python-prctl (picamera2 dependency)
    libcap-dev \
    # Useful debug tools
    iproute2 \
    iputils-ping \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ── ROS2 packages ───────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-v4l2-camera \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-rmw-cyclonedds-cpp \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages (GPIO, IMU, Camera) ─────────────────────────────────────
RUN pip3 install --no-cache-dir \
    gpiozero \
    lgpio \
    smbus2 \
    picamera2 \
    numpy

# ── Workspace setup ─────────────────────────────────────────────────────────
ENV VECTOR_WS=/ros2_ws
RUN mkdir -p ${VECTOR_WS}/src
WORKDIR ${VECTOR_WS}

# ── RPLidar from source (apt package has buffer overflow on ARM) ───────────
RUN cd ${VECTOR_WS}/src && \
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# Copy vector_control package
COPY vector_control ${VECTOR_WS}/src/vector_control

# Build both packages
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select rplidar_ros vector_control --symlink-install

# ── Entrypoint ──────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "vector_control", "control.launch.py"]
