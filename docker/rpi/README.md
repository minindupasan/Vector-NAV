# Vector Nav — Raspberry Pi 5 Docker Setup

Two containers sharing ROS2 topics via DDS over host network:
- **vector-control**: motor control, IMU, encoders, EKF
- **vector-camera**: RPi Camera Module 3 stream (libcamera + rpicam-apps built from source)

## Build Images

```bash
cd ~/vector_nav/docker/rpi

# Build control image
docker build -f Dockerfile.control -t vector-control .

# Build camera image (takes ~30min - compiles libcamera + rpicam-apps)
docker build -f Dockerfile.camera -t vector-camera .
```

## Run Containers

### vector-control
```bash
docker run -d \
  --name vector-control \
  --network=host \
  --privileged \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e GPIOZERO_PIN_FACTORY=lgpio \
  -v /dev:/dev \
  -v ./vector_control:/ros2_ws/src/vector_control:ro \
  -v /lib/firmware:/lib/firmware:ro \
  -v /proc/device-tree:/proc/device-tree:ro \
  vector-control:latest \
  ros2 launch vector_control control.launch.py
```

### vector-camera
```bash
docker run -d \
  --name vector-camera \
  --network=host \
  --privileged \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  -v /run/libcamera:/run/libcamera \
  -v /lib/firmware:/lib/firmware:ro \
  -v /proc/device-tree:/proc/device-tree:ro \
  vector-camera:latest \
  bash -c "sleep infinity"
```

### Or use docker compose for both
```bash
cd ~/vector_nav/docker/rpi
docker compose up --build
```

## Useful Commands

```bash
# Exec into containers
docker exec -it vector-control bash
docker exec -it vector-camera bash

# Test camera inside vector-camera container
rpicam-hello --list-cameras
rpicam-still -o /tmp/test.jpg
rpicam-vid -t 10000 -o /tmp/test.h264 --codec yuv420

# Check ROS2 topics (from either container)
ros2 topic list

# Stop and remove containers
docker stop vector-control vector-camera
docker rm vector-control vector-camera
```

## Notes

- `/run/udev:/run/udev:ro` mount is required for libcamera to discover camera devices inside Docker.
- Both containers use `--network=host` so DDS topic discovery works automatically between them.
- `ROS_DOMAIN_ID` must match across all containers and any external devices (e.g., Jetson).
