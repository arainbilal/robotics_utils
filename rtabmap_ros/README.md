# RTAB-Map ROS with OAK-D Camera (Docker Setup)

This note provides a Dockerized setup for running [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros) with the [Luxonis OAK-D](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM-series/OAK-D/) camera for 3D SLAM and mapping. The setup includes support for the [depthai-ros](https://github.com/luxonis/depthai-ros) wrapper and a fully integrated RTAB-Map SLAM pipeline.

---

## 🔧 Features

- ✅ RTAB-Map ROS integration
- ✅ OAK-D camera support (RGB-D)
- ✅ Docker-based reproducible environment
- ✅ Launch files for easy start-up
- ✅ Example mapping configuration

---

## 🐳 Docker Setup

### 1. Pull the Docker Image

```
docker pull osrf/ros:noetic-desktop-full
```

### 2. Create the Docker Container

```
xhost +local:docker

docker run -it  \
    --privileged \
    --network=host \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/input/js0:/dev/input/js0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY \
    --name="depthai-noetic-container" \
    osrf/ros:noetic-desktop-full
```

