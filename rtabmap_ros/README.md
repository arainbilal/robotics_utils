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

To attach another terminal to the running container, issue the following command:

```
xhost +local:docker
docker exec -it depthai-noetic-container bash"
```

----

## 🛠️ System Requirements & Installation (Inside Docker)

The Docker container require to be configured to install all necessary packages and dependencies to run RTAB-Map with the OAK-D camera. Below is a breakdown of what is the minimal requirments to be installed inside the container:

### 📸 DepthAI ROS Wrapper

The following key pacakage are required to be installed inside the Docker container for the OAK-D camera:

```
sudo apt-get update
sudo apt-get install ros-noetic-depthai-ros
```

#### Additional utilities and plugins for Rviz

```
sudo apt-get install mesa-utils
sudo apt-get install ros-noetic-rviz ros-noetic-rviz-imu-plugin
```

### ✅ Test DepthAI Camera is Working Inside Docker Container

To verify that your OAK-D (DepthAI) camera is functioning properly inside the Docker container, follow the steps below after launching the container.

#### Source the ROS environment

Once inside the container, source the script to use ROS:

```
source /opt/ros/noetic/setup.bash
```
#### Launch the DepthAI ROS Node

Run the stereo-inertial launch file from the ```depthai_examples``` package

```
roslaunch depthai_examples stereo_inertial_node.launch
```

#### Check the Camera topics

In a separate terminal (attach another terminal to the running container), check if the topics are being published:

```
rostopic list
```

and you should be able to see the following (**important**) topics being published, which are required by the RTABMap:

```
/stereo_inertial_publisher/color/camera_info
/stereo_inertial_publisher/color/image
/stereo_inertial_publisher/imu
/stereo_inertial_publisher/stereo/camera_info
/stereo_inertial_publisher/stereo/depth
/stereo_inertial_publisher/stereo/points
/tf
/tf_static
```

### ⚠️ Warning

If you have not successfully completed the test in the previous section — "**Check the Camera topics**" — and you do not see camera-related topics being published, **do not proceed further**.

You must resolve the issue with the DepthAI camera setup before continuing with RTAB-Map or any other SLAM components.

🛠️ Resolve camera topic issues first — otherwise, RTAB-Map will not receive valid sensor data and mapping will fail. 

---

## 🧱  RTAB-Map core and ROS wrapper

The following key pacakage are required to be installed inside the Docker container for the RTAB-Map:

```
sudo apt-get update
sudo apt-get install ros-noetic-rtabmap-ros
```

---

## 🚀 Running RTAB-Map with OAK-D

Once inside the container, source the script to use ROS:

```
source /opt/ros/noetic/setup.bash
```

### 🔌 Launch DepthAI (OAK-D) Nodes

```
roslaunch depthai_examples stereo_inertial_node.launch
```

### 🧭 Estimate quaternion of the IMU data

```
rosrun imu_filter_madgwick imu_filter_node \
   imu/data_raw:=/stereo_inertial_publisher/imu \
   imu/data:=/stereo_inertial_publisher/imu/data  \
   _use_mag:=false \
   _publish_tf:=false
```

### 🗺️ Launch RTAB-Map RGBD-Mapping

```
roslaunch rtabmap_launch rtabmap.launch \
    args:="--delete_db_on_start" \
    rgb_topic:=/stereo_inertial_publisher/color/image \
    depth_topic:=/stereo_inertial_publisher/stereo/depth \
    camera_info_topic:=/stereo_inertial_publisher/color/camera_info \
    imu_topic:=/stereo_inertial_publisher/imu/data \
    frame_id:=oak-d_frame \
    approx_sync_max_interval:=0.001 \
    approx_sync:=true \
    wait_imu_to_init:=true
```

### ⚠️ Warning

If you see the following messages on the ```rtab-map``` terminal, it means that odometry is lost and/or the image has no features to track:

```
[ WARN] (2025-04-08 10:36:37.236) RegistrationVis.cpp:1366::computeTransformationImpl() All projected points are outside the camera. Guess (xyz=-0.095709,-0.169845,0.071579 rpy=-2.713211,-1.092857,-1.263656) is wrong or images are not overlapping.
[ WARN] (2025-04-08 10:36:37.237) OdometryF2M.cpp:569::computeTransform() Registration failed: "Missing correspondences for registration (-1->537). fromWords = 0 fromImageEmpty=1 toWords = 0 toImageEmpty=0" (guess=xyz=0.000000,0.000000,0.000000 rpy=-2.766600,-1.219348,1.931667)
[ WARN] (2025-04-08 10:36:37.237) OdometryF2M.cpp:317::computeTransform() Failed to find a transformation with the provided guess (xyz=0.000000,0.000000,0.000000 rpy=-2.766600,-1.219348,1.931667), trying again without a guess.
[ WARN] (2025-04-08 10:36:37.331) OdometryF2M.cpp:559::computeTransform() Trial with no guess still fail.
[ WARN] (2025-04-08 10:36:37.331) OdometryF2M.cpp:569::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=14) between -1 and 537" (guess=xyz=0.000000,0.000000,0.000000 rpy=-2.766600,-1.219348,1.931667)
[ INFO] [1744108597.333395396]: Odom: quality=0, std dev=0.000000m|0.000000rad, update time=0.156573s, delay=0.266872s

[ERROR] (2025-04-08 10:36:37.335) Rtabmap.cpp:1406::process() RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. Image 1766 is ignored!
```

![RTAB-Map Stereo Inertial Demo](rtabmap_ros/RTAB-Map-Stereo_Inertial.gif)

<!-- <div align="center"><img src="rtabmap_ros/RTAB-Map-Stereo_Inertial.gif" width=600px/></div> -->



## 📬 Need Help or Would like to add? Open an Issue
If you run into any problems or have questions, feel free to open an issue on this repository.

📌 Steps to Open an Issue:
1. Go to the Issues tab of this repository.
2. Click on the New Issue button.
3. Provide a clear title and a detailed description of your question or problem.
4. (Optional) Attach screenshots, logs, or code snippets that help explain the issue.
5. Click Submit new issue to post your request.

## 📚 References and Resources

* [RTAB-Map ROS Wiki](http://wiki.ros.org/rtabmap_ros)
* [Luxonis DepthAI ROS](https://github.com/luxonis/depthai-ros)
* [RTAB-Map Tutorials](http://wiki.ros.org/rtabmap_ros/Tutorials)