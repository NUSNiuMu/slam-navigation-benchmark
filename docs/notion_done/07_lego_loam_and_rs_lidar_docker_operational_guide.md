# LeGO-LOAM and RS-LiDAR Docker Operational Guide

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/2eeb3934766d8064a592d092c4bc5fa4>

## Overview

This note captures an earlier Docker-based workflow for LiDAR SLAM experiments using LeGO-LOAM and an RS-LiDAR setup. It is preserved here as historical reference rather than as part of the main four-algorithm benchmark pipeline.

## Container Initialization

Check container status:

```bash
docker ps -a
```

Start and attach:

```bash
docker start -ai liosam_rpi
```

Open another terminal inside the container:

```bash
docker exec -it liosam_rpi bash
```

## Workspace Setup

```bash
cd /root/ros_catkin_ws
source devel/setup.bash
```

## GUI and Network Setup

Allow X11 access on the host:

```bash
xhost +
```

Configure the network interface if needed:

```bash
sudo ip addr add 192.168.1.102/24 dev eth0
```

Inside the container:

```bash
export DISPLAY=:0
```

## Launching the Pipeline

Typical commands used in separate terminals:

- LiDAR driver: `roslaunch rslidar_sdk start.launch`
- LeGO-LOAM: `roslaunch lego_loam run.launch`
- rosbag playback: `rosbag play <file_name>.bag --clock --topic /velodyne_points /imu/data`

## Additional Historical Notes

The original note also referenced:

- `gtsam 4.02`
- `rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIRT`
- `python2 ~/ros_catkin_ws/src/imu_converter.py`
- `roslaunch rslidar_sdk imu_filter.launch`
- `roslaunch lio_sam run.launch`
- `rosservice call /lio_sam/save_map 0.2 "/root/Downloads/"`
- `pcl_viewer ~/Downloads/LOAM/GlobalMap.pcd`
