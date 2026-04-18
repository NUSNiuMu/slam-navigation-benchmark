# Hands-On Operation on TurtleBot3

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/288b3934766d80e18b82e9a62224f016>

## Overview

This note records the first end-to-end real-robot bring-up workflow on TurtleBot3, including ROS networking, teleoperation, LiDAR startup, SSH access, and LIO-SAM execution.

## Network Configuration

Use the phone hotspot IP as the ROS master address:

- ROS master: `172.20.10.2`

Add the following to `~/.bashrc` on the robot:

```bash
export ROS_MASTER_URI=http://172.20.10.2:11311
export ROS_IP=172.20.10.2
```

If ROS cannot contact its own server, source the shell configuration again:

```bash
source ~/.bashrc
```

## Base Platform Bring-Up

To bring up the robot base and test motion, use three terminals:

```bash
roscore
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
roslaunch base_control base_control.launch
```

To inspect IMU data:

```bash
rostopic echo /raw_imu
```

## SSH Access

Remote access examples:

```bash
ssh -Y bingda@172.20.10.2
ssh bingda@172.20.10.2
```

## LIO-SAM Startup

```bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
source devel/setup.bash
roslaunch lio_sam run6axis.launch
```

## RViz Display on the Raspberry Pi

```bash
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
```

## Recommended ROS Network Settings

On the laptop:

```bash
export ROS_MASTER_URI=http://172.20.10.2:11311
export ROS_IP=172.20.10.3
```

On the robot:

```bash
unset ROS_HOSTNAME
export ROS_IP=172.20.10.2
export ROS_MASTER_URI=http://172.20.10.2:11311
```

Always verify:

```bash
echo $ROS_MASTER_URI
echo $ROS_IP
echo $ROS_HOSTNAME
```

If `ROS_HOSTNAME` is set, remove it:

```bash
unset ROS_HOSTNAME
```

## RViz Launch on the Laptop

```bash
rviz -d /home/niumu/下载/rviz.rviz
```

## Saving and Viewing Point Cloud Maps

Save the global point cloud map:

```bash
rosrun pcl_ros pointcloud_to_pcd input:=/lio_sam/mapping/map_global _prefix:=lio_sam_map_
```

View the saved PCD:

```bash
pcl_viewer map_0000.pcd
```

## Practical Note

The key deployment detail is the network topology:

- laptop <-> robot: Wi-Fi
- robot <-> LiDAR: Ethernet
