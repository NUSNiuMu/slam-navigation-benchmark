# Mobile Robot Control Optimization Usage Guide

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/328b3934766d81478bd8cdfbc6e51b7c>

## Background

This note documents the control optimization completed on 2026-03-18. The main idea was to replace step-like teleoperation commands with acceleration-limited smoothed commands, reducing shocks and oscillation during manual driving and rosbag recording.

## Core Optimization

A new smoothing node was added:

- script: `cmd_vel_smoother.py`
- subscribes to: `/cmd_vel_raw`
- publishes to: `/cmd_vel`

Its purpose is to limit linear and angular acceleration so that command transitions are continuous rather than abrupt.

## Recording Integration

The smoothing chain was integrated into:

- `ws_livox/src/fyp_utils/launch/record_dataset.launch`

After launch, the system starts both rosbag recording and the command smoothing node automatically.

## Usage

### 1. Start the base controller

```bash
roslaunch base_control base_control.launch
```

### 2. Start recording and smoothing

```bash
roslaunch fyp_utils record_dataset.launch \
  bag_dir:=/home/bingda/PointCloudBag \
  bag_prefix:=eval_$(date +%Y%m%d_%H%M%S)
```

### 3. Use keyboard teleoperation on the raw topic

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_raw
```

## Quick Verification

```bash
rostopic echo /cmd_vel -n 5
rostopic info /cmd_vel
```

Expected result:

- `/cmd_vel` is published by the smoothing node
- velocity changes are continuous instead of step-like

## Saving a PCD Map

```bash
rosservice call /lio_sam/save_map "resolution: 0.2
destination: 'FYP/PointCloudBag/maps_415'"
```

## Recorded Topics in the Current Pipeline

`record_dataset.launch` records:

- `/livox/lidar`
- `/livox/imu`
- `/odom`
- `/tf`
- `/tf_static`

## Common Pitfalls

1. If `/odom` is not published, ATE/RPE cannot be computed later.
2. Keyboard teleoperation must publish to `/cmd_vel_raw`, not directly to `/cmd_vel`.
3. Before recording, verify the ground-truth trajectory topic:

```bash
rostopic hz /odom
```
