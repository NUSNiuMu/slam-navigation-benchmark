# Rosbag Recording Standard for Trajectory Evaluation

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/327b3934766d8150ae75ff33964a6ed5>

## Purpose

This document standardizes rosbag recording for trajectory-error evaluation so that the following metrics can be computed reliably afterward:

- ATE
- RPE

It is intended for offline unified evaluation of:

- `LIO-SAM`
- `FAST-LIO`
- `Point-LIO`
- `FASTER-LIO`

## Why ATE/RPE Can Become `NA`

In a previous run, the bag contained only:

- `/livox/lidar`
- `/livox/imu`
- `/tf_static`

There was no valid ground-truth trajectory topic, so ATE/RPE could not be computed.

## Required Pre-Check

Before recording, verify that the ground-truth topic exists and is publishing:

```bash
rostopic list | egrep 'odom|ground|truth|mocap|vicon'
rostopic hz /odom
```

If your ground-truth topic is not `/odom`, replace it in all commands below.

## Recommended Recording Commands

### Option A: Existing launch file

```bash
roslaunch fyp_utils record_dataset.launch \
  bag_dir:=/home/bingda/PointCloudBag \
  bag_prefix:=eval_$(date +%Y%m%d_%H%M%S)
```

### Option B: Manual rosbag recording

```bash
rosbag record -O /home/bingda/PointCloudBag/eval_$(date +%Y%m%d_%H%M%S).bag \
  /livox/lidar /livox/imu /tf /tf_static /odom
```

If the GT topic is different:

```bash
rosbag record -O /home/bingda/PointCloudBag/eval_$(date +%Y%m%d_%H%M%S).bag \
  /livox/lidar /livox/imu /tf /tf_static /your_gt_topic
```

## Mandatory Post-Recording Check

```bash
rosbag info /home/bingda/PointCloudBag/<your_bag>.bag
```

Confirm that:

1. the GT topic appears in the topic list
2. the GT topic message count is greater than zero
3. `/livox/lidar`, `/livox/imu`, `/tf`, and `/tf_static` are present

## Connection to the Offline Evaluation Script

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/<your_bag>.bag \
  --gt-topic /odom \
  --no-rviz true
```

If the GT topic is different:

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/<your_bag>.bag \
  --gt-topic /your_gt_topic \
  --no-rviz true
```

## Recommended Experiment Log Items

For each bag, record:

- recording date and scenario
- start point, end point, and duration
- GT source
- whether wheel slip or strong motion occurred
- evaluation command and output directory

## One-Sentence Standard

A bag suitable for ATE/RPE must contain sensor data, TF, and one valid ground-truth trajectory topic with message count greater than zero.
