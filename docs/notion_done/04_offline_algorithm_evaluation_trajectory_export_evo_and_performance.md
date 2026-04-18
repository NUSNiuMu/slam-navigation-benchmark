# Offline Algorithm Evaluation: Trajectory Export, `evo`, and Runtime Performance

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/327b3934766d818bab8ce16fb77aab64>

## Objective

Run a unified offline evaluation of:

- `liosam`
- `fastlio`
- `pointlio`
- `fasterlio`

on the same rosbag, and automatically complete:

- trajectory export in TUM format
- `evo` computation of ATE and RPE
- statistics for CPU, memory, and per-frame processing time

## Script Locations

- main script: `ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh`
- monitoring script: `ws_livox/src/fyp_utils/scripts/slam_eval_monitor.py`
- trajectory export: `ws_livox/src/fyp_utils/scripts/bag_to_tum.py`
- result summary: `ws_livox/src/fyp_utils/scripts/summarize_static_eval.py`

## One-Command Run

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/short_test_20260318_171639_2026-03-18-17-16-42_0.bag \
  --gt-topic /odom
```

## Common Parameters

Run only one algorithm:

```bash
--algorithms liosam
```

Set playback rate:

```bash
--play-rate 1.0
```

Specify the output directory:

```bash
--out-root /home/niumu/ws_livox/src/fyp_utils/results/static_eval_20260318
```

Use a different GT topic if needed:

```bash
--gt-topic /your/ground_truth_topic
```

## Output Structure

Each algorithm directory contains:

- `estimated.bag`
- `traj_est.tum`
- `traj_gt.tum`
- `evo_ape.txt` / `evo_rpe.txt`
- `evo_ape.zip` / `evo_rpe.zip`
- `metrics/summary.json`
- `metrics/resource_usage.csv`
- `metrics/frame_times.csv`

The root output directory contains:

- `summary.csv`
- `summary.md`

## Metric Definitions

- `ATE`: computed by `evo_ape`
- `RPE`: computed by `evo_rpe` with `delta=1s`
- `CPU`: total sampled CPU utilisation across all ROS processes of the algorithm
- `memory`: total sampled RSS across all ROS processes, in MB
- `per-frame processing time`: wall-time interval between adjacent output odometry callbacks

## Dependency Check

Make sure the following are installed:

- `evo_ape`
- `evo_rpe`
- `rosbag`
- `rosnode`
- `rostopic`
- `python3`

## Notes

Default assumptions:

- `liosam` publishes `/odometry/imu`
- `fastlio`, `pointlio`, and `fasterlio` publish `/slam/odom`

If your topic mapping differs, update `odom_topic_for_alg()` inside `eval_static_rosbag.sh`.
