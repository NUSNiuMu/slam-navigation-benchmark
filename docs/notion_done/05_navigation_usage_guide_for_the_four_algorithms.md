# Navigation Usage Guide for the Four Algorithms

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/327b3934766d818d92d8fc3643c68f2b>

## Objective

Under one shared navigation stack, perform a fair navigation comparison across four SLAM algorithms:

- `liosam`
- `fastlio`
- `fasterlio`
- `pointlio`

## Core Entry Point

Use the unified script:

- `ws_livox/src/fyp_utils/scripts/run_slam_nav.sh`

It automatically:

1. launches the selected SLAM backend
2. bridges outputs to the unified navigation inputs
3. launches the shared navigation stack

Unified navigation inputs:

- `/slam/odom`
- `/slam/cloud_registered`

## One-Command Launch

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh liosam
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fastlio
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fasterlio
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh pointlio
```

## Optional Parameters

Show RViz:

```bash
SHOW_RVIZ=true bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh liosam
```

Specify the map:

```bash
MAP_YAML=/home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml \
SHOW_RVIZ=true \
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fastlio
```

## Recommended Navigation Procedure

1. launch one algorithm using the unified script
2. set the RViz fixed frame to `map`
3. use `2D Pose Estimate` if needed
4. send the same target set using `2D Nav Goal`
5. record success, traversal time, smoothness, and oscillation

## Fair-Comparison Notes

- keep the same map
- keep the same start pose and goals
- keep `move_base` and DWA unchanged
- repeat each run multiple times
- clear old nodes before each experiment

## Quick Troubleshooting

Check odometry and scan frequency:

```bash
rostopic hz /slam/odom
rostopic hz /scan
```

Check cloud availability:

```bash
rostopic list | egrep 'cloud_registered|slam'
```

Check TF:

```bash
rosrun tf tf_echo map base_link
```
