# SLAM Navigation Benchmark for Livox MID360 Mobile Robot

This repository contains the curated code and documentation bundle for an undergraduate Final Year Project at the National University of Singapore. The project benchmarks four LiDAR-inertial SLAM backends on a real mobile robot using a shared ROS navigation stack, with the goal of comparing both offline trajectory quality and online navigation behaviour under consistent downstream settings.

The benchmark focuses on the following SLAM backends:

- `LIO-SAM`
- `FAST-LIO`
- `Point-LIO`
- `FASTER-LIO`

All four methods are integrated with the same navigation pipeline through unified interfaces for odometry and registered point clouds. This allows the comparison to focus on the odometry and obstacle observations produced by each SLAM method rather than planner retuning.

## Project Scope

The repository combines three layers of work:

- SLAM backends adapted for Livox MID360 and real-robot testing
- A shared navigation stack based on `move_base`, `DWAPlannerROS`, and point-cloud-to-laserscan conversion
- Evaluation scripts, experiment workflows, and report materials used to benchmark the four methods

The project includes both:

- offline rosbag-based SLAM evaluation using ATE/RPE and runtime statistics
- online waypoint-based navigation evaluation using success rate, traversal time, path length, safety margin, recovery count, and related metrics

## Repository Layout

See [REPOSITORY_STRUCTURE.md](./REPOSITORY_STRUCTURE.md) for a fuller breakdown. The main top-level folders are:

```text
.
├── ws_livox/
│   └── src/
│       ├── LIO-SAM-MID360/
│       ├── fyp_utils/
│       └── mid360_navigation/
├── fastlio2_ws/
│   └── src/
│       ├── FAST_LIO/
│       └── Point-LIO/
└── fasterlio_ws/
    └── src/
        └── faster-lio/
```

## Main Components

### 1. `ws_livox/src/LIO-SAM-MID360`
Livox MID360-adapted `LIO-SAM` package used as one of the benchmarked SLAM backends.

Relevant files include:

- `config/paramsLivoxIMU.yaml`
- `launch/run.launch`
- `src/mapOptmization.cpp`

### 2. `fastlio2_ws/src/FAST_LIO`
`FAST-LIO` backend configured for the same MID360 platform and navigation interface.

Relevant files include:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laserMapping.cpp`

### 3. `fastlio2_ws/src/Point-LIO`
`Point-LIO` backend used in the comparison, including local parameter adjustments for fairer downstream navigation benchmarking.

Relevant files include:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laserMapping.cpp`
- `src/Estimator.cpp`

### 4. `fasterlio_ws/src/faster-lio`
`FASTER-LIO` backend used as the fourth SLAM method in the benchmark.

Relevant files include:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laser_mapping.cc`

### 5. `ws_livox/src/mid360_navigation`
Shared navigation stack used by all four SLAM methods.

It contains:

- launch files for navigation, `move_base`, and point-cloud-to-laserscan conversion
- global/local costmap configuration
- DWA local planner parameters
- generated occupancy maps for several test scenarios
- helper scripts for map generation and map cleaning

Key files:

- `launch/navigation.launch`
- `launch/move_base.launch`
- `launch/pointcloud_to_laserscan.launch`
- `config/dwa_local_planner_params.yaml`
- `config/costmap_common_params.yaml`
- `scripts/pcd_to_grid.py`
- `scripts/clean_map_pgm.py`

### 6. `ws_livox/src/fyp_utils`
Project-specific tooling for experiment execution, recording, evaluation, and documentation.

It contains:

- rosbag recording launch files
- static rosbag evaluation scripts
- waypoint-based navigation evaluation scripts
- deployment and monitoring helpers
- project notes, evaluation docs, and the final report source

Key files:

- `scripts/eval_static_rosbag.sh`
- `scripts/nav_waypoint_eval.py`
- `scripts/eval_nav_4alg.sh`
- `scripts/capture_waypoints.py`
- `scripts/bag_to_tum.py`
- `scripts/slam_eval_monitor.py`
- `docs/nus_fyp_report_english_20260405.tex`
- `docs/nus_fyp_report_english_20260405.pdf`

## Benchmark Workflow

The overall workflow used in this project is:

1. Launch one SLAM backend on the real robot or on recorded rosbag data
2. Relay its odometry and registered cloud to a shared navigation interface
3. Run the same ROS navigation stack for all methods
4. Record navigation and system metrics through a common evaluator
5. Compare success rate, path efficiency, safety margin, and compute cost

In the online setting, all methods are tested with the same:

- map
- waypoint file
- local planner
- costmap configuration
- safety thresholds

## Included Documentation

This repository includes the project report and several experiment-oriented notes. Important documents are under:

- `ws_livox/src/fyp_utils/docs/`

Notable files include:

- `nus_fyp_report_english_20260405.pdf`
- `graduation_report_draft_20260405.md`
- `notion_02_navigation_usage.md`
- `notion_03_static_rosbag_eval.md`
- `notion_05_nav_eval_4alg_waypoints.md`

## What Is Intentionally Excluded

Large generated assets are intentionally not included in this GitHub upload. Examples include:

- rosbags
- raw videos
- large point-cloud map outputs such as `.pcd`
- experiment result archives stored outside this repository
- build products such as `build/`, `devel/`, and log folders

This keeps the repository focused on code, configuration, evaluation scripts, and report materials.

## Notes on Reproducibility

This repository is a curated project bundle rather than a fully containerised release. Some upstream packages retain their original structure, and some experiment results in the report were generated from datasets, maps, and videos that are stored separately from the repository because of size constraints.

For that reason, the repository should be understood as:

- a reproducible code-and-configuration archive for the project
- a reference implementation of the benchmark pipeline
- a source repository for the written report and experiment tooling

## Acknowledgement

This project was developed as part of a Final Year Project at the National University of Singapore.
