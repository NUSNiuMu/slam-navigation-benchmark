# Repository Structure

This document summarises the layout of the `slam-navigation-benchmark` repository and explains the role of each major folder.

## Top Level

```text
github_project_upload/
├── README.md
├── REPOSITORY_STRUCTURE.md
├── docs/
├── ws_livox/
├── fastlio2_ws/
└── fasterlio_ws/
```

## `docs/`

Top-level curated documentation layer added for repository readability.

```text
docs/
├── README.md
├── notion_done/
├── reports/
└── waypoints/
```

Purpose:

- provide a clear reading entry for GitHub visitors
- mirror `Done` project notes from Notion in Markdown form
- surface report entry points without changing the original workspace layout

### `docs/notion_done/`

Curated copies of project notes whose status was `Done` in the Notion FYP database.

This includes:

- real-robot bring-up notes
- rosbag recording standards
- offline evaluation workflow notes
- navigation usage notes
- waypoint-based navigation evaluation notes
- control optimisation notes
- one earlier historical reference guide

### `docs/reports/`

Top-level report entry containing the final project PDF and its index.

### `docs/waypoints/`

Top-level copies of representative waypoint YAML files kept for documentation and evaluation examples.

## `ws_livox/`

This workspace contains the project-specific integration layer around the benchmark.

```text
ws_livox/
└── src/
    ├── LIO-SAM-MID360/
    ├── fyp_utils/
    └── mid360_navigation/
```

### `ws_livox/src/LIO-SAM-MID360/`

Livox MID360-oriented `LIO-SAM` package.

Main purpose:

- provide the `LIO-SAM` baseline used in the comparison
- support real-robot and map-saving workflows

Examples of important files:

- `config/paramsLivoxIMU.yaml`
- `src/mapOptmization.cpp`

### `ws_livox/src/fyp_utils/`

Project utilities and experiment scripts.

```text
fyp_utils/
├── docs/
├── launch/
└── scripts/
```

#### `launch/`

Contains project launch files such as rosbag recording helpers.

#### `scripts/`

Contains benchmark tooling such as:

- waypoint capture
- static rosbag evaluation
- navigation evaluation
- trajectory export
- monitoring and summarisation

Representative scripts:

- `eval_static_rosbag.sh`
- `nav_waypoint_eval.py`
- `capture_waypoints.py`
- `bag_to_tum.py`
- `summarize_nav_eval.py`

### `ws_livox/src/mid360_navigation/`

Shared navigation stack used by all four SLAM methods.

```text
mid360_navigation/
├── config/
├── launch/
├── maps/
├── maps_324/
├── maps_415_20260415/
├── maps_415_20260415_cleaned/
├── maps_corridor2_20260407/
├── maps_corridor2_20260407_cleaned/
├── maps_fastlio_20260402/
├── rviz/
└── scripts/
```

Purpose:

- provide a single downstream navigation configuration
- host costmap and DWA planner settings
- store generated occupancy maps used in experiments
- convert point clouds into 2D scan observations

Representative files:

- `launch/navigation.launch`
- `config/dwa_local_planner_params.yaml`
- `config/costmap_common_params.yaml`
- `scripts/pcd_to_grid.py`
- `scripts/clean_map_pgm.py`

## `fastlio2_ws/`

Workspace containing `FAST-LIO` and `Point-LIO`.

```text
fastlio2_ws/
└── src/
    ├── FAST_LIO/
    └── Point-LIO/
```

### `fastlio2_ws/src/FAST_LIO/`

`FAST-LIO` backend configured for the MID360 robot platform.

Commonly referenced files:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laserMapping.cpp`

### `fastlio2_ws/src/Point-LIO/`

`Point-LIO` backend used in the benchmark.

Commonly referenced files:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laserMapping.cpp`
- `src/Estimator.cpp`
- `src/parameters.cpp`

## `fasterlio_ws/`

Workspace containing the `FASTER-LIO` backend.

```text
fasterlio_ws/
└── src/
    └── faster-lio/
```

### `fasterlio_ws/src/faster-lio/`

`FASTER-LIO` backend used as the fourth SLAM baseline.

Representative files:

- `config/mid360.yaml`
- `launch/mapping_mid360.launch`
- `src/laser_mapping.cc`

## How the Repository Fits Together

At a high level:

1. `LIO-SAM-MID360`, `FAST_LIO`, `Point-LIO`, and `faster-lio` provide the four SLAM backends
2. `mid360_navigation` provides the shared navigation stack used to benchmark them fairly
3. `fyp_utils` provides the experiment scripts, evaluators, summaries, and report materials

This structure reflects the practical workflow of the project: algorithm integration, shared navigation, controlled evaluation, and final documentation.
