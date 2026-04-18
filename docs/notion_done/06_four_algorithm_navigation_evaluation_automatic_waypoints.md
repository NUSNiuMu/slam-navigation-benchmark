# Four-Algorithm Navigation Evaluation with Automatic Waypoints

- Source: Notion FYP database
- Status: `Done`
- Original page: <https://www.notion.so/32db3934766d818fa93fc4eb1f2456f0>

## Objective

Automatically compare the navigation performance of:

- `liosam`
- `fastlio`
- `pointlio`
- `fasterlio`

on the same map and the same waypoint set, and generate a unified summary.

## Preparation

### Fixed Map

Use one fixed map throughout the comparison. If a non-default map is needed, pass `--map-yaml`.

### Waypoint File

The evaluator reads target points from a waypoint YAML file rather than relying on manual clicking each time.

Example:

- `docs/waypoints/nav_waypoints_example.yaml`

## Recording Waypoints

```bash
source /opt/ros/noetic/setup.bash
source /home/niumu/ws_livox/devel/setup.bash
python3 /home/niumu/ws_livox/src/fyp_utils/scripts/capture_waypoints.py \
  --out /home/niumu/ws_livox/github_project_upload/docs/waypoints/nav_waypoints_0324.yaml
```

## Running the Evaluation

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_nav_4alg.sh \
  --waypoints /home/niumu/ws_livox/github_project_upload/docs/waypoints/nav_waypoints_0324.yaml
```

Common options:

- `--map-yaml /path/to/map.yaml`
- `--show-rviz true`
- `--goal-timeout-sec 120`
- `--xy-tolerance 0.25`
- `--algorithms liosam,fastlio,pointlio,fasterlio`

## Output

Result root:

- `/home/niumu/FYP/nav_eval_results/nav_eval_<timestamp>`

Per-algorithm outputs:

- `per_goal.csv`
- `summary.json`
- `run.log`
- `eval.log`

Overall summary:

- `summary.md`

## Core Metrics

- `success_rate`
- `avg_time_success_s`
- `avg_path_success_m`
- `min_scan_dist_all_m`
- `avg_cmd_lin_std`
- `avg_cmd_ang_std`

## Notes

- do not change the map or DWA settings during the four-algorithm comparison
- keep the start pose consistent
- dynamic obstacles can affect results, so the environment should be comparable across runs
