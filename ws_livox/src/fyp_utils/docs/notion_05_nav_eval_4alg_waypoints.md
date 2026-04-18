# 四算法导航测评（自动导航点版）

## 目标
在同一地图、同一导航点集下，自动对比 `liosam/fastlio/pointlio/fasterlio` 的导航效果，输出统一评测结果。

---

## 一、准备

### 1) 地图
使用固定地图，避免切换中途替换默认地图。  
如需指定非默认地图，后续执行脚本时传 `--map-yaml`。

### 2) 导航点文件（waypoints）
脚本通过 `waypoints.yaml` 知道目标点，不需要每次手点 RViz。

示例文件：
`/home/niumu/ws_livox/src/fyp_utils/docs/nav_waypoints_example.yaml`

结构：
```yaml
frame_id: map
goals:
  - name: corridor_start
    x: -0.5
    y: 0.0
    yaw: 0.0
  - name: corridor_mid
    x: 3.0
    y: 0.2
    yaw: 0.0
```

---

## 二、录制导航点（推荐）

先启动一套导航（任一算法即可），在 RViz 用 `2D Nav Goal` 点目标：

```bash
source /opt/ros/noetic/setup.bash
source /home/niumu/ws_livox/devel/setup.bash
python3 /home/niumu/ws_livox/src/fyp_utils/scripts/capture_waypoints.py \
  --out /home/niumu/ws_livox/src/fyp_utils/docs/nav_waypoints_0324.yaml
```

点完后 `Ctrl+C`，会自动保存 `frame_id + goals`。

---

## 三、运行四算法导航测评

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_nav_4alg.sh \
  --waypoints /home/niumu/ws_livox/src/fyp_utils/docs/nav_waypoints_0324.yaml
```

常用可选参数：
- `--map-yaml /path/to/map.yaml`
- `--show-rviz true`
- `--goal-timeout-sec 120`
- `--xy-tolerance 0.25`
- `--algorithms liosam,fastlio,pointlio,fasterlio`

---

## 四、输出结果

结果根目录：
`/home/niumu/FYP/nav_eval_results/nav_eval_<timestamp>`

每个算法目录包含：
- `per_goal.csv`：每个目标点的成败、耗时、路径长度、最小障碍距离、速度波动
- `summary.json`：算法级汇总
- `run.log` / `eval.log`：运行日志

总表：
- `summary.md`

核心指标：
- `success_rate`：到达成功率
- `avg_time_success_s`：成功目标平均耗时
- `avg_path_success_m`：成功目标平均路径长度
- `min_scan_dist_all_m`：全程最小障碍距离（越大越安全）
- `avg_cmd_lin_std / avg_cmd_ang_std`：速度抖动（越小越平顺）

---

## 五、注意事项

1. 四算法测试期间不要改地图和 DWA 参数。  
2. 起点姿态尽量一致。  
3. 环境动态障碍（行人）会影响结果，建议每算法测试时段保持相近环境。  
4. 出现 TF 时间戳报错时，先同步系统时间并重启整套节点再测。

---

## 六、相关脚本

- `/home/niumu/ws_livox/src/fyp_utils/scripts/capture_waypoints.py`
- `/home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py`
- `/home/niumu/ws_livox/src/fyp_utils/scripts/eval_nav_4alg.sh`
- `/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_nav_eval.py`
