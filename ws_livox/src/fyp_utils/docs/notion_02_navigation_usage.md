# Notion 笔记 2：四算法一键导航使用手册（LIO-SAM / FAST-LIO / FASTER-LIO / Point-LIO）

## 目标
在同一套导航栈下，对 4 种 SLAM 算法做公平导航对比：
- `liosam`
- `fastlio`
- `fasterlio`
- `pointlio`

## 一、核心入口
统一使用脚本：
`/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh`

它会自动完成：
1. 启动你现有的 `/home/niumu/run_slam.sh <算法>`
2. 自动桥接到统一导航输入：
   - `/slam/odom`
   - `/slam/cloud_registered`
3. 启动导航栈 `mid360_navigation/navigation.launch`

## 二、使用前检查
```bash
source /opt/ros/noetic/setup.bash
source /home/niumu/ws_livox/devel/setup.bash
```

可选检查：
```bash
rospack find mid360_navigation
rospack find fyp_utils
```

## 三、一键启动命令
### 1) LIO-SAM
```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh liosam
```

### 2) FAST-LIO
```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fastlio
```

### 3) FASTER-LIO
```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fasterlio
```

### 4) Point-LIO
```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh pointlio
```

## 四、可选参数
显示 RViz：
```bash
SHOW_RVIZ=true bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh liosam
```

指定地图：
```bash
MAP_YAML=/home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml \
SHOW_RVIZ=true \
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fastlio
```

## 五、导航操作流程（每次实验一致）
1. 启动某个算法（一键脚本）
2. 在 RViz 设置固定坐标系 `map`
3. 使用 `2D Pose Estimate`（如需要）
4. 使用 `2D Nav Goal` 下发同一组目标点
5. 记录结果（到达时间、是否到达、轨迹平滑度、是否振荡）

## 六、对比实验建议（保证公平）
- 地图一致（同一 `map.yaml`）
- 起点和目标点一致
- 速度参数一致（`move_base` / `DWA` 不改）
- 每个算法至少重复 3 次
- 每次实验前清理上次节点，避免残留话题干扰

## 七、快速排障
### 1) `move_base` 无法出速度
检查：
```bash
rostopic hz /slam/odom
rostopic hz /scan
rostopic echo /move_base/status -n 1
```

### 2) 没有 `/scan`
检查点云桥接输入是否存在：
```bash
rostopic list | egrep 'cloud_registered|slam'
```

### 3) TF 报错
检查：
```bash
rosrun tf tf_echo map base_link
```

### 4) 算法能跑但导航不稳定
优先检查：
- 点云高度裁剪（`pointcloud_to_laserscan.launch` 里的 `min_height/max_height`）
- odom 是否连续且频率稳定
- 目标点是否落在可通行区域

## 八、当前统一导航配置位置
- 导航入口：`/home/niumu/ws_livox/src/mid360_navigation/launch/navigation.launch`
- move_base：`/home/niumu/ws_livox/src/mid360_navigation/launch/move_base.launch`
- 点云转激光：`/home/niumu/ws_livox/src/mid360_navigation/launch/pointcloud_to_laserscan.launch`
- DWA 参数：`/home/niumu/ws_livox/src/mid360_navigation/config/dwa_local_planner_params.yaml`
- costmap 参数：`/home/niumu/ws_livox/src/mid360_navigation/config/costmap_common_params.yaml`
