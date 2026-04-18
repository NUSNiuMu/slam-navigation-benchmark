# 毕业设计报告初稿

题目：基于 Livox MID360 的多 SLAM 移动机器人导航系统设计、调试与评测  
作者：[待填写]  
学号：[待填写]  
专业：[待填写]  
指导教师：[待填写]  
完成时间：2026 年 4 月

## 写作结构说明
本初稿结构参考了国内高校本科毕业设计（论文）常见规范，正文组织为“摘要/关键词—绪论—系统设计与实现—实验与分析—结论—参考文献—附录”。格式依据主要参考：

1. 东南大学《本科生毕业设计（论文）撰写规范（2019年9月修订）》  
   https://cs.seu.edu.cn/2024/0328/c49468a485621/page.htm
2. 长沙理工大学《本科生毕业设计（论文）撰写规范》  
   https://www.csust.edu.cn/dq/info/1039/3223.htm

以下内容已经结合本地工程记录、脚本、实验结果与调试过程整理，可作为后续 Word 版论文或答辩材料的基础稿。

---

## 摘要
本课题面向基于 Livox MID360 激光雷达的小型移动机器人，围绕“统一导航底座 + 多种激光惯导里程计算法对比评测”展开设计与实现。项目以 ROS Noetic 为软件基础，以 `move_base + DWA + local/global costmap` 为统一导航框架，对 LIO-SAM、FAST-LIO、Point-LIO、FASTER-LIO 四种激光惯导算法进行统一接入，并构建了实车数据录制、点云地图生成、静态 rosbag 离线评测、在线 waypoint 导航评测等完整实验链路。

课题实现了以下几项关键工作：第一，设计并实现了基于 `fyp_utils` 的工程化工具链，包括远程部署、底盘速度平滑、统一启动脚本、导航评测脚本、静态 rosbag 评测脚本以及 PCD 到 2D 栅格地图转换脚本；第二，完成了 `mid360_navigation` 导航底座的统一化重构，使四种 SLAM 算法能够通过统一的 `/slam/odom` 与 `/slam/cloud_registered` 接口接入导航系统；第三，针对实车运行过程中出现的局部代价地图不更新、TF 冲突、rosbag 缺少 ground truth、节点无法干净退出、Point-LIO 与 FASTER-LIO 无法正确生成 `/scan`、LIO-SAM 保存地图时空点云崩溃等一系列工程问题进行了系统定位与修复；第四，建立了离线静态评测与在线 waypoint 导航评测体系，能够输出 ATE、RPE、CPU 占用、内存占用、导航成功率、耗时、路径长度、平均速度、最小障碍距离、恢复次数以及 CPU/GPU 占用等指标。

已有实验结果表明，在静态 rosbag 评测中，四种算法均能够完成基本定位建图任务，其中 LIO-SAM 与 FAST-LIO、FASTER-LIO 在轨迹精度上表现接近，Point-LIO 的误差相对较大；在当前已完成的在线 waypoint 导航评测中，LIO-SAM 与 FAST-LIO 均取得了 100% 的到达成功率，其中 FAST-LIO 在总时间和平均速度上略优于 LIO-SAM，而 LIO-SAM 与 FAST-LIO 在障碍最小距离和控制平滑性上又呈现不同特点。工程调试过程还表明，不同 SLAM 算法对局部导航的影响不仅取决于定位精度，还与输出点云的坐标系、稠密度、是否为车体系点云、局部代价地图更新链路等因素密切相关。

本课题最终形成了一套面向 Livox MID360 实车平台的多算法导航对比实验框架，为后续继续开展四算法公平比较、参数优化和论文撰写提供了可复用的平台基础。

**关键词**：Livox MID360；LIO-SAM；FAST-LIO；Point-LIO；FASTER-LIO；移动机器人导航；局部代价地图；ROS

## Abstract
This project focuses on the design, debugging, and evaluation of a unified navigation framework for a mobile robot equipped with a Livox MID360 LiDAR. Based on ROS Noetic and a common `move_base + DWA + costmap` navigation stack, four LiDAR-inertial odometry algorithms, namely LIO-SAM, FAST-LIO, Point-LIO, and FASTER-LIO, were integrated into a unified experimental framework.

The work includes the development of a practical engineering toolkit for remote deployment, velocity smoothing, dataset recording, unified navigation startup, static rosbag evaluation, waypoint-based online navigation evaluation, and PCD-to-grid-map conversion. A standardized interface using `/slam/odom` and `/slam/cloud_registered` was built so that different SLAM algorithms could be compared under the same navigation stack. In addition, multiple system issues encountered during real robot deployment were analyzed and addressed, including local costmap update failures, TF conflicts, missing ground-truth topics in rosbag recordings, unstable node shutdown, missing scan generation in Point-LIO and FASTER-LIO, and empty-map crashes during LIO-SAM map saving.

Two evaluation pipelines were established. The first is an offline static rosbag evaluation pipeline for ATE, RPE, CPU usage, memory usage, and frame processing time. The second is an online waypoint-based navigation evaluation pipeline for success rate, completion time, path length, average speed, minimum obstacle distance, recovery count, control smoothness, and CPU/GPU usage. Experimental results show that LIO-SAM, FAST-LIO, and FASTER-LIO achieved comparable trajectory accuracy in static evaluation, while Point-LIO exhibited larger error. In online waypoint evaluation, LIO-SAM and FAST-LIO both achieved 100% success rate, while FAST-LIO showed slightly better efficiency and LIO-SAM behaved differently in obstacle clearance and control smoothness.

The project demonstrates that the navigation performance of a mobile robot depends not only on odometry accuracy, but also on the coordinate frame consistency, point cloud representation, and the real-time obstacle update chain from SLAM outputs to local costmap. The resulting system provides a reusable benchmark platform for further comparative studies and optimization.

**Key Words**: Livox MID360, LIO-SAM, FAST-LIO, Point-LIO, FASTER-LIO, mobile robot navigation, local costmap, ROS

---

## 1 绪论

### 1.1 课题背景
随着服务机器人、巡检机器人与室内外自主移动平台的发展，激光雷达与惯性测量单元融合的激光惯导定位技术逐渐成为移动机器人定位建图的重要方案。相较于纯里程计或纯视觉方案，激光惯导方法在光照变化、纹理不足、动态扰动等场景中通常具有更高的稳定性。Livox MID360 作为一款紧凑型 3D 激光雷达，具备较高的点云刷新率和内置 IMU，适合在小型移动机器人平台上部署。

然而，在实际工程中，仅有 SLAM 算法本身并不足以保证导航系统稳定工作。移动机器人完成自主导航还依赖地图构建、点云转激光、局部代价地图更新、路径规划、底盘控制平滑、多机通信与实验评测等完整链路。不同 SLAM 算法的输出频率、坐标系约定、点云类型与时延特征也会直接影响统一导航底座的性能。因此，有必要在同一实车平台和同一导航参数下，对多种激光惯导算法进行公平接入与系统级评测。

### 1.2 课题目标
本课题的总体目标是构建一套基于 Livox MID360 的多 SLAM 移动机器人导航实验平台，并完成以下任务：

1. 搭建统一的实车数据录制与回放链路。
2. 实现基于 `move_base` 的统一导航框架，支持多种 SLAM 算法快速切换。
3. 设计一套公平的静态与动态评测方法，对四种 SLAM 算法进行定量比较。
4. 系统整理工程实现过程中遇到的问题、定位思路与解决方法。
5. 为毕业设计报告与后续论文撰写形成可复用的实验材料与工程脚本。

### 1.3 研究内容
围绕上述目标，本课题具体完成了以下研究内容：

1. 对 `mid360_navigation` 进行统一化配置，完成 `map_server`、`pointcloud_to_laserscan`、`move_base` 与 `DWA` 的集成。
2. 使用 `run_slam_nav.sh` 对 LIO-SAM、FAST-LIO、Point-LIO 与 FASTER-LIO 进行统一接入。
3. 基于 `record_dataset.launch` 与 `cmd_vel_smoother.py` 完成实车数据标准化采集。
4. 基于 `eval_static_rosbag.sh` 与 `nav_waypoint_eval.py` 构建离线和在线两种评测体系。
5. 针对局部代价地图不更新、`/scan` 空白、TF 抖动、rosbag 指标缺失等问题开展定位与修复。

### 1.4 本课题的主要工作与创新点
相较于直接运行单个开源 SLAM 包，本课题的主要工程创新点体现在以下几个方面：

1. 搭建了一个面向四种激光惯导算法的统一导航实验平台。
2. 将数据录制、地图生成、导航启动、静态评测、在线评测组织为完整脚本链路，显著降低实验重复成本。
3. 将静态轨迹精度指标与在线导航行为指标结合起来，形成了更接近实际移动机器人应用场景的综合评测方法。
4. 系统地记录并解决了多个实际工程问题，为类似平台复现实验提供了经验总结。

---

## 2 系统总体方案

### 2.1 硬件与软件环境
本项目所使用的主要硬件与软件环境如下：

1. 传感器：Livox MID360 激光雷达（含 IMU）。
2. 机器人平台：差速驱动小车底盘。
3. 操作系统：Ubuntu 20.04 / Ubuntu 18.04 混合环境。
4. ROS 版本：ROS Noetic。
5. 导航框架：`move_base` + `DWAPlannerROS`。
6. 地图表示：2D 栅格静态地图 + local/global costmap。
7. 对比算法：LIO-SAM、FAST-LIO、Point-LIO、FASTER-LIO。

### 2.2 系统架构
系统整体可以分为五层：

1. 传感器层：Livox MID360 发布 `/livox/lidar` 与 `/livox/imu`。
2. SLAM 层：四种激光惯导算法分别输出位姿里程计与点云。
3. 接口统一层：通过 `topic_tools relay` 将不同算法映射到统一话题：
   - `/slam/odom`
   - `/slam/cloud_registered`
4. 导航层：`pointcloud_to_laserscan` 将统一点云转为 `/scan`，`move_base` 在静态地图与 local costmap 上完成规划与避障。
5. 评测层：离线 `evo` 轨迹评测 + 在线 waypoint 导航评测。

### 2.3 四种算法统一接入方式
统一接入脚本为：
- [run_slam_nav.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh)

该脚本的核心作用是：

1. 启动对应 SLAM 算法。
2. 将其 odom 与点云桥接到统一导航接口。
3. 启动 `mid360_navigation/navigation.launch`。
4. 在退出时清理 `move_base`、`pointcloud_to_laserscan`、`map_server`、relay 节点以及各 SLAM 进程，避免重复残留。

截至 2026-04-05，该脚本中四算法的统一接口策略为：

| 算法 | 统一 odom 来源 | 统一点云来源 | `scan_frame` |
|---|---|---|---|
| LIO-SAM | `/odometry/imu` | `/lio_sam/mapping/cloud_registered_raw` | `base_link` |
| FAST-LIO | `/Odometry` | `/cloud_registered` | `base_link` |
| Point-LIO | `/aft_mapped_to_init` | `/cloud_registered_body` | `base_link` |
| FASTER-LIO | `/Odometry` | `/cloud_registered_body` | `base_link` |

其中，Point-LIO 与 FASTER-LIO 最终采用 `body-frame` 点云，是由于实际调试表明，直接使用 world-frame `/cloud_registered` 再变换到 `base_link` 生成 `/scan`，会导致局部代价地图更新延迟甚至空白。

---

## 3 关键功能实现

### 3.1 实车录包与远程部署
为降低小车端手动部署负担，项目设计了远程同步和录包标准化流程。

相关文件：
- [remote_deploy_fyp_utils.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/remote_deploy_fyp_utils.sh)
- [record_dataset.launch](/home/niumu/ws_livox/src/fyp_utils/launch/record_dataset.launch)
- [notion_01_rosbag_recording.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_01_rosbag_recording.md)

`record_dataset.launch` 集成了以下功能：

1. 启动 `cmd_vel_smoother.py`，将 `/cmd_vel_raw` 平滑后输出到 `/cmd_vel`。
2. 发布 `base_link -> livox_frame` 静态 TF。
3. 将 `/raw_odom` 统一桥接为 `/odom`。
4. 录制 `/livox/lidar`、`/livox/imu`、`/odom`、`/tf`、`/tf_static`。

通过该流程，后续 `ATE/RPE` 离线评测所需的 ground truth 轨迹话题被纳入 rosbag 标准录制范围。

### 3.2 底盘速度平滑
为解决手动遥控时指令阶跃过大、底盘加减速突兀的问题，项目实现了速度平滑节点：
- [cmd_vel_smoother.py](/home/niumu/ws_livox/src/fyp_utils/scripts/cmd_vel_smoother.py)

其核心做法为：

1. 订阅 `/cmd_vel_raw`。
2. 对线速度与角速度分别进行加速度/减速度限幅。
3. 以固定频率发布平滑后的 `/cmd_vel`。

当前在 [record_dataset.launch](/home/niumu/ws_livox/src/fyp_utils/launch/record_dataset.launch) 中采用的默认参数为：

- `accel_lim_v = 0.12`
- `accel_lim_w = 0.30`
- `decel_lim_v = 0.18`
- `decel_lim_w = 0.45`

这一参数组合优先保证实车数据采集时的平稳性。

### 3.3 统一导航底座
导航主入口为：
- [navigation.launch](/home/niumu/ws_livox/src/mid360_navigation/launch/navigation.launch)

其内部由三部分组成：

1. `map_server`：加载 `map_yaml` 指定的静态地图。
2. `pointcloud_to_laserscan`：将 `/slam/cloud_registered` 转为 `/scan`。
3. `move_base`：加载 local/global costmap 与 `DWAPlannerROS` 参数。

与之配套的关键配置文件包括：

- [pointcloud_to_laserscan.launch](/home/niumu/ws_livox/src/mid360_navigation/launch/pointcloud_to_laserscan.launch)
- [move_base.launch](/home/niumu/ws_livox/src/mid360_navigation/launch/move_base.launch)
- [costmap_common_params.yaml](/home/niumu/ws_livox/src/mid360_navigation/config/costmap_common_params.yaml)
- [local_costmap_params.yaml](/home/niumu/ws_livox/src/mid360_navigation/config/local_costmap_params.yaml)
- [global_costmap_params.yaml](/home/niumu/ws_livox/src/mid360_navigation/config/global_costmap_params.yaml)
- [dwa_local_planner_params.yaml](/home/niumu/ws_livox/src/mid360_navigation/config/dwa_local_planner_params.yaml)

### 3.4 PCD 点云地图到 2D 栅格地图
为了将 FAST-LIO 等算法输出的 PCD 地图转成导航可用的 `pgm + yaml` 地图，本项目编写了：
- [pcd_to_grid.py](/home/niumu/ws_livox/src/mid360_navigation/scripts/pcd_to_grid.py)

该脚本主要完成以下步骤：

1. 读取二进制 PCD 点云。
2. 提取 XYZ 点并按 `z` 范围过滤。
3. 在给定分辨率下建立 2D 栅格。
4. 使用二维直方图和点数阈值抑制噪声。
5. 输出 `.pgm` 与 `.yaml` 文件。

例如，2026-04-02 由 FAST-LIO 导出的 PCD 已生成新地图：
- [map_fastlio_20260402.pgm](/home/niumu/ws_livox/src/mid360_navigation/maps_fastlio_20260402/map_fastlio_20260402.pgm)
- [map_fastlio_20260402.yaml](/home/niumu/ws_livox/src/mid360_navigation/maps_fastlio_20260402/map_fastlio_20260402.yaml)

且没有覆盖默认地图：
- [map.pgm](/home/niumu/ws_livox/src/mid360_navigation/maps/map.pgm)
- [map.yaml](/home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml)

### 3.5 静态 rosbag 评测脚本
为了统一比较四算法在同一 rosbag 上的定位精度与资源消耗，项目构建了静态评测脚本链路。相关脚本包括：

- [eval_static_rosbag.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh)
- [slam_eval_monitor.py](/home/niumu/ws_livox/src/fyp_utils/scripts/slam_eval_monitor.py)
- [bag_to_tum.py](/home/niumu/ws_livox/src/fyp_utils/scripts/bag_to_tum.py)
- [summarize_static_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_static_eval.py)

该评测链路支持：

1. 播放同一 rosbag。
2. 导出估计轨迹与真值轨迹为 TUM 格式。
3. 使用 `evo_ape` 与 `evo_rpe` 计算 ATE/RPE。
4. 统计 CPU、内存与帧间处理时间。
5. 输出 `summary.md` 汇总表。

### 3.6 在线 waypoint 导航评测脚本
为对比四算法在相同地图与目标点集下的导航效果，本项目实现了以下脚本：

- [capture_waypoints.py](/home/niumu/ws_livox/src/fyp_utils/scripts/capture_waypoints.py)
- [nav_waypoint_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py)
- [eval_nav_4alg.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/eval_nav_4alg.sh)
- [summarize_nav_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_nav_eval.py)

其中：

1. `capture_waypoints.py` 用于在 RViz 中采集 waypoint。
2. `nav_waypoint_eval.py` 用于单算法导航评测。
3. `eval_nav_4alg.sh` 用于四算法批量导航评测。
4. `summarize_nav_eval.py` 用于汇总结果为 `summary.md`。

当前在线导航评测脚本已经支持输出以下指标：

1. 成功率、总耗时、总路径长度、平均速度。
2. 全程最小障碍距离。
3. goal 位置/姿态误差。
4. 起点/终点位置与姿态差。
5. 恢复次数、近碰撞次数。
6. 线速度与角速度波动标准差。
7. CPU / GPU / GPU 显存占用。

---

## 4 调试问题与解决方法汇总

本课题在工程实现阶段经历了大量联调问题。为了体现系统搭建的完整性，下面按“现象—原因—解决”方式总结关键问题。

### 4.1 关键问题汇总表

| 问题现象 | 原因分析 | 解决方法 | 相关文件 |
|---|---|---|---|
| 遥控时底盘加减速过猛，录包质量差 | `/cmd_vel` 阶跃太大，底盘直接执行原始指令 | 实现 `cmd_vel_smoother.py`，在 `record_dataset.launch` 中加入平滑节点并降低加减速限制 | [cmd_vel_smoother.py](/home/niumu/ws_livox/src/fyp_utils/scripts/cmd_vel_smoother.py), [record_dataset.launch](/home/niumu/ws_livox/src/fyp_utils/launch/record_dataset.launch) |
| 初期静态 rosbag 评测 ATE/RPE 为 `NA` | rosbag 未录制 ground truth 话题，只有雷达、IMU 和 TF | 将 `/odom` 纳入标准录包话题，形成 ATE/RPE 评测专用录包规范 | [notion_04_rosbag_recording_for_trajectory_eval.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_04_rosbag_recording_for_trajectory_eval.md), [record_dataset.launch](/home/niumu/ws_livox/src/fyp_utils/launch/record_dataset.launch) |
| `run_slam_nav.sh` 启停后残留节点无法关闭 | `roslaunch`、relay 和导航子进程未被完整回收 | 在脚本中增加多层清理逻辑、进程组终止、`rosnode cleanup` 和残留 relay 清理 | [run_slam_nav.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh) |
| LIO-SAM 保存地图时报空点云崩溃 | `save_map` 时没有关键帧或全局点云为空，触发 `pcl::PCDWriter::writeBinary` 异常 | 在 `mapOptmization.cpp` 中增加空关键帧和空全局地图保护 | [mapOptmization.cpp](/home/niumu/ws_livox/src/LIO-SAM-MID360/src/mapOptmization.cpp) |
| 局部代价地图不更新，但 RViz 中点云能看到 | `Point-LIO/FASTER-LIO` 默认未发布 body-frame 点云，world-frame 点云再转 `/scan` 导致延迟或空白 | 启用 `scan_bodyframe_pub_en`，改用 `/cloud_registered_body` 作为 `/slam/cloud_registered` 输入 | [run_slam_nav.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh), `/home/niumu/fastlio2_ws/src/Point-LIO/config/mid360.yaml`, `/home/niumu/fasterlio_ws/src/faster-lio/config/mid360.yaml` |
| `FASTER-LIO` 的 `/scan` 有内容但 local costmap 更新很慢 | 导航使用的是 world-frame `/cloud_registered`，还要再次变回 `base_link` | 改为直接使用 `/cloud_registered_body`，减少一次重投影链路 | [run_slam_nav.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh) |
| `roslaunch lio_sam run6axis.launch` 配合 `rosbag play` 时 RViz 位姿抖动 | rosbag 回放中含有 `/tf`/`/tf_static`，同时当前系统也在发布 TF，造成冲突 | 离线回放时只播放 `/livox/lidar` 和 `/livox/imu`，并使用 `--clock` 与 `use_sim_time` | [worklog_2026-03-18.txt](/home/niumu/ws_livox/src/fyp_utils/docs/worklog_2026-03-18.txt) |
| 在线评测脚本第一次写日志时报 `events.log` 路径不存在 | `nav_waypoint_eval.py` 写日志前未自动创建 `--out-dir` | 在脚本中补充 `os.makedirs(out_dir, exist_ok=True)` | [nav_waypoint_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py) |
| 评测中出现 `status=UNKNOWN`、`cmd=(0,0)` 且不动 | `/move_base_simple/goal` 订阅连接未准备好，goal 可能发丢 | 在评测脚本中等待 goal 订阅者并多次发布 goal | [nav_waypoint_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py) |
| 生成新地图时不想覆盖默认地图 | 默认 `map.yaml` 会被复用，容易污染稳定导航配置 | 使用 `pcd_to_grid.py` 输出到独立目录，并通过 `map_yaml:=...` 手动加载新地图 | [pcd_to_grid.py](/home/niumu/ws_livox/src/mid360_navigation/scripts/pcd_to_grid.py), [navigation.launch](/home/niumu/ws_livox/src/mid360_navigation/launch/navigation.launch) |

### 4.2 局部代价地图问题的定位过程
本项目中最棘手的问题之一是 local costmap 更新不稳定。具体表现包括：

1. `/scan` 话题存在，但 local costmap 不更新。
2. 抬起车体一段距离后局部地图会恢复。
3. 不同 SLAM 算法下表现差异明显。

调试过程中的关键发现包括：

1. local costmap 是否更新不仅与点云是否存在有关，更与点云的 frame、稠密度以及是否为车体系点云直接相关。
2. 对于 FAST-LIO、Point-LIO、FASTER-LIO 这类算法，如果导航链路使用 world-frame 的 `/cloud_registered` 再转成 `/scan`，会引入时延与重投影误差。
3. 对于 Point-LIO 和 FASTER-LIO，必须启用 `scan_bodyframe_pub_en` 并优先使用 `/cloud_registered_body`，local costmap 才能获得足够实时的障碍信息。

这一过程说明，统一导航框架下的公平比较，不仅要统一 DWA 参数和地图，还必须统一“用于生成 `/scan` 的点云语义”。

### 4.3 LIO-SAM 的参数探索与回退
在调试过程中，还曾尝试将 LIO-SAM 的雷达 frame 从 `base_link` 调整为 `livox_frame`，并在 URDF 中加入 `base_link -> livox_frame = 0.2m` 的安装偏移，以更严格地表达真实安装关系。但在后续联调中，为保证对照实验的一致性与系统稳定性，最终按照需求将 LIO-SAM 回退到：

1. `lidarFrame: base_link`
2. `baselinkFrame: base_link`
3. `cloud_registered_raw` 作为导航输入点云

这一过程表明，在工程系统中，理论上更精确的建模方案并不总能立即带来更好的综合效果，系统稳定性与已有链路兼容性同样重要。

---

## 5 实验设计

### 5.1 静态 rosbag 离线评测设计
静态评测用于在同一数据集上比较四种 SLAM 算法的定位建图精度与资源开销。

#### 5.1.1 评测流程
1. 使用包含 `/livox/lidar`、`/livox/imu`、`/odom`、`/tf`、`/tf_static` 的 rosbag。
2. 对四种算法分别离线运行。
3. 导出估计轨迹与真值轨迹为 TUM 文件。
4. 使用 `evo_ape` 与 `evo_rpe` 计算轨迹误差。
5. 使用监控脚本记录 CPU、内存和帧耗时。

#### 5.1.2 评价指标
1. `ATE RMSE`：绝对轨迹误差。
2. `RPE RMSE`：相对位姿误差。
3. `CPU Mean`：平均 CPU 占用。
4. `Mem Mean`：平均内存占用。
5. `Frame Mean`：平均帧间隔。
6. `Odom Msg Count`：输出里程计消息数量。

### 5.2 在线 waypoint 导航评测设计
在线导航评测用于比较同一地图、同一 waypoint 集下，不同 SLAM 输出对导航系统的影响。

#### 5.2.1 评测流程
1. 固定静态地图。
2. 固定 waypoint 文件。
3. 保持 `move_base` 与 DWA 参数一致。
4. 分别启动 LIO-SAM、FAST-LIO、Point-LIO、FASTER-LIO。
5. 使用 `nav_waypoint_eval.py` 自动发布目标点并记录结果。

#### 5.2.2 评价指标
在线导航评测不采用 ATE/RTE，而重点使用更接近导航任务的指标：

1. `success_rate`
2. `total_time_s`
3. `total_path_m`
4. `avg_speed_all_mps`
5. `min_scan_dist_all_m`
6. `total_recovery_count`
7. `total_near_collision_count`
8. `avg_goal_pos_error_m`
9. `avg_goal_yaw_error_rad`
10. `end_to_start_error_m`
11. `end_to_start_yaw_error_rad`
12. `avg_cmd_lin_std`
13. `avg_cmd_ang_std`
14. `avg_cpu_percent` / `avg_gpu_percent`

---

## 6 实验结果与分析

### 6.1 静态 rosbag 评测结果
根据 2026-03-23 的最新静态评测汇总结果：
- [summary.md](/home/niumu/FYP/static_eval_results/static_eval_20260323_184858/summary.md)

得到结果如下：

| 算法 | ATE RMSE/m | RPE RMSE | CPU 平均/% | 内存平均/MB | 平均帧间隔/s | Odom 消息数 |
|---|---:|---:|---:|---:|---:|---:|
| LIO-SAM | 0.141737 | 0.010850 | 57.855198 | 229.855216 | 0.005000 | 36682 |
| FAST-LIO | 0.142670 | 0.006296 | 33.503534 | 478.383343 | 0.100001 | 1837 |
| Point-LIO | 0.203076 | 0.021380 | 30.259501 | 203.800959 | 0.100006 | 1836 |
| FASTER-LIO | 0.144304 | 0.003444 | 122.935643 | 488.948487 | 0.100002 | 1837 |

#### 结果分析
1. 从 ATE 看，LIO-SAM、FAST-LIO、FASTER-LIO 三者非常接近，Point-LIO 误差较大。
2. 从 RPE 看，FASTER-LIO 最优，说明其局部连续性最好；FAST-LIO 次之，LIO-SAM 略大，Point-LIO 最大。
3. 从 CPU 占用看，FASTER-LIO 代价最高，LIO-SAM 次之，FAST-LIO 和 Point-LIO 相对更低。
4. 从消息频率看，LIO-SAM 里程计输出频率更高，而其余三种算法的输出频率大致在 10Hz 量级。

值得注意的是，在 2026-03-18 的早期评测中，ATE/RPE 一度为 `NA`。后来通过将 `/odom` 纳入标准录包流程后，2026-03-19 与 2026-03-23 的评测才得到有效误差结果，这说明统一的 ground truth 录制规范是静态评测成立的前提。

### 6.2 在线 waypoint 导航评测结果
当前已有较完整在线评测结果的算法包括 LIO-SAM 与 FAST-LIO：

- [manual_liosam/summary.json](/home/niumu/FYP/nav_eval_results/manual_liosam/summary.json)
- [manual_fastlio/summary.json](/home/niumu/FYP/nav_eval_results/manual_fastlio/summary.json)

汇总如下：

| 指标 | LIO-SAM | FAST-LIO |
|---|---:|---:|
| goal_count | 5 | 5 |
| success_count | 5 | 5 |
| success_rate | 1.000000 | 1.000000 |
| total_time_s | 123.550883 | 120.300767 |
| total_path_m | 29.567586 | 29.425598 |
| avg_speed_all_mps | 0.239315 | 0.244600 |
| min_scan_dist_all_m | 0.657686 | 0.309164 |
| total_recovery_count | 0 | 0 |
| avg_goal_pos_error_m | 0.236822 | 0.240930 |
| avg_goal_yaw_error_rad | 1.319621 | 1.301457 |
| avg_cpu_percent | 12.559016 | 12.448333 |
| avg_gpu_percent | 13.163934 | 15.475000 |
| avg_cmd_lin_std | 0.028561 | 0.022158 |
| avg_cmd_ang_std | 0.159488 | 0.104466 |
| end_to_start_error_m | 0.239838 | 0.243477 |
| end_to_start_yaw_error_rad | 0.266475 | 0.151207 |

#### 结果分析
1. 在本次 waypoint 测评中，LIO-SAM 与 FAST-LIO 均达到 100% 成功率。
2. FAST-LIO 在总耗时和平均速度上略优于 LIO-SAM。
3. LIO-SAM 的最小障碍距离更大，说明在当前参数下其局部避障更保守；FAST-LIO 虽然更快，但更贴近障碍物。
4. FAST-LIO 的线速度与角速度波动略小，控制输出更平稳。
5. 两者回到起点的终点误差接近，说明在短程导航任务中二者的闭环返回效果相近。

### 6.3 Point-LIO 与 FASTER-LIO 在线导航结果说明
截至本报告整理时，Point-LIO 与 FASTER-LIO 的静态 rosbag 评测已经完成，但在线 waypoint 导航的最终稳定数值尚未全部补齐。其原因不是评测脚本缺失，而是 local costmap 的输入链路在调试中经历了以下过程：

1. 初期使用 `/cloud_registered` 作为导航输入，导致 `/scan` 不稳定或 local costmap 更新延迟。
2. 后续通过检查源码和配置，确认其更适合使用 `/cloud_registered_body`。
3. 对 Point-LIO 还额外启用了 `scan_bodyframe_pub_en: true`。

因此，本项目已经完成了 Point-LIO 和 FASTER-LIO 的导航接入链路修复，但正式四算法全量在线定量结果仍需在统一场景下补测。这部分可以作为论文答辩前的补充实验工作。

### 6.4 关于“同样参数但不同算法行为不同”的讨论
实验表明，在相同 DWA 与 costmap 参数下，不同 SLAM 算法的导航表现仍存在显著差异。这说明：

1. 导航系统行为不仅取决于规划参数，还强烈依赖 SLAM 输出的 odom 与点云质量。
2. `/scan` 的时延、坐标系和点云稠密度会直接影响 local costmap 的障碍刷新速度。
3. 一个算法“跑得更快”，并不一定代表其导航更优，也可能意味着其感知到的近距离障碍更少或更稀疏。

因此，多算法公平比较必须建立在统一地图、统一目标点、统一导航参数、统一点云语义与统一评测指标之上。

---

## 7 脚本与工程成果整理

本项目已经形成了一套较完整的工程化脚本集合，见：
- [/home/niumu/ws_livox/src/fyp_utils/scripts](/home/niumu/ws_livox/src/fyp_utils/scripts)

### 7.1 脚本功能汇总

| 脚本/文件 | 作用 |
|---|---|
| [remote_deploy_fyp_utils.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/remote_deploy_fyp_utils.sh) | 将 `fyp_utils` 自动同步到小车并编译 |
| [cmd_vel_smoother.py](/home/niumu/ws_livox/src/fyp_utils/scripts/cmd_vel_smoother.py) | 对 `/cmd_vel_raw` 进行加减速限幅 |
| [record_dataset.launch](/home/niumu/ws_livox/src/fyp_utils/launch/record_dataset.launch) | 标准化录包，集成平滑与静态 TF |
| [run_slam_nav.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh) | 一键启动指定算法并接入统一导航 |
| [capture_waypoints.py](/home/niumu/ws_livox/src/fyp_utils/scripts/capture_waypoints.py) | 采集 RViz waypoint |
| [nav_waypoint_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py) | 单算法在线导航评测 |
| [eval_nav_4alg.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/eval_nav_4alg.sh) | 四算法自动导航评测 |
| [summarize_nav_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_nav_eval.py) | 汇总在线导航结果 |
| [eval_static_rosbag.sh](/home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh) | 四算法静态 rosbag 离线评测 |
| [slam_eval_monitor.py](/home/niumu/ws_livox/src/fyp_utils/scripts/slam_eval_monitor.py) | 静态评测过程的资源监控 |
| [bag_to_tum.py](/home/niumu/ws_livox/src/fyp_utils/scripts/bag_to_tum.py) | 将轨迹导出为 TUM 格式 |
| [summarize_static_eval.py](/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_static_eval.py) | 汇总静态评测结果 |
| [pcd_to_grid.py](/home/niumu/ws_livox/src/mid360_navigation/scripts/pcd_to_grid.py) | PCD 点云生成导航地图 |

### 7.2 文档成果
项目还形成了多份实验操作与记录文档：

1. [notion_01_rosbag_recording.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_01_rosbag_recording.md)
2. [notion_02_navigation_usage.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_02_navigation_usage.md)
3. [notion_03_static_rosbag_eval.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_03_static_rosbag_eval.md)
4. [notion_04_rosbag_recording_for_trajectory_eval.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_04_rosbag_recording_for_trajectory_eval.md)
5. [notion_05_nav_eval_4alg_waypoints.md](/home/niumu/ws_livox/src/fyp_utils/docs/notion_05_nav_eval_4alg_waypoints.md)
6. [worklog_2026-03-18.txt](/home/niumu/ws_livox/src/fyp_utils/docs/worklog_2026-03-18.txt)

这些文档与脚本共同构成了本毕业设计的工程化成果。

---

## 8 结论与展望

### 8.1 结论
本课题以 Livox MID360 小车平台为对象，完成了从数据采集、地图生成、SLAM 接入、导航运行到离线与在线评测的完整系统搭建。通过对 LIO-SAM、FAST-LIO、Point-LIO 与 FASTER-LIO 的统一化接入与实验，可以得出以下结论：

1. 在统一导航框架下，SLAM 算法的输出不仅影响定位精度，也会显著影响 local costmap 的实时更新和局部规划行为。
2. 采用工程化脚本将多算法接入、地图生成和评测流程标准化，可以显著提高实验复现性和开发效率。
3. 静态 rosbag 评测表明，LIO-SAM、FAST-LIO 与 FASTER-LIO 的轨迹精度接近，而 Point-LIO 在当前数据集下误差较大。
4. 在线 waypoint 导航评测表明，LIO-SAM 与 FAST-LIO 均可实现稳定导航，但在速度、安全距离和控制平滑性方面存在差异。
5. 对 Point-LIO 与 FASTER-LIO 来说，body-frame 点云发布是 local costmap 正常工作的关键因素之一。

### 8.2 不足与后续工作
尽管系统主体已经搭建完成，但仍有以下工作可继续推进：

1. 补充 Point-LIO 与 FASTER-LIO 在修复后的四算法在线 waypoint 完整定量评测。
2. 进一步校准机器人 `footprint`，提升局部避障安全距离的一致性。
3. 将 LIO-SAM 的 odom 选择、frame 建模与导航性能之间的关系做更系统的实验分析。
4. 优化 `pointcloud_to_laserscan` 参数，进一步降低 `/scan` 时延。
5. 将当前 Markdown 报告整理为学校模板下的 Word/PDF 正式稿，并补充图表、系统框图、TF 树截图与 RViz 结果图。

---

## 参考文献

[1] 东南大学. 本科生毕业设计（论文）撰写规范（2019年9月修订）.  
https://cs.seu.edu.cn/2024/0328/c49468a485621/page.htm

[2] 长沙理工大学. 本科生毕业设计（论文）撰写规范.  
https://www.csust.edu.cn/dq/info/1039/3223.htm

[3] Shan T, Englot B, Meyers D, Wang W, Ratti C, Rus D. LIO-SAM: Tightly-coupled lidar inertial odometry via smoothing and mapping. In: 2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). 2020:5135-5142. DOI:10.1109/IROS45743.2020.9341176.  
https://researchwith.stevens.edu/en/publications/lio-sam-tightly-coupled-lidar-inertial-odometry-via-smoothing-and/

[4] Xu W, Cai Y, He D, Lin J, Zhang F. FAST-LIO2: Fast Direct LiDAR-Inertial Odometry. IEEE Transactions on Robotics, 2022, 38:2053-2073. DOI:10.1109/TRO.2022.3141876.  
https://www.scirp.org/reference/referencespapers?referenceid=4198527

[5] He D, Xu W, Chen N, Kong F, Yuan C, Zhang F. Point-LIO: Robust High-Bandwidth LiDAR-Inertial Odometry. Advanced Intelligent Systems, 2023, 5(7). DOI:10.1002/aisy.202200459.  
https://hub.hku.hk/handle/10722/331147

[6] Bai C, Xiao T, Chen Y, Wang H, Zhang F, Gao X. Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels. IEEE Robotics and Automation Letters, 2022, 7(2):4861-4868. DOI:10.1109/LRA.2022.3152830.  
https://github.com/gaoxiang12/faster-lio

[7] ROS Navigation Stack Documentation. Obstacle Layer Parameters / costmap_2d / DWA Local Planner.  
https://docs.ros.org/en/noetic/api/costmap_2d/html/classcostmap__2d_1_1ObstacleLayer.html

---

## 附录 A 关键命令示例

### A.1 四算法统一导航启动
统一入口：

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh liosam
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fastlio
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh pointlio
bash /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh fasterlio
```

### A.2 四算法静态 rosbag 评测

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/<your_bag>.bag \
  --gt-topic /odom
```

### A.3 单算法在线 waypoint 导航评测

```bash
source ~/ws_livox/devel/setup.bash
python3 /home/niumu/ws_livox/src/fyp_utils/scripts/nav_waypoint_eval.py \
  --algorithm fastlio \
  --waypoints /home/niumu/FYP/waypoints/test_route.yaml \
  --out-dir /home/niumu/FYP/nav_eval_results/manual_fastlio \
  --odom-topic /slam/odom \
  --scan-topic /scan \
  --cmd-topic /cmd_vel \
  --recovery-topic /move_base/recovery_status \
  --xy-tolerance 0.25 \
  --goal-timeout-sec 90 \
  --near-collision-threshold-m 0.25 \
  --append-start-goal
```

### A.4 PCD 转地图

```bash
python3 /home/niumu/ws_livox/src/mid360_navigation/scripts/pcd_to_grid.py \
  /home/niumu/fastlio2_ws/src/FAST_LIO/PCD/scans.pcd \
  /home/niumu/ws_livox/src/mid360_navigation/maps_fastlio_20260402/map_fastlio_20260402
```

### A.5 加载指定地图

```bash
source ~/ws_livox/devel/setup.bash && \
roslaunch mid360_navigation navigation.launch \
  map_yaml:=/home/niumu/ws_livox/src/mid360_navigation/maps_fastlio_20260402/map_fastlio_20260402.yaml \
  odom_topic:=/slam/odom \
  cloud_topic:=/slam/cloud_registered \
  scan_frame:=base_link \
  show_rviz:=true
```

