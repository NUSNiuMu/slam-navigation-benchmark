# Notion 笔记 3：静态 Rosbag 四算法离线评测（轨迹导出 + evo + 性能）

## 目标
对 `liosam / fastlio / pointlio / fasterlio` 在同一 `rosbag` 上做统一离线评测，自动完成：
- 轨迹导出（TUM）
- `evo` 计算 `ATE` 与 `RPE`
- 统计 `CPU`、`内存`、`每帧处理时间`

## 脚本位置
- 主脚本：`/home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh`
- 监控脚本：`/home/niumu/ws_livox/src/fyp_utils/scripts/slam_eval_monitor.py`
- 轨迹导出：`/home/niumu/ws_livox/src/fyp_utils/scripts/bag_to_tum.py`
- 结果汇总：`/home/niumu/ws_livox/src/fyp_utils/scripts/summarize_static_eval.py`

## 一键运行（四算法）
```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/short_test_20260318_171639_2026-03-18-17-16-42_0.bag \
  --gt-topic /odom
```

## 常用参数
- 只跑单算法：
```bash
--algorithms liosam
```
- 指定播放倍率：
```bash
--play-rate 1.0
```
- 指定输出目录：
```bash
--out-root /home/niumu/ws_livox/src/fyp_utils/results/static_eval_20260318
```
- 若 bag 的真值里程计不是 `/odom`，替换为你实际话题：
```bash
--gt-topic /your/ground_truth_topic
```

## 输出结构
每个算法目录下会有：
- `estimated.bag`：算法输出里程计录包
- `traj_est.tum`：估计轨迹
- `traj_gt.tum`：真值轨迹
- `evo_ape.txt` / `evo_rpe.txt`：evo 结果文本
- `evo_ape.zip` / `evo_rpe.zip`：evo 原始结果
- `metrics/summary.json`：CPU/内存/帧耗时统计
- `metrics/resource_usage.csv`：资源采样序列
- `metrics/frame_times.csv`：逐帧耗时序列

总目录下会有：
- `summary.csv`
- `summary.md`

## 指标说明
- `ATE`：由 `evo_ape` 计算
- `RPE`：由 `evo_rpe` 计算（`delta=1s`）
- `CPU`：采样所有算法 ROS 进程总 CPU 占比
- `内存`：采样所有算法 ROS 进程总 RSS（MB）
- `每帧处理时间`：输出里程计消息回调的相邻 wall-time 间隔

## 依赖检查
需确保命令存在：
- `evo_ape`
- `evo_rpe`
- `rosbag`
- `rosnode`
- `rostopic`
- `python3`

## 注意
1. 默认假设：
- `liosam` 输出 `/odometry/imu`
- `fastlio/pointlio/fasterlio` 输出 `/slam/odom`
2. 若你的话题映射与上述不一致，需要修改 `eval_static_rosbag.sh` 中 `odom_topic_for_alg()`。
3. 若 `summary.md` 中 `ATE/RPE` 为 `NA`，优先检查：
- `traj_gt.tum` 是否生成
- `--gt-topic` 是否正确
- `evo_ape.txt` / `evo_rpe.txt` 是否报错
