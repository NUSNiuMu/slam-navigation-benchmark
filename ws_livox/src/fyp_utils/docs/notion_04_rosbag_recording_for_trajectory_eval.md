# Notion 笔记 4：ATE/RPE 评测专用 Rosbag 录制规范（四算法离线对比）

## 文档目的
这份文档用于规范“可用于轨迹误差评估”的 rosbag 录制流程，确保后续可以稳定计算：
- ATE（绝对轨迹误差）
- RPE（相对位姿误差）

适用场景：`LIO-SAM / FAST-LIO / Point-LIO / FASTER-LIO` 离线统一评测。

---

## 一、为什么之前会出现 ATE/RPE = NA
在 2026-03-18 的一次评测中，bag 仅包含：
- `/livox/lidar`
- `/livox/imu`
- `/tf_static`

没有真实轨迹（ground truth）话题，因此无法计算 ATE/RPE。

结论：**后续录包必须包含一个“真值轨迹话题”**（如 `/odom` 或 `/ground_truth/odom`）。

---

## 二、录包前必须确认
先确认真值话题存在且在发布：

```bash
rostopic list | egrep 'odom|ground|truth|mocap|vicon'
rostopic hz /odom
```

如果你不用 `/odom`，把下面命令中的 `/odom` 替换为真实 GT 话题。

---

## 三、推荐录包命令

### 方案 A：使用现有 launch（推荐）

```bash
roslaunch fyp_utils record_dataset.launch \
  bag_dir:=/home/bingda/PointCloudBag \
  bag_prefix:=eval_$(date +%Y%m%d_%H%M%S)
```

注意：请确认该 launch 内 `rosbag record` 已包含 GT 话题（如 `/odom`）。

### 方案 B：手动 rosbag record（最清晰）

```bash
rosbag record -O /home/bingda/PointCloudBag/eval_$(date +%Y%m%d_%H%M%S).bag \
  /livox/lidar /livox/imu /tf /tf_static /odom
```

如果 GT 不是 `/odom`，改成：

```bash
rosbag record -O /home/bingda/PointCloudBag/eval_$(date +%Y%m%d_%H%M%S).bag \
  /livox/lidar /livox/imu /tf /tf_static /your_gt_topic
```

---

## 四、录包后立即自检（必须做）

```bash
rosbag info /home/bingda/PointCloudBag/<your_bag>.bag
```

检查点：
1. `topics` 中必须出现你的 GT 话题（`/odom` 或 `/your_gt_topic`）
2. GT 话题消息数必须 > 0
3. `/livox/lidar`、`/livox/imu`、`/tf`、`/tf_static` 都在列表中

---

## 五、与离线评测脚本的对接
评测命令示例：

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/<your_bag>.bag \
  --gt-topic /odom \
  --no-rviz true
```

若真值不是 `/odom`：

```bash
bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
  --bag /home/niumu/FYP/PointCloudBag/<your_bag>.bag \
  --gt-topic /your_gt_topic \
  --no-rviz true
```

---

## 六、实验记录建议（便于写报告）
每个 bag 建议记录以下信息：
- 录制日期与场景（室内/室外、路线描述）
- 起终点与运动时长
- GT 来源（轮速里程计 / mocap / vicon / 其他）
- 是否有激烈运动或打滑
- 使用的评测命令与结果目录

---

## 七、一句话标准
**能用于 ATE/RPE 的 bag = 传感器数据 + TF + 有效 GT 轨迹话题（消息数 > 0）**。
