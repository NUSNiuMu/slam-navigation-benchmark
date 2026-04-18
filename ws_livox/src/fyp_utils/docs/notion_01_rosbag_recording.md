# Notion 笔记 1：MID360 实车录制 Rosbag 操作手册

## 目标
将实车数据录制到小车目录：`/home/bingda/PointCloudBag`，用于后续 SLAM/导航算法对比。

## 一、一次性准备（如果尚未部署）
在本机（`niumu`）执行：

```bash
SSH_PASSWORD='bingda' REMOTE_WS='/home/bingda/catkin_ws' \
/home/niumu/ws_livox/src/fyp_utils/scripts/remote_deploy_fyp_utils.sh deploy
```

说明：该命令会自动把 `fyp_utils` 同步到小车、赋可执行权限并 `catkin_make`。

## 二、开始录包（小车端执行）
### 1) 启动底盘
```bash
roslaunch base_control base_control.launch
```

### 2) 启动录包+速度平滑+静态TF（一条命令）
```bash
roslaunch fyp_utils record_dataset.launch \
  bag_dir:=/home/bingda/PointCloudBag \
  bag_prefix:=fyp_run_$(date +%Y%m%d_%H%M%S)
```

该 launch 默认会：
- 启动 `cmd_vel_smoother`（`/cmd_vel_raw -> /cmd_vel`）
- 发布 `base_link -> livox_frame` 静态 TF
- 启动 rosbag 录制以下话题：
  - `/livox/lidar`
  - `/livox/imu`
  - `/odom`
  - `/tf_static`
- 启用分包：`--split --size=2048`（约 2GB 一包）

### 3) 键盘遥控请使用 raw 话题（避免翘头）
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_raw
```

## 三、结束录包
在录包终端按 `Ctrl+C`。

正常结束后会看到 `.bag` 文件（例如）：
`fyp_run_20260318_160928_2026-03-18-16-09-31_0.bag`

## 四、录包有效性检查
```bash
cd /home/bingda/PointCloudBag
rosbag info <你的包名>.bag
```

判断标准：
- 有 duration（时长）
- 有 messages（消息数）
- 话题存在且数量正常（至少应有 `/livox/lidar`、`/livox/imu`、`/tf_static`）

若出现索引异常：
```bash
rosbag reindex <你的包名>.bag
```

## 五、磁盘空间告警说明
若出现：`Less than 5 x 1G of space free ...`，表示剩余空间不足 5GB，录制仍可能继续，但有中断风险。

建议：
```bash
df -h /home/bingda/PointCloudBag
```
录制前尽量保证可用空间 >= 10GB。

## 六、将 bag 从小车拷回本机
目标目录（本机）：`/home/niumu/FYP/PointCloudBag`

```bash
scp bingda@172.20.10.2:/home/bingda/PointCloudBag/<你的包名>.bag \
    /home/niumu/FYP/PointCloudBag/
```

拷贝后本机验证：
```bash
rosbag info /home/niumu/FYP/PointCloudBag/<你的包名>.bag
```

## 七、常用参数（可选）
调整平滑强度：
```bash
roslaunch fyp_utils record_dataset.launch \
  accel_lim_v:=0.12 accel_lim_w:=0.30
```

更保守（更不容易翘头）：
```bash
roslaunch fyp_utils record_dataset.launch \
  accel_lim_v:=0.08 accel_lim_w:=0.20
```
