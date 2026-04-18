#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import shutil
import statistics
import subprocess
import time

import rospy
import yaml
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import RecoveryStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

try:
    import psutil
except ImportError:
    psutil = None


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def dist2d(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}


class NavEvalNode:
    def __init__(self, args):
        self.args = args
        self.latest_odom = None
        self.latest_scan_min = float("inf")
        self.latest_scan_front_min = float("inf")
        self.latest_cmd = (0.0, 0.0)
        self.latest_total_recoveries = 0
        self.latest_status_code = None
        self.latest_status_text = ""
        self.latest_feedback_xy = None
        self.latest_cpu_percent = None
        self.latest_gpu_percent = None
        self.latest_gpu_mem_percent = None
        self.has_nvidia_smi = shutil.which("nvidia-smi") is not None
        if psutil is not None:
            psutil.cpu_percent(interval=None)

        self.goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1, latch=False)
        rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=200)
        rospy.Subscriber(args.scan_topic, LaserScan, self.scan_cb, queue_size=50)
        rospy.Subscriber(args.cmd_topic, Twist, self.cmd_cb, queue_size=200)
        rospy.Subscriber(args.recovery_topic, RecoveryStatus, self.recovery_cb, queue_size=50)
        rospy.Subscriber(args.status_topic, GoalStatusArray, self.status_cb, queue_size=50)
        rospy.Subscriber(args.feedback_topic, MoveBaseActionFeedback, self.feedback_cb, queue_size=50)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.latest_odom = (p.x, p.y, yaw)

    def scan_cb(self, msg):
        front_half_angle = math.radians(float(self.args.dynamic_obstacle_front_angle_deg))
        min_any = float("inf")
        min_front = float("inf")

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            min_any = min(min_any, r)
            angle = norm_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) <= front_half_angle:
                min_front = min(min_front, r)

        self.latest_scan_min = min_any
        self.latest_scan_front_min = min_front

    def cmd_cb(self, msg):
        self.latest_cmd = (msg.linear.x, msg.angular.z)

    def recovery_cb(self, msg):
        self.latest_total_recoveries = int(msg.total_number_of_recoveries)

    def status_cb(self, msg):
        if not msg.status_list:
            return
        st = msg.status_list[-1]
        self.latest_status_code = int(st.status)
        self.latest_status_text = st.text

    def feedback_cb(self, msg):
        p = msg.feedback.base_position.pose.position
        self.latest_feedback_xy = (p.x, p.y)

    def status_name(self):
        if self.latest_status_code is None:
            return "UNKNOWN"
        return STATUS_NAMES.get(self.latest_status_code, str(self.latest_status_code))

    def log(self, out_dir, line):
        print(line)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "events.log"), "a", encoding="utf-8") as f:
            f.write(line + "\n")

    def sample_resources(self):
        cpu_percent = None
        gpu_percent = None
        gpu_mem_percent = None

        if psutil is not None:
            try:
                cpu_percent = float(psutil.cpu_percent(interval=None))
            except Exception:
                cpu_percent = None

        if self.has_nvidia_smi:
            try:
                out = subprocess.check_output(
                    [
                        "nvidia-smi",
                        "--query-gpu=utilization.gpu,memory.used,memory.total",
                        "--format=csv,noheader,nounits",
                    ],
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=0.5,
                )
                gpu_utils = []
                gpu_mems = []
                for line in out.splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    parts = [p.strip() for p in line.split(",")]
                    if len(parts) != 3:
                        continue
                    util = float(parts[0])
                    mem_used = float(parts[1])
                    mem_total = float(parts[2])
                    gpu_utils.append(util)
                    if mem_total > 0.0:
                        gpu_mems.append(100.0 * mem_used / mem_total)
                if gpu_utils:
                    gpu_percent = max(gpu_utils)
                if gpu_mems:
                    gpu_mem_percent = max(gpu_mems)
            except Exception:
                gpu_percent = None
                gpu_mem_percent = None

        self.latest_cpu_percent = cpu_percent
        self.latest_gpu_percent = gpu_percent
        self.latest_gpu_mem_percent = gpu_mem_percent

        return {
            "cpu_percent": cpu_percent,
            "gpu_percent": gpu_percent,
            "gpu_mem_percent": gpu_mem_percent,
        }

    def wait_ready(self, timeout_sec):
        start = time.time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.latest_odom is not None:
                return True
            if time.time() - start > timeout_sec:
                return False
            rate.sleep()
        return False

    def wait_goal_subscriber(self, timeout_sec):
        start = time.time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.goal_pub.get_num_connections() > 0:
                return True
            if time.time() - start > timeout_sec:
                return False
            rate.sleep()
        return False

    def publish_goal(self, frame_id, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def run_goal(self, frame_id, goal):
        out_dir = self.args.out_dir
        name = goal.get("name", "goal")
        gx = float(goal["x"])
        gy = float(goal["y"])
        gyaw = float(goal.get("yaw", 0.0))

        if self.latest_odom is None:
            raise RuntimeError("No odom before goal execution")

        rospy.sleep(0.1)
        goal_msg = self.publish_goal(frame_id, gx, gy, gyaw)
        has_goal_sub = self.wait_goal_subscriber(float(self.args.goal_pub_timeout_sec))
        connections = self.goal_pub.get_num_connections()
        self.log(
            out_dir,
            f"[GOAL-START] {name} -> x={gx:.3f} y={gy:.3f} yaw={gyaw:.3f} "
            f"goal_subscribers={connections} ready={has_goal_sub}",
        )
        for _ in range(3):
            if rospy.is_shutdown():
                break
            goal_msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal_msg)
            rospy.sleep(0.05)
        start_t = time.time()
        last_loop_t = start_t
        last_odom = self.latest_odom
        path_len = 0.0
        min_scan = float("inf")
        lin_samples = []
        ang_samples = []
        recovery_count_start = self.latest_total_recoveries
        max_lin = 0.0
        max_ang = 0.0
        near_collision_count = 0
        in_near_collision = False
        last_progress_log_t = start_t - float(self.args.progress_log_period_sec)
        last_resource_sample_t = start_t - float(self.args.resource_sample_sec)
        timeout = float(self.args.goal_timeout_sec)
        xy_tol = float(self.args.xy_tolerance)
        timeline_rows = []
        resource_rows = []
        dynamic_events = []
        active_dynamic_event = None
        dynamic_event_candidate_t = None
        dynamic_event_clear_candidate_t = None
        last_dynamic_event_end_t = None
        dynamic_goal_margin_m = max(xy_tol * 2.0, float(self.args.dynamic_goal_margin_m))

        def finish_dynamic_event(end_t, bypass_success):
            nonlocal active_dynamic_event, dynamic_event_clear_candidate_t, last_dynamic_event_end_t
            if active_dynamic_event is None:
                return
            active_dynamic_event["end_t"] = end_t
            active_dynamic_event["duration_s"] = max(0.0, end_t - active_dynamic_event["start_t"])
            active_dynamic_event["bypass_success"] = bypass_success
            dynamic_events.append(active_dynamic_event)
            last_dynamic_event_end_t = end_t
            self.log(
                out_dir,
                f"[DYN-EVENT-END] {name} success={bypass_success} "
                f"duration={active_dynamic_event['duration_s']:.2f}s "
                f"clearance={active_dynamic_event['min_clearance_m'] if active_dynamic_event['min_clearance_m'] is not None else '-'} "
                f"response_latency={active_dynamic_event['response_latency_s'] if active_dynamic_event['response_latency_s'] is not None else '-'} "
                f"stop_time={active_dynamic_event['stop_time_s']:.2f}s "
                f"stop_count={active_dynamic_event['stop_count']}"
            )
            active_dynamic_event = None
            dynamic_event_clear_candidate_t = None

        rate = rospy.Rate(20)
        success = False
        while not rospy.is_shutdown():
            now = time.time()
            elapsed = now - start_t
            loop_dt = max(0.0, now - last_loop_t)
            last_loop_t = now
            dist_to_goal = None

            if self.latest_odom is not None:
                cx, cy, _ = self.latest_odom
                path_len += dist2d(last_odom[0], last_odom[1], cx, cy)
                last_odom = (cx, cy, self.latest_odom[2])
                dist_to_goal = dist2d(cx, cy, gx, gy)
                if dist_to_goal <= xy_tol:
                    success = True
                    break

            if math.isfinite(self.latest_scan_min):
                min_scan = min(min_scan, self.latest_scan_min)
                is_near = self.latest_scan_min <= float(self.args.near_collision_threshold_m)
                if is_near and not in_near_collision:
                    near_collision_count += 1
                in_near_collision = is_near

            front_scan = self.latest_scan_front_min if math.isfinite(self.latest_scan_front_min) else None

            lin_samples.append(self.latest_cmd[0])
            ang_samples.append(self.latest_cmd[1])
            max_lin = max(max_lin, abs(self.latest_cmd[0]))
            max_ang = max(max_ang, abs(self.latest_cmd[1]))

            can_start_dynamic_event = (
                front_scan is not None
                and front_scan <= float(self.args.dynamic_obstacle_threshold_m)
                and (dist_to_goal is None or dist_to_goal > dynamic_goal_margin_m)
                and (
                    last_dynamic_event_end_t is None
                    or (now - last_dynamic_event_end_t) >= float(self.args.dynamic_event_min_gap_sec)
                )
            )

            if active_dynamic_event is None:
                if can_start_dynamic_event:
                    if dynamic_event_candidate_t is None:
                        dynamic_event_candidate_t = now
                    elif (now - dynamic_event_candidate_t) >= float(self.args.dynamic_obstacle_persist_sec):
                        active_dynamic_event = {
                            "start_t": now,
                            "min_clearance_m": front_scan,
                            "response_latency_s": None,
                            "stop_time_s": 0.0,
                            "stop_count": 0,
                            "in_stop": False,
                        }
                        dynamic_event_candidate_t = None
                        dynamic_event_clear_candidate_t = None
                        self.log(
                            out_dir,
                            f"[DYN-EVENT-START] {name} t={elapsed:.2f}s front_scan={front_scan:.3f}m",
                        )
                else:
                    dynamic_event_candidate_t = None
            else:
                if front_scan is not None:
                    active_dynamic_event["min_clearance_m"] = min(
                        active_dynamic_event["min_clearance_m"],
                        front_scan,
                    )

                if active_dynamic_event["response_latency_s"] is None:
                    if (
                        abs(self.latest_cmd[0]) <= float(self.args.dynamic_response_lin_threshold_mps)
                        or abs(self.latest_cmd[1]) >= float(self.args.dynamic_response_ang_threshold_rps)
                    ):
                        active_dynamic_event["response_latency_s"] = now - active_dynamic_event["start_t"]

                is_stopped = abs(self.latest_cmd[0]) <= float(self.args.dynamic_stop_lin_threshold_mps)
                if is_stopped:
                    if not active_dynamic_event["in_stop"]:
                        active_dynamic_event["in_stop"] = True
                        active_dynamic_event["stop_count"] += 1
                    active_dynamic_event["stop_time_s"] += loop_dt
                else:
                    active_dynamic_event["in_stop"] = False

                if front_scan is not None and front_scan >= float(self.args.dynamic_obstacle_clear_threshold_m):
                    if dynamic_event_clear_candidate_t is None:
                        dynamic_event_clear_candidate_t = now
                    elif (now - dynamic_event_clear_candidate_t) >= float(self.args.dynamic_obstacle_clear_persist_sec):
                        finish_dynamic_event(now, bypass_success=True)
                else:
                    dynamic_event_clear_candidate_t = None

            if now - last_resource_sample_t >= float(self.args.resource_sample_sec):
                last_resource_sample_t = now
                resource_rows.append(self.sample_resources())

            timeline_rows.append({
                "t": elapsed,
                "x": self.latest_odom[0] if self.latest_odom is not None else None,
                "y": self.latest_odom[1] if self.latest_odom is not None else None,
                "yaw": self.latest_odom[2] if self.latest_odom is not None else None,
                "dist_to_goal_m": dist_to_goal,
                "scan_min_m": self.latest_scan_min if math.isfinite(self.latest_scan_min) else None,
                "front_scan_min_m": front_scan,
                "cmd_lin": self.latest_cmd[0],
                "cmd_ang": self.latest_cmd[1],
                "recovery_total": self.latest_total_recoveries,
                "status_code": self.latest_status_code,
                "status_name": self.status_name(),
                "status_text": self.latest_status_text,
                "cpu_percent": self.latest_cpu_percent,
                "gpu_percent": self.latest_gpu_percent,
                "gpu_mem_percent": self.latest_gpu_mem_percent,
                "dynamic_event_active": active_dynamic_event is not None,
            })

            if now - last_progress_log_t >= float(self.args.progress_log_period_sec):
                last_progress_log_t = now
                dist_str = f"{dist_to_goal:.3f}" if dist_to_goal is not None else "-"
                scan_str = f"{self.latest_scan_min:.3f}" if math.isfinite(self.latest_scan_min) else "-"
                front_scan_str = f"{front_scan:.3f}" if front_scan is not None else "-"
                cpu_str = f"{self.latest_cpu_percent:.1f}" if self.latest_cpu_percent is not None else "-"
                gpu_str = f"{self.latest_gpu_percent:.1f}" if self.latest_gpu_percent is not None else "-"
                self.log(
                    out_dir,
                    f"[PROGRESS] {name} t={elapsed:.1f}s dist={dist_str}m scan_min={scan_str}m front_scan={front_scan_str}m "
                    f"cmd=({self.latest_cmd[0]:.3f},{self.latest_cmd[1]:.3f}) "
                    f"status={self.status_name()} recoveries={self.latest_total_recoveries} "
                    f"cpu={cpu_str}% gpu={gpu_str}%",
                )

            if elapsed > timeout:
                break
            rate.sleep()

        duration = time.time() - start_t
        if active_dynamic_event is not None:
            finish_dynamic_event(time.time(), bypass_success=False)
        if not math.isfinite(min_scan):
            min_scan = None

        lin_std = statistics.pstdev(lin_samples) if len(lin_samples) > 1 else 0.0
        ang_std = statistics.pstdev(ang_samples) if len(ang_samples) > 1 else 0.0
        avg_speed = (path_len / duration) if duration > 0.0 else 0.0
        recovery_count = max(0, self.latest_total_recoveries - recovery_count_start)

        def metric_vals(key):
            return [r[key] for r in resource_rows if r.get(key) is not None]

        cpu_vals = metric_vals("cpu_percent")
        gpu_vals = metric_vals("gpu_percent")
        gpu_mem_vals = metric_vals("gpu_mem_percent")

        goal_pos_error = None
        goal_yaw_error = None
        if self.latest_odom is not None:
            goal_pos_error = dist2d(self.latest_odom[0], self.latest_odom[1], gx, gy)
            goal_yaw_error = abs(angle_diff(self.latest_odom[2], gyaw))

        successful_dynamic_events = [e for e in dynamic_events if e.get("bypass_success")]
        response_latencies = [e["response_latency_s"] for e in dynamic_events if e.get("response_latency_s") is not None]
        bypass_times = [e["duration_s"] for e in successful_dynamic_events]
        min_dynamic_clearance = min([e["min_clearance_m"] for e in dynamic_events], default=None)
        total_stop_time = sum(e["stop_time_s"] for e in dynamic_events)
        stop_count = sum(e["stop_count"] for e in dynamic_events)
        bypass_success_count = len(successful_dynamic_events)
        human_event_count = len(dynamic_events)

        timeline_dir = os.path.join(out_dir, "timelines")
        os.makedirs(timeline_dir, exist_ok=True)
        timeline_csv = os.path.join(timeline_dir, f"{name}.csv")
        with open(timeline_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(
                f,
                fieldnames=[
                    "t", "x", "y", "yaw", "dist_to_goal_m", "scan_min_m",
                    "front_scan_min_m",
                    "cmd_lin", "cmd_ang", "recovery_total",
                    "status_code", "status_name", "status_text",
                    "cpu_percent", "gpu_percent", "gpu_mem_percent",
                    "dynamic_event_active",
                ],
            )
            w.writeheader()
            for r in timeline_rows:
                w.writerow(r)

        self.log(
            out_dir,
            f"[GOAL-END] {name} success={success} duration={duration:.2f}s path={path_len:.3f}m "
            f"avg_speed={avg_speed:.3f}m/s min_scan={min_scan if min_scan is not None else '-'} "
            f"goal_err={goal_pos_error if goal_pos_error is not None else '-'} "
            f"yaw_err={goal_yaw_error if goal_yaw_error is not None else '-'} "
            f"recoveries={recovery_count} near_collision={near_collision_count} "
            f"human_events={human_event_count} bypass_success={bypass_success_count} "
            f"stop_time={total_stop_time:.2f}s "
            f"cpu_avg={statistics.mean(cpu_vals) if cpu_vals else '-'} "
            f"gpu_avg={statistics.mean(gpu_vals) if gpu_vals else '-'}",
        )

        return {
            "name": name,
            "goal_x": gx,
            "goal_y": gy,
            "goal_yaw": gyaw,
            "success": success,
            "duration_s": duration,
            "path_length_m": path_len,
            "avg_speed_mps": avg_speed,
            "min_scan_dist_m": min_scan,
            "goal_pos_error_m": goal_pos_error,
            "goal_yaw_error_rad": goal_yaw_error,
            "recovery_count": recovery_count,
            "near_collision_count": near_collision_count,
            "human_event_count": human_event_count,
            "bypass_success_count": bypass_success_count,
            "bypass_success_rate": (bypass_success_count / human_event_count) if human_event_count > 0 else None,
            "avg_bypass_time_s": statistics.mean(bypass_times) if bypass_times else None,
            "max_bypass_time_s": max(bypass_times) if bypass_times else None,
            "avg_replan_latency_s": statistics.mean(response_latencies) if response_latencies else None,
            "min_dynamic_clearance_m": min_dynamic_clearance,
            "total_stop_time_s": total_stop_time,
            "stop_count": stop_count,
            "max_cmd_lin": max_lin,
            "max_cmd_ang": max_ang,
            "avg_cpu_percent": statistics.mean(cpu_vals) if cpu_vals else None,
            "max_cpu_percent": max(cpu_vals) if cpu_vals else None,
            "avg_gpu_percent": statistics.mean(gpu_vals) if gpu_vals else None,
            "max_gpu_percent": max(gpu_vals) if gpu_vals else None,
            "avg_gpu_mem_percent": statistics.mean(gpu_mem_vals) if gpu_mem_vals else None,
            "max_gpu_mem_percent": max(gpu_mem_vals) if gpu_mem_vals else None,
            "cmd_lin_std": lin_std,
            "cmd_ang_std": ang_std,
            "_cpu_sum": sum(cpu_vals) if cpu_vals else 0.0,
            "_cpu_count": len(cpu_vals),
            "_gpu_sum": sum(gpu_vals) if gpu_vals else 0.0,
            "_gpu_count": len(gpu_vals),
            "_gpu_mem_sum": sum(gpu_mem_vals) if gpu_mem_vals else 0.0,
            "_gpu_mem_count": len(gpu_mem_vals),
            "_bypass_time_sum": sum(bypass_times) if bypass_times else 0.0,
            "_bypass_time_count": len(bypass_times),
            "_replan_latency_sum": sum(response_latencies) if response_latencies else 0.0,
            "_replan_latency_count": len(response_latencies),
        }


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "goals" not in data:
        raise ValueError("waypoints file must contain top-level keys: frame_id, goals")
    frame_id = data.get("frame_id", "map")
    goals = data["goals"]
    if not isinstance(goals, list) or not goals:
        raise ValueError("goals must be a non-empty list")
    for i, g in enumerate(goals):
        if "x" not in g or "y" not in g:
            raise ValueError(f"goal[{i}] missing x/y")
    return frame_id, goals


def save_results(out_dir, algorithm, rows, start_pose=None, end_pose=None):
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "per_goal.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "name",
                "goal_x",
                "goal_y",
                "goal_yaw",
                "success",
                "duration_s",
                "path_length_m",
                "avg_speed_mps",
                "min_scan_dist_m",
                "goal_pos_error_m",
                "goal_yaw_error_rad",
                "recovery_count",
                "near_collision_count",
                "human_event_count",
                "bypass_success_count",
                "bypass_success_rate",
                "avg_bypass_time_s",
                "max_bypass_time_s",
                "avg_replan_latency_s",
                "min_dynamic_clearance_m",
                "total_stop_time_s",
                "stop_count",
                "max_cmd_lin",
                "max_cmd_ang",
                "avg_cpu_percent",
                "max_cpu_percent",
                "avg_gpu_percent",
                "max_gpu_percent",
                "avg_gpu_mem_percent",
                "max_gpu_mem_percent",
                "cmd_lin_std",
                "cmd_ang_std",
            ],
            extrasaction="ignore",
        )
        w.writeheader()
        for r in rows:
            w.writerow(r)

    success_rows = [r for r in rows if r["success"]]
    last_goal_error = None
    last_goal_yaw_error = None
    if rows and end_pose is not None:
        last_goal_error = dist2d(end_pose[0], end_pose[1], rows[-1]["goal_x"], rows[-1]["goal_y"])
        last_goal_yaw_error = abs(angle_diff(end_pose[2], rows[-1]["goal_yaw"]))

    end_to_start_error = None
    end_to_start_yaw_error = None
    if start_pose is not None and end_pose is not None:
        end_to_start_error = dist2d(start_pose[0], start_pose[1], end_pose[0], end_pose[1])
        end_to_start_yaw_error = abs(angle_diff(end_pose[2], start_pose[2]))

    summary = {
        "algorithm": algorithm,
        "goal_count": len(rows),
        "success_count": len(success_rows),
        "success_rate": (len(success_rows) / len(rows)) if rows else 0.0,
        "total_time_s": sum(r["duration_s"] for r in rows) if rows else None,
        "total_path_m": sum(r["path_length_m"] for r in rows) if rows else None,
        "avg_time_success_s": statistics.mean([r["duration_s"] for r in success_rows]) if success_rows else None,
        "avg_path_success_m": statistics.mean([r["path_length_m"] for r in success_rows]) if success_rows else None,
        "avg_speed_all_mps": (sum(r["path_length_m"] for r in rows) / sum(r["duration_s"] for r in rows)) if rows and sum(r["duration_s"] for r in rows) > 0 else None,
        "avg_speed_success_mps": statistics.mean([r["avg_speed_mps"] for r in success_rows]) if success_rows else None,
        "min_scan_dist_all_m": min([r["min_scan_dist_m"] for r in rows if r["min_scan_dist_m"] is not None], default=None),
        "total_recovery_count": sum(r["recovery_count"] for r in rows) if rows else None,
        "total_near_collision_count": sum(r["near_collision_count"] for r in rows) if rows else None,
        "total_human_event_count": sum(r["human_event_count"] for r in rows) if rows else None,
        "total_bypass_success_count": sum(r["bypass_success_count"] for r in rows) if rows else None,
        "bypass_success_rate": (
            sum(r["bypass_success_count"] for r in rows) / sum(r["human_event_count"] for r in rows)
            if rows and sum(r["human_event_count"] for r in rows) > 0 else None
        ),
        "avg_bypass_time_s": (
            sum(r.get("_bypass_time_sum", 0.0) for r in rows) / sum(r.get("_bypass_time_count", 0) for r in rows)
            if sum(r.get("_bypass_time_count", 0) for r in rows) > 0 else None
        ),
        "max_bypass_time_s": max([r["max_bypass_time_s"] for r in rows if r.get("max_bypass_time_s") is not None], default=None),
        "avg_replan_latency_s": (
            sum(r.get("_replan_latency_sum", 0.0) for r in rows) / sum(r.get("_replan_latency_count", 0) for r in rows)
            if sum(r.get("_replan_latency_count", 0) for r in rows) > 0 else None
        ),
        "min_dynamic_clearance_m": min([r["min_dynamic_clearance_m"] for r in rows if r["min_dynamic_clearance_m"] is not None], default=None),
        "total_stop_time_s": sum(r["total_stop_time_s"] for r in rows) if rows else None,
        "total_stop_count": sum(r["stop_count"] for r in rows) if rows else None,
        "avg_goal_pos_error_m": statistics.mean([r["goal_pos_error_m"] for r in rows if r["goal_pos_error_m"] is not None]) if rows else None,
        "avg_goal_yaw_error_rad": statistics.mean([r["goal_yaw_error_rad"] for r in success_rows if r["goal_yaw_error_rad"] is not None]) if success_rows else None,
        "max_cmd_lin_mps": max([r["max_cmd_lin"] for r in rows], default=None),
        "max_cmd_ang_rps": max([r["max_cmd_ang"] for r in rows], default=None),
        "avg_cpu_percent": (
            sum(r.get("_cpu_sum", 0.0) for r in rows) / sum(r.get("_cpu_count", 0) for r in rows)
            if sum(r.get("_cpu_count", 0) for r in rows) > 0 else None
        ),
        "max_cpu_percent": max([r["max_cpu_percent"] for r in rows if r.get("max_cpu_percent") is not None], default=None),
        "avg_gpu_percent": (
            sum(r.get("_gpu_sum", 0.0) for r in rows) / sum(r.get("_gpu_count", 0) for r in rows)
            if sum(r.get("_gpu_count", 0) for r in rows) > 0 else None
        ),
        "max_gpu_percent": max([r["max_gpu_percent"] for r in rows if r.get("max_gpu_percent") is not None], default=None),
        "avg_gpu_mem_percent": (
            sum(r.get("_gpu_mem_sum", 0.0) for r in rows) / sum(r.get("_gpu_mem_count", 0) for r in rows)
            if sum(r.get("_gpu_mem_count", 0) for r in rows) > 0 else None
        ),
        "max_gpu_mem_percent": max([r["max_gpu_mem_percent"] for r in rows if r.get("max_gpu_mem_percent") is not None], default=None),
        "avg_cmd_lin_std": statistics.mean([r["cmd_lin_std"] for r in rows]) if rows else None,
        "avg_cmd_ang_std": statistics.mean([r["cmd_ang_std"] for r in rows]) if rows else None,
        "start_x": start_pose[0] if start_pose is not None else None,
        "start_y": start_pose[1] if start_pose is not None else None,
        "start_yaw": start_pose[2] if start_pose is not None else None,
        "end_x": end_pose[0] if end_pose is not None else None,
        "end_y": end_pose[1] if end_pose is not None else None,
        "end_yaw": end_pose[2] if end_pose is not None else None,
        "last_goal_error_m": last_goal_error,
        "last_goal_yaw_error_rad": last_goal_yaw_error,
        "end_to_start_error_m": end_to_start_error,
        "end_to_start_yaw_error_rad": end_to_start_yaw_error,
    }

    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")


def apply_preset(args, parser):
    if args.preset != "human_dynamic":
        return

    # Only override values that are still at parser defaults so users can still
    # customize a subset of parameters when needed.
    preset_values = {
        "goal_timeout_sec": 150.0,
        "near_collision_threshold_m": 0.20,
        "progress_log_period_sec": 0.5,
        "resource_sample_sec": 0.5,
        "dynamic_obstacle_threshold_m": 1.0,
        "dynamic_obstacle_clear_threshold_m": 1.3,
        "dynamic_obstacle_persist_sec": 0.4,
        "dynamic_obstacle_clear_persist_sec": 0.6,
        "dynamic_obstacle_front_angle_deg": 30.0,
        "dynamic_event_min_gap_sec": 1.0,
        "dynamic_response_lin_threshold_mps": 0.12,
        "dynamic_response_ang_threshold_rps": 0.25,
        "dynamic_stop_lin_threshold_mps": 0.03,
    }
    for key, value in preset_values.items():
        if getattr(args, key) == parser.get_default(key):
            setattr(args, key, value)


def main():
    parser = argparse.ArgumentParser(description="Waypoint-based navigation evaluation")
    parser.add_argument("--waypoints", required=True, help="YAML with frame_id and goals")
    parser.add_argument("--algorithm", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--preset", choices=["default", "human_dynamic"], default="default")
    parser.add_argument("--goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--odom-topic", default="/slam/odom")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--recovery-topic", default="/move_base/recovery_status")
    parser.add_argument("--status-topic", default="/move_base/status")
    parser.add_argument("--feedback-topic", default="/move_base/feedback")
    parser.add_argument("--xy-tolerance", type=float, default=0.25)
    parser.add_argument("--goal-timeout-sec", type=float, default=90.0)
    parser.add_argument("--goal-pub-timeout-sec", type=float, default=3.0)
    parser.add_argument("--settle-sec", type=float, default=0.0)
    parser.add_argument("--ready-timeout-sec", type=float, default=30.0)
    parser.add_argument("--near-collision-threshold-m", type=float, default=0.25)
    parser.add_argument("--progress-log-period-sec", type=float, default=1.0)
    parser.add_argument("--resource-sample-sec", type=float, default=1.0)
    parser.add_argument("--dynamic-obstacle-threshold-m", type=float, default=0.8)
    parser.add_argument("--dynamic-obstacle-clear-threshold-m", type=float, default=1.0)
    parser.add_argument("--dynamic-obstacle-persist-sec", type=float, default=0.4)
    parser.add_argument("--dynamic-obstacle-clear-persist-sec", type=float, default=0.6)
    parser.add_argument("--dynamic-obstacle-front-angle-deg", type=float, default=35.0)
    parser.add_argument("--dynamic-event-min-gap-sec", type=float, default=1.0)
    parser.add_argument("--dynamic-goal-margin-m", type=float, default=0.5)
    parser.add_argument("--dynamic-response-lin-threshold-mps", type=float, default=0.10)
    parser.add_argument("--dynamic-response-ang-threshold-rps", type=float, default=0.20)
    parser.add_argument("--dynamic-stop-lin-threshold-mps", type=float, default=0.03)
    parser.add_argument("--append-start-goal", action="store_true", help="Append the robot start pose as the final evaluation goal")
    parser.add_argument("--append-start-goal-name", default="return_to_start")
    args = parser.parse_args()
    apply_preset(args, parser)
    os.makedirs(args.out_dir, exist_ok=True)

    frame_id, goals = load_waypoints(args.waypoints)
    rospy.init_node("nav_waypoint_eval", anonymous=True)
    node = NavEvalNode(args)

    if not node.wait_ready(args.ready_timeout_sec):
        raise RuntimeError("Timed out waiting for odom")

    rows = []
    start_pose = node.latest_odom
    if args.append_start_goal and start_pose is not None:
        goals = list(goals) + [{
            "name": args.append_start_goal_name,
            "x": float(start_pose[0]),
            "y": float(start_pose[1]),
            "yaw": float(start_pose[2]),
        }]
        print(
            f"[INFO] Appended final goal '{args.append_start_goal_name}' "
            f"at start pose x={start_pose[0]:.3f} y={start_pose[1]:.3f} yaw={start_pose[2]:.3f}"
        )
    for g in goals:
        rows.append(node.run_goal(frame_id, g))
    end_pose = node.latest_odom

    save_results(args.out_dir, args.algorithm, rows, start_pose=start_pose, end_pose=end_pose)
    print(f"[DONE] Saved: {args.out_dir}/per_goal.csv and summary.json")


if __name__ == "__main__":
    main()
