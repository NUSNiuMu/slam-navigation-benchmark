#!/usr/bin/env python3
import argparse
import csv
import json
import os
import re
import signal
import subprocess
import threading
import time
from statistics import mean

import rospy
import rostopic


def read_total_cpu_jiffies():
    with open('/proc/stat', 'r', encoding='utf-8') as f:
        line = f.readline().strip()
    parts = line.split()
    if not parts or parts[0] != 'cpu':
        return 0
    return sum(int(x) for x in parts[1:])


def read_proc_cpu_jiffies(pid):
    stat_path = f'/proc/{pid}/stat'
    with open(stat_path, 'r', encoding='utf-8') as f:
        content = f.read().strip()
    fields = content.split()
    # utime=14th, stime=15th (1-based)
    utime = int(fields[13])
    stime = int(fields[14])
    return utime + stime


def read_proc_rss_mb(pid):
    status_path = f'/proc/{pid}/status'
    with open(status_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith('VmRSS:'):
                parts = line.split()
                # kB -> MB
                return float(parts[1]) / 1024.0
    return 0.0


def get_node_pid(node_name):
    try:
        out = subprocess.check_output(
            ['rosnode', 'info', node_name],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except subprocess.CalledProcessError:
        return None

    m = re.search(r'Pid:\s*(\d+)', out)
    if not m:
        return None
    return int(m.group(1))


def topic_stamp_to_sec(msg):
    if not hasattr(msg, 'header'):
        return None
    hdr = getattr(msg, 'header')
    if hdr is None or not hasattr(hdr, 'stamp'):
        return None
    try:
        return hdr.stamp.to_sec()
    except Exception:
        return None


class Monitor:
    def __init__(self, args):
        self.args = args
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.latest_input_stamp = None
        self.prev_odom_arrival = None
        self.odom_count = 0

        self.frame_intervals = []
        self.sim_latencies = []
        self.cpu_samples = []
        self.mem_samples = []

        self.node_to_pid = {}
        self.prev_pid_cpu = {}
        self.prev_total_cpu = None
        self.ncpu = os.cpu_count() or 1

        os.makedirs(self.args.out_dir, exist_ok=True)
        self.frame_csv_path = os.path.join(self.args.out_dir, 'frame_times.csv')
        self.resource_csv_path = os.path.join(self.args.out_dir, 'resource_usage.csv')
        self.summary_json_path = os.path.join(self.args.out_dir, 'summary.json')

        self.frame_csv = open(self.frame_csv_path, 'w', newline='', encoding='utf-8')
        self.frame_writer = csv.writer(self.frame_csv)
        self.frame_writer.writerow([
            'wall_time',
            'odom_stamp',
            'frame_interval_s',
            'sim_latency_s',
        ])

        self.resource_csv = open(self.resource_csv_path, 'w', newline='', encoding='utf-8')
        self.resource_writer = csv.writer(self.resource_csv)
        self.resource_writer.writerow([
            'wall_time',
            'cpu_pct_total',
            'rss_mb_total',
            'pid_count',
            'pids',
        ])

    def input_cb(self, msg):
        stamp = topic_stamp_to_sec(msg)
        if stamp is None:
            return
        with self.lock:
            self.latest_input_stamp = stamp

    def odom_cb(self, msg):
        now = time.time()
        odom_stamp = topic_stamp_to_sec(msg)

        with self.lock:
            frame_interval = None
            if self.prev_odom_arrival is not None:
                frame_interval = now - self.prev_odom_arrival
                self.frame_intervals.append(frame_interval)

            sim_latency = None
            if odom_stamp is not None and self.latest_input_stamp is not None:
                sim_latency = odom_stamp - self.latest_input_stamp
                if sim_latency >= 0.0:
                    self.sim_latencies.append(sim_latency)

            self.prev_odom_arrival = now
            self.odom_count += 1

        self.frame_writer.writerow([
            f'{now:.6f}',
            '' if odom_stamp is None else f'{odom_stamp:.6f}',
            '' if frame_interval is None else f'{frame_interval:.6f}',
            '' if sim_latency is None else f'{sim_latency:.6f}',
        ])

    def list_candidate_nodes(self):
        try:
            nodes = rospy.get_published_topics()
            _ = nodes  # keep master connectivity check lightweight
            node_names = rospy.get_param('/roslaunch/uris', default={})
            _ = node_names
        except Exception:
            pass

        try:
            names = subprocess.check_output(['rosnode', 'list'], text=True).splitlines()
        except Exception:
            return []

        exclude = set(x.strip() for x in self.args.exclude_nodes.split(',') if x.strip())
        result = []
        for n in names:
            n = n.strip()
            if not n:
                continue
            if n in exclude:
                continue
            if 'rosbag' in n and self.args.exclude_rosbag:
                continue
            if 'slam_eval_monitor' in n:
                continue
            result.append(n)
        return result

    def refresh_pids(self):
        for node in self.list_candidate_nodes():
            if node in self.node_to_pid:
                continue
            pid = get_node_pid(node)
            if pid is not None:
                self.node_to_pid[node] = pid

        # drop dead pids
        alive = {}
        for node, pid in self.node_to_pid.items():
            if os.path.exists(f'/proc/{pid}'):
                alive[node] = pid
        self.node_to_pid = alive

    def sample_resources_once(self):
        self.refresh_pids()
        pids = sorted(set(self.node_to_pid.values()))

        total_cpu_now = read_total_cpu_jiffies()
        cpu_pct_sum = 0.0
        mem_sum = 0.0

        for pid in pids:
            try:
                proc_cpu_now = read_proc_cpu_jiffies(pid)
                mem_sum += read_proc_rss_mb(pid)
            except Exception:
                continue

            proc_cpu_prev = self.prev_pid_cpu.get(pid)
            if (
                proc_cpu_prev is not None
                and self.prev_total_cpu is not None
                and total_cpu_now > self.prev_total_cpu
            ):
                dproc = proc_cpu_now - proc_cpu_prev
                dtotal = total_cpu_now - self.prev_total_cpu
                if dproc >= 0 and dtotal > 0:
                    cpu_pct_sum += (dproc / dtotal) * self.ncpu * 100.0

            self.prev_pid_cpu[pid] = proc_cpu_now

        self.prev_total_cpu = total_cpu_now

        now = time.time()
        self.cpu_samples.append(cpu_pct_sum)
        self.mem_samples.append(mem_sum)

        self.resource_writer.writerow([
            f'{now:.6f}',
            f'{cpu_pct_sum:.4f}',
            f'{mem_sum:.4f}',
            len(pids),
            ' '.join(str(p) for p in pids),
        ])

    def resource_loop(self):
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            self.sample_resources_once()
            time.sleep(self.args.sample_period)

    def close(self):
        self.frame_csv.flush()
        self.resource_csv.flush()
        self.frame_csv.close()
        self.resource_csv.close()

    def write_summary(self):
        def stat_block(values):
            if not values:
                return {
                    'count': 0,
                    'mean': None,
                    'max': None,
                    'min': None,
                }
            return {
                'count': len(values),
                'mean': mean(values),
                'max': max(values),
                'min': min(values),
            }

        summary = {
            'algorithm': self.args.algorithm,
            'odom_topic': self.args.odom_topic,
            'input_topic': self.args.input_topic,
            'odom_msg_count': self.odom_count,
            'cpu_pct_total': stat_block(self.cpu_samples),
            'rss_mb_total': stat_block(self.mem_samples),
            'frame_interval_s': stat_block(self.frame_intervals),
            'sim_latency_s': stat_block(self.sim_latencies),
            'files': {
                'frame_times_csv': self.frame_csv_path,
                'resource_usage_csv': self.resource_csv_path,
            },
        }

        with open(self.summary_json_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)


def wait_topic(topic, timeout_sec=30.0):
    start = time.time()
    while time.time() - start < timeout_sec and not rospy.is_shutdown():
        try:
            cls, _, _ = rostopic.get_topic_class(topic, blocking=False)
        except Exception:
            cls = None
        if cls is not None:
            return cls
        time.sleep(0.5)
    return None


def main():
    parser = argparse.ArgumentParser(description='Monitor SLAM evaluation metrics.')
    parser.add_argument('--algorithm', required=True)
    parser.add_argument('--odom-topic', required=True)
    parser.add_argument('--input-topic', default='')
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--sample-period', type=float, default=0.5)
    parser.add_argument('--exclude-nodes', default='/rosout,/slam_eval_monitor')
    parser.add_argument('--exclude-rosbag', action='store_true', default=False)
    args = parser.parse_args()

    rospy.init_node('slam_eval_monitor', anonymous=False, disable_signals=True)

    monitor = Monitor(args)

    def handle_signal(signum, _frame):
        _ = signum
        monitor.stop_event.set()
        rospy.signal_shutdown('signal')

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    odom_cls = wait_topic(args.odom_topic, timeout_sec=60.0)
    if odom_cls is None:
        raise RuntimeError(f'No message type resolved for odom topic: {args.odom_topic}')

    rospy.Subscriber(args.odom_topic, odom_cls, monitor.odom_cb, queue_size=200)

    if args.input_topic:
        input_cls = wait_topic(args.input_topic, timeout_sec=10.0)
        if input_cls is not None:
            rospy.Subscriber(args.input_topic, input_cls, monitor.input_cb, queue_size=400)

    t = threading.Thread(target=monitor.resource_loop, daemon=True)
    t.start()

    try:
        while not monitor.stop_event.is_set() and not rospy.is_shutdown():
            time.sleep(0.2)
    finally:
        monitor.stop_event.set()
        t.join(timeout=2.0)
        monitor.write_summary()
        monitor.close()


if __name__ == '__main__':
    main()
