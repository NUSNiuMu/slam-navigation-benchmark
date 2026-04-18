#!/usr/bin/env python3
import argparse
import csv
import json
import os
import re
from typing import Optional


def parse_rmse(txt_path: str) -> Optional[float]:
    if not os.path.exists(txt_path):
        return None
    with open(txt_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = re.match(r'^\s*rmse\s+([0-9eE+\-.]+)\s*$', line.strip())
            if m:
                try:
                    return float(m.group(1))
                except ValueError:
                    return None
    return None


def get_json(path, key_path, default=None):
    if not os.path.exists(path):
        return default
    with open(path, 'r', encoding='utf-8') as f:
        obj = json.load(f)
    cur = obj
    for k in key_path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def main():
    parser = argparse.ArgumentParser(description='Summarize static rosbag SLAM eval results.')
    parser.add_argument('--root', required=True)
    parser.add_argument('--algs', default='liosam,fastlio,pointlio,fasterlio')
    args = parser.parse_args()

    algs = [x.strip() for x in args.algs.split(',') if x.strip()]
    rows = []

    for alg in algs:
        alg_dir = os.path.join(args.root, alg)
        summary_json = os.path.join(alg_dir, 'metrics', 'summary.json')
        ape_txt = os.path.join(alg_dir, 'evo_ape.txt')
        rpe_txt = os.path.join(alg_dir, 'evo_rpe.txt')

        rows.append({
            'algorithm': alg,
            'ate_rmse_m': parse_rmse(ape_txt),
            'rpe_rmse': parse_rmse(rpe_txt),
            'cpu_mean_pct': get_json(summary_json, ['cpu_pct_total', 'mean']),
            'cpu_max_pct': get_json(summary_json, ['cpu_pct_total', 'max']),
            'mem_mean_mb': get_json(summary_json, ['rss_mb_total', 'mean']),
            'mem_max_mb': get_json(summary_json, ['rss_mb_total', 'max']),
            'frame_mean_s': get_json(summary_json, ['frame_interval_s', 'mean']),
            'frame_max_s': get_json(summary_json, ['frame_interval_s', 'max']),
            'odom_msg_count': get_json(summary_json, ['odom_msg_count']),
        })

    csv_path = os.path.join(args.root, 'summary.csv')
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)

    md_path = os.path.join(args.root, 'summary.md')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('# Static Rosbag SLAM Eval Summary\n\n')
        f.write(f'Result root: `{args.root}`\n\n')
        f.write('| algorithm | ate_rmse_m | rpe_rmse | cpu_mean_pct | mem_mean_mb | frame_mean_s | odom_msg_count |\n')
        f.write('|---|---:|---:|---:|---:|---:|---:|\n')
        for r in rows:
            f.write(
                f"| {r['algorithm']} | {fmt(r['ate_rmse_m'])} | {fmt(r['rpe_rmse'])} | {fmt(r['cpu_mean_pct'])} | {fmt(r['mem_mean_mb'])} | {fmt(r['frame_mean_s'])} | {fmt(r['odom_msg_count'])} |\n"
            )

    print(f'Wrote: {csv_path}')
    print(f'Wrote: {md_path}')


def fmt(v):
    if v is None:
        return 'NA'
    if isinstance(v, int):
        return str(v)
    return f'{v:.6f}'


if __name__ == '__main__':
    main()
