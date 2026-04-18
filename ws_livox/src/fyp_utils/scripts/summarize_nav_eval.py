#!/usr/bin/env python3
import argparse
import json
import os


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, (int, float)):
        return f"{v:.6f}"
    return str(v)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--root", required=True)
    p.add_argument("--algs", required=True)
    args = p.parse_args()

    algs = [a.strip() for a in args.algs.split(",") if a.strip()]
    rows = []
    for alg in algs:
        s = os.path.join(args.root, alg, "summary.json")
        if not os.path.isfile(s):
            rows.append({"algorithm": alg})
            continue
        with open(s, "r", encoding="utf-8") as f:
            d = json.load(f)
        d["algorithm"] = alg
        rows.append(d)

    md = os.path.join(args.root, "summary.md")
    with open(md, "w", encoding="utf-8") as f:
        f.write("# Navigation Eval Summary\n\n")
        f.write(f"Result root: `{args.root}`\n\n")
        f.write("| algorithm | goal_count | success_count | success_rate | total_time_s | total_path_m | avg_speed_all_mps | avg_time_success_s | avg_path_success_m | avg_speed_success_mps | min_scan_dist_all_m | total_recovery_count | total_near_collision_count | total_human_event_count | total_bypass_success_count | bypass_success_rate | avg_bypass_time_s | max_bypass_time_s | avg_replan_latency_s | min_dynamic_clearance_m | total_stop_time_s | total_stop_count | avg_goal_pos_error_m | avg_goal_yaw_error_rad | start_yaw | end_yaw | end_to_start_error_m | end_to_start_yaw_error_rad | last_goal_error_m | last_goal_yaw_error_rad | avg_cpu_percent | max_cpu_percent | avg_gpu_percent | max_gpu_percent | avg_gpu_mem_percent | max_gpu_mem_percent | avg_cmd_lin_std | avg_cmd_ang_std |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for r in rows:
            f.write(
                f"| {r.get('algorithm','-')} | {fmt(r.get('goal_count'))} | {fmt(r.get('success_count'))} | {fmt(r.get('success_rate'))} | "
                f"{fmt(r.get('total_time_s'))} | {fmt(r.get('total_path_m'))} | {fmt(r.get('avg_speed_all_mps'))} | "
                f"{fmt(r.get('avg_time_success_s'))} | {fmt(r.get('avg_path_success_m'))} | {fmt(r.get('avg_speed_success_mps'))} | "
                f"{fmt(r.get('min_scan_dist_all_m'))} | {fmt(r.get('total_recovery_count'))} | {fmt(r.get('total_near_collision_count'))} | "
                f"{fmt(r.get('total_human_event_count'))} | {fmt(r.get('total_bypass_success_count'))} | {fmt(r.get('bypass_success_rate'))} | "
                f"{fmt(r.get('avg_bypass_time_s'))} | {fmt(r.get('max_bypass_time_s'))} | {fmt(r.get('avg_replan_latency_s'))} | "
                f"{fmt(r.get('min_dynamic_clearance_m'))} | {fmt(r.get('total_stop_time_s'))} | {fmt(r.get('total_stop_count'))} | "
                f"{fmt(r.get('avg_goal_pos_error_m'))} | {fmt(r.get('avg_goal_yaw_error_rad'))} | "
                f"{fmt(r.get('start_yaw'))} | {fmt(r.get('end_yaw'))} | "
                f"{fmt(r.get('end_to_start_error_m'))} | {fmt(r.get('end_to_start_yaw_error_rad'))} | "
                f"{fmt(r.get('last_goal_error_m'))} | {fmt(r.get('last_goal_yaw_error_rad'))} | "
                f"{fmt(r.get('avg_cpu_percent'))} | {fmt(r.get('max_cpu_percent'))} | "
                f"{fmt(r.get('avg_gpu_percent'))} | {fmt(r.get('max_gpu_percent'))} | "
                f"{fmt(r.get('avg_gpu_mem_percent'))} | {fmt(r.get('max_gpu_mem_percent'))} | "
                f"{fmt(r.get('avg_cmd_lin_std'))} | {fmt(r.get('avg_cmd_ang_std'))} |\n"
            )

    print(f"[DONE] Summary: {md}")


if __name__ == "__main__":
    main()
