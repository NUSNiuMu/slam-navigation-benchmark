#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure Python analysis of LIO-SAM profiling CSV.
No numpy, no pandas, no matplotlib.
Outputs one interactive HTML report using embedded Plotly.js.
"""

import csv
import os
import argparse
from collections import defaultdict
from statistics import mean, median, pstdev
import math


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    if n == 1:
        return s[0]
    pos = (n - 1) * q
    i = int(math.floor(pos))
    j = int(math.ceil(pos))
    if i == j:
        return s[i]
    frac = pos - i
    return s[i] * (1 - frac) + s[j] * frac


def load_csv(path):
    durations = defaultdict(list)
    timeline = defaultdict(list)

    first_stamp = None

    with open(path, "r") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                stamp = int(row["stamp_ns"])
                mod = row["module"]
                dur = float(row["duration_ms"])
            except:
                continue

            if first_stamp is None:
                first_stamp = stamp
            t = (stamp - first_stamp) / 1e9  # seconds

            durations[mod].append(dur)
            timeline[mod].append((t, dur))

    return durations, timeline


def compute_stats(durations):
    stats = {}
    for m, vals in durations.items():
        if not vals:
            continue
        stats[m] = {
            "count": len(vals),
            "mean": mean(vals),
            "std": pstdev(vals) if len(vals) > 1 else 0.0,
            "min": min(vals),
            "median": median(vals),
            "max": max(vals),
            "sum": sum(vals),
            "p90": percentile(vals, 0.90),
            "p95": percentile(vals, 0.95),
            "p99": percentile(vals, 0.99),
        }

    # sort by mean desc
    ordered = sorted(stats.items(), key=lambda kv: kv[1]["mean"], reverse=True)
    return ordered


def json_escape(s):
    return s.replace("\\", "\\\\").replace("\"", "\\\"")


def build_html(stats, durations, timeline, out_path):
    # Load Plotly via CDN
    plotly_cdn = "https://cdn.plot.ly/plotly-latest.min.js"

    # Prepare JS arrays
    # 1) Bar chart for mean duration
    modules = [m for m, _ in stats[:10]]
    means = [s["mean"] for _, s in stats[:10]]

    # 2) Box plot data
    box_modules = [m for m, _ in stats[:12]]
    box_values = [durations[m] for m in box_modules]

    # 3) Time series (moving average window=5)
    def moving_avg(vals, w=5):
        out = []
        q = []
        acc = 0.0
        for x in vals:
            q.append(x)
            acc += x
            if len(q) > w:
                acc -= q.pop(0)
            out.append(acc / len(q))
        return out

    time_series = []
    for m, _ in stats[:6]:
        t = [x for x, _ in timeline[m]]
        v = [x for _, x in timeline[m]]
        v_ma = moving_avg(v, 5)
        time_series.append((m, t, v_ma))

    # Build HTML
    html = f"""
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<title>LIO-SAM Profile Report</title>
<script src="{plotly_cdn}"></script>
</head>
<body>
<h1>LIO-SAM Profiling Report</h1>

<h2>1. Mean Duration (Top 10 Modules)</h2>
<div id="bar_mean"></div>
<script>
Plotly.newPlot("bar_mean", [{{
    x: {modules},
    y: {means},
    type: "bar"
}}], {{
    title: "Mean duration (ms)"
}});
</script>

<h2>2. Box Plot (Top Modules)</h2>
<div id="boxplot"></div>
<script>
Plotly.newPlot("boxplot", [
"""

    for i, m in enumerate(box_modules):
        vals = box_values[i]
        html += f"""
{{
    y: {vals},
    type: "box",
    name: "{json_escape(m)}",
    boxpoints: false
}},
"""

    html += """
], {
    title: "Distribution of Duration (ms)"
});
</script>

<h2>3. Time Series (Moving Average)</h2>
<div id="timeseries"></div>
<script>
Plotly.newPlot("timeseries", [
"""

    for m, t, v in time_series:
        html += f"""
{{
    x: {t},
    y: {v},
    mode: "lines",
    name: "{json_escape(m)}"
}},
"""

    html += """
], {
    title: "Duration over time (moving avg)"
});
</script>

<h2>4. Summary Table</h2>
<table border="1" cellspacing="0" cellpadding="4">
<tr>
<th>Module</th><th>Count</th><th>Mean</th><th>Std</th><th>Min</th><th>Median</th><th>Max</th><th>p95</th>
</tr>
"""

    for m, s in stats:
        html += f"""
<tr>
<td>{json_escape(m)}</td>
<td>{s['count']}</td>
<td>{s['mean']:.3f}</td>
<td>{s['std']:.3f}</td>
<td>{s['min']:.3f}</td>
<td>{s['median']:.3f}</td>
<td>{s['max']:.3f}</td>
<td>{s['p95']:.3f}</td>
</tr>
"""

    html += """
</table>
</body>
</html>
"""

    with open(out_path, "w") as f:
        f.write(html)

    print(f"[OK] HTML report saved to: {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="Path to liosam_profile.csv")
    args = parser.parse_args()

    durations, timeline = load_csv(args.csv)
    stats = compute_stats(durations)

    out_path = os.path.join(os.path.dirname(args.csv), "profile_report.html")
    build_html(stats, durations, timeline, out_path)


if __name__ == "__main__":
    main()

