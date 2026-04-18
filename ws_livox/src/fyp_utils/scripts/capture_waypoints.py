#!/usr/bin/env python3
import argparse
import math
import os

import rospy
import yaml
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(z, w):
    return 2.0 * math.atan2(z, w)


class WaypointCapture:
    def __init__(self, args):
        self.args = args
        self.goals = []
        rospy.Subscriber(args.topic, PoseStamped, self.cb, queue_size=20)

    def cb(self, msg):
        yaw = quat_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        entry = {
            "name": f"goal_{len(self.goals)+1}",
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "yaw": float(yaw),
        }
        self.goals.append(entry)
        rospy.loginfo("Captured %s: x=%.3f y=%.3f yaw=%.3f", entry["name"], entry["x"], entry["y"], entry["yaw"])

        if self.args.max_goals > 0 and len(self.goals) >= self.args.max_goals:
            self.save(msg.header.frame_id or self.args.frame_id)
            rospy.signal_shutdown("max goals reached")

    def save(self, frame_id):
        out = {
            "frame_id": frame_id if frame_id else self.args.frame_id,
            "goals": self.goals,
        }
        os.makedirs(os.path.dirname(self.args.out) or ".", exist_ok=True)
        with open(self.args.out, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
        rospy.loginfo("Saved waypoints to %s", self.args.out)


def main():
    p = argparse.ArgumentParser(description="Capture RViz 2D Nav Goals into waypoint yaml")
    p.add_argument("--topic", default="/move_base_simple/goal")
    p.add_argument("--out", required=True)
    p.add_argument("--frame-id", default="map")
    p.add_argument("--max-goals", type=int, default=0, help="0 means unlimited; Ctrl+C to save")
    args = p.parse_args()

    rospy.init_node("capture_waypoints", anonymous=True)
    cap = WaypointCapture(args)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cap.save(args.frame_id)


if __name__ == "__main__":
    main()
