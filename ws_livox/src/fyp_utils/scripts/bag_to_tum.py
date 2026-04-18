#!/usr/bin/env python3
import argparse

import rosbag


def extract_pose(msg):
    # nav_msgs/Odometry
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return p.x, p.y, p.z, q.x, q.y, q.z, q.w

    # geometry_msgs/PoseStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
        p = msg.pose.position
        q = msg.pose.orientation
        return p.x, p.y, p.z, q.x, q.y, q.z, q.w

    # geometry_msgs/PoseWithCovarianceStamped
    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose') and hasattr(msg.pose.pose, 'position'):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return p.x, p.y, p.z, q.x, q.y, q.z, q.w

    return None


def msg_stamp_sec(msg, bag_time):
    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
        try:
            return msg.header.stamp.to_sec()
        except Exception:
            pass
    return bag_time.to_sec()


def main():
    parser = argparse.ArgumentParser(description='Convert ROS bag pose topic to TUM trajectory format.')
    parser.add_argument('--bag', required=True)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    count = 0
    with rosbag.Bag(args.bag, 'r') as bag, open(args.out, 'w', encoding='utf-8') as f:
        for topic, msg, t in bag.read_messages(topics=[args.topic]):
            _ = topic
            pose = extract_pose(msg)
            if pose is None:
                continue
            ts = msg_stamp_sec(msg, t)
            x, y, z, qx, qy, qz, qw = pose
            f.write(f'{ts:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n')
            count += 1

    if count == 0:
        raise RuntimeError(f'No valid pose messages found in topic: {args.topic}')

    print(f'Wrote {count} poses to {args.out}')


if __name__ == '__main__':
    main()
