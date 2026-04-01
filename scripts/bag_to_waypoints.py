#!/usr/bin/env python3
"""Extract waypoints from a rosbag of /pcl_pose, downsample by minimum distance, and save as CSV."""

import argparse
import math
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Convert rosbag /pcl_pose to waypoints CSV")
    parser.add_argument("bag", help="Path to rosbag directory (containing metadata.yaml)")
    parser.add_argument("-o", "--output", default="waypoints_from_bag.csv", help="Output CSV path")
    parser.add_argument("-d", "--min-dist", type=float, default=0.5,
                        help="Minimum distance (meters) between consecutive waypoints")
    parser.add_argument("-t", "--topic", default="/pcl_pose", help="Pose topic name")
    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"Error: {bag_path} does not exist")
        return 1

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    points = []
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic != args.topic:
                continue
            msg = reader.deserialize(rawdata, connection.msgtype)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            points.append((x, y))

    if not points:
        print(f"No messages found on {args.topic}")
        return 1

    print(f"Read {len(points)} raw poses from bag")

    # Downsample: keep points that are at least min_dist apart
    waypoints = [points[0]]
    for x, y in points[1:]:
        px, py = waypoints[-1]
        dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
        if dist >= args.min_dist:
            waypoints.append((x, y))

    # Always include the last point as the final destination
    last = points[-1]
    lx, ly = waypoints[-1]
    if math.sqrt((last[0] - lx) ** 2 + (last[1] - ly) ** 2) > 0.01:
        waypoints.append(last)

    print(f"Downsampled to {len(waypoints)} waypoints (min spacing: {args.min_dist}m)")

    with open(args.output, "w") as f:
        f.write("x,y\n")
        for x, y in waypoints:
            f.write(f"{x:.4f},{y:.4f}\n")

    print(f"Saved to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
