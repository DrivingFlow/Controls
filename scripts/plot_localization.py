#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import threading
import argparse
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from collections import deque


class LocalizationPlotter(Node):
    def __init__(self, odom_topic):
        super().__init__('localization_plotter')
        self.get_logger().info(f"Starting LocalizationPlotter: odom_topic={odom_topic}")

        # Latest data
        self.lock = threading.Lock()
        self.latest_pose = {'x': 0.0, 'y': 0.0, 'got': False}

        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)

    def odom_cb(self, msg: Odometry):
        with self.lock:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            self.latest_pose['x'] = px
            self.latest_pose['y'] = py
            self.latest_pose['got'] = True


def start_rclpy_node(node):
    # rclpy.spin must run in its own thread so matplotlib (main thread) can run
    rclpy.spin(node)


def main():
    parser = argparse.ArgumentParser(description='Plot x and y from localization')
    parser.add_argument('--odom', default='/localization', help='Odometry topic (nav_msgs/Odometry)')
    parser.add_argument('--rate', type=float, default=10.0, help='Plot update rate (Hz)')
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationPlotter(args.odom)

    # start rclpy spinner in a background thread
    spin_thread = threading.Thread(target=start_rclpy_node, args=(node,), daemon=True)
    spin_thread.start()

    # prepare matplotlib: single plot for x-y trajectory
    plt.ion()
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    ax.set_title('Localization: X-Y Position')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    # trajectory line
    traj_line, = ax.plot([], [], '-', color='tab:blue', linewidth=2, label='Trajectory')
    # current position marker
    current_point, = ax.plot([], [], 'ro', markersize=10, label='Current Position')

    ax.legend()

    # trajectory data storage
    x_history = deque(maxlen=10000)
    y_history = deque(maxlen=10000)

    start_time = time.time()

    # initial axis limits
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)

    def update(frame):
        with node.lock:
            got_pose = node.latest_pose['got']
            rx = node.latest_pose['x']
            ry = node.latest_pose['y']

        if got_pose:
            # add to history
            x_history.append(rx)
            y_history.append(ry)

            # update trajectory line
            if len(x_history) > 1:
                traj_line.set_data(list(x_history), list(y_history))
            
            # update current position marker
            current_point.set_data([rx], [ry])

            # auto-scale axes to fit trajectory with margin
            if len(x_history) > 0:
                margin = 0.5
                min_x, max_x = min(x_history), max(x_history)
                min_y, max_y = min(y_history), max(y_history)
                
                # add margin
                x_range = max_x - min_x
                y_range = max_y - min_y
                
                if x_range < 0.1:
                    x_range = 10.0
                if y_range < 0.1:
                    y_range = 10.0
                
                ax.set_xlim(min_x - margin, max_x + margin)
                ax.set_ylim(min_y - margin, max_y + margin)

        return traj_line, current_point

    ani = animation.FuncAnimation(fig, update, interval=1000.0/args.rate, blit=False, cache_frame_data=False)
    
    plt.tight_layout()
    
    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass

    # cleanup
    node.get_logger().info('Shutting down LocalizationPlotter')
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
