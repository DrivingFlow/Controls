    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Float32

    import threading
    import argparse
    import csv
    import math
    import time

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np

    from collections import deque


    class DebugVisualizer(Node):
        def __init__(self, waypoints, odom_topic, cmd_topic):
            super().__init__('debug_waypoints')
            self.get_logger().info(f"Starting DebugVisualizer: odom_topic={odom_topic}, cmd_topic={cmd_topic}")

            self.waypoints = waypoints

            # Latest data
            self.lock = threading.Lock()
            self.latest_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'got': False}
            self.latest_cmd = {'lin_x': 0.0, 'lin_y': 0.0, 'ang_z': 0.0, 'got': False}
            # time at which latest_cmd was received (for estimation)
            self.latest_cmd_time = None

            # derivatives (if published by controller)
            self.latest_deriv_ang = {'val': 0.0, 'got': False}
            self.latest_deriv_lin = {'val': 0.0, 'got': False}

            # estimator state (previous command value/time)
            self._prev_cmd_lin_x = None
            self._prev_cmd_time = None
            # default derivative source; may be overridden by main when creating node
            self.derivative_source = 'publish'

            self.sub_odom = self.create_subscription(
                Odometry, odom_topic, self.odom_cb, 10)
            self.sub_cmd = self.create_subscription(
                TwistStamped, cmd_topic, self.cmd_cb, 10)

            # optionally subscribe to published derivatives; main() will set
            # up the topic names and whether to use them.
            self.sub_deriv_ang = None
            self.sub_deriv_lin = None

        def odom_cb(self, msg: Odometry):
            with self.lock:
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                q = msg.pose.pose.orientation
                # quaternion to yaw
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.latest_pose['x'] = px
                self.latest_pose['y'] = py
                self.latest_pose['yaw'] = yaw
                self.latest_pose['got'] = True

        def cmd_cb(self, msg: TwistStamped):
            with self.lock:
                self.latest_cmd['lin_x'] = msg.twist.linear.x
                self.latest_cmd['lin_y'] = msg.twist.linear.y
                # try angular.z, handle different message shapes if needed
                self.latest_cmd['ang_z'] = getattr(msg.twist.angular, 'z', 0.0)
                self.latest_cmd['got'] = True
                # record arrival time for numerical differentiation
                now = time.time()
                self.latest_cmd_time = now

        def deriv_ang_cb(self, msg: Float32):
            with self.lock:
                self.latest_deriv_ang['val'] = float(msg.data)
                self.latest_deriv_ang['got'] = True
        
        def deriv_lin_cb(self, msg: Float32):
            with self.lock:
                self.latest_deriv_lin['val'] = float(msg.data)
                self.latest_deriv_lin['got'] = True


    def read_waypoints(csv_path):
        waypoints = []
        with open(csv_path, 'r') as fh:
            reader = csv.reader(fh)
            try:
                header = next(reader)
            except StopIteration:
                raise RuntimeError('Waypoints CSV is empty')

            # find x and y columns (case-insensitive)
            x_col = y_col = -1
            for i, col in enumerate(header):
                name = col.strip().lower()
                if name == 'x':
                    x_col = i
                if name == 'y':
                    y_col = i
            if x_col == -1 or y_col == -1:
                # fallback: try common formats sec,nsec,x,y -> choose columns named like 'x' or 'y' anywhere
                # if still not found, raise with helpful message
                raise RuntimeError(f"Could not find 'x' and 'y' columns in CSV header: {header}")

            for row in reader:
                if len(row) <= max(x_col, y_col):
                    continue
                try:
                    x = float(row[x_col].strip())
                    y = float(row[y_col].strip())
                    waypoints.append((x, y))
                except ValueError:
                    # skip malformed rows
                    continue
        return waypoints


    def start_rclpy_node(node):
        # rclpy.spin must run in its own thread so matplotlib (main thread) can run
        rclpy.spin(node)


    def main():
        parser = argparse.ArgumentParser(description='Debug waypoints visualizer')
        parser.add_argument('--csv', '-c', default='/home/unitree/calib_imu/src/mover/src/waypoints.csv', help='Path to waypoints CSV')
        parser.add_argument('--odom', default='/localization', help='Odometry topic (nav_msgs/Odometry)')
        parser.add_argument('--cmd', default='/cmd_vel', help='Motion command topic (geometry_msgs/TwistStamped)')
        parser.add_argument('--derivative-topic-angular', default='/pd_derivative/angular', help='Angular derivative topic (std_msgs/Float32)')
        parser.add_argument('--derivative-topic-linear', default='/pd_derivative/linear', help='Linear derivative topic (std_msgs/Float32)')
        parser.add_argument('--derivative-source', choices=['publish', 'estimate'], default='publish', help='Source for derivative: "publish" to subscribe, "estimate" to numerically differentiate commands')
        parser.add_argument('--rate', type=float, default=10.0, help='Plot update rate (Hz)')
        args = parser.parse_args()

        try:
            waypoints = read_waypoints(args.csv)
        except Exception as e:
            print(f"Failed to read waypoints: {e}")
            return 1

        rclpy.init()
        node = DebugVisualizer(waypoints, args.odom, args.cmd)
        # configure derivative source and subscription
        node.derivative_source = args.derivative_source
        if args.derivative_source == 'publish':
            # subscribe to published derivative values
            node.sub_deriv_ang = node.create_subscription(Float32, args.derivative_topic_angular, node.deriv_ang_cb, 10)
            node.sub_deriv_lin = node.create_subscription(Float32, args.derivative_topic_linear, node.deriv_lin_cb, 10)

        # start rclpy spinner in a background thread
        spin_thread = threading.Thread(target=start_rclpy_node, args=(node,), daemon=True)
        spin_thread.start()

        # prepare matplotlib: two stacked plots (main map + derivative trace)
        plt.ion()
        fig, (ax, ax2) = plt.subplots(2, 1, figsize=(8, 8), gridspec_kw={'height_ratios': [3, 1]})
        ax.set_title('Waypoints + Robot Pose')
        ax.set_xlabel('X (forward)')
        ax.set_ylabel('Y (left)')
        ax.grid(True)
        ax2.set_title('PD Derivative (not multiplied by Kd)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Derivative')
        ax2.grid(True)

        # waypoints
        wp_x = [p[0] for p in waypoints]
        wp_y = [p[1] for p in waypoints]
        line_wp, = ax.plot(wp_x, wp_y, '-o', color='tab:blue', label='waypoints')

        # robot marker and orientation arrow
        robot_point, = ax.plot([], [], 'ro', markersize=10, label='robot')
        # Initialize quiver with dummy data - we'll update it each frame
        quiv = ax.quiver([0], [0], [1], [0], angles='xy', scale_units='xy', scale=1, color='r', width=0.006)

        # text for cmd values
        cmd_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, verticalalignment='top',
                        fontfamily='monospace', fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        ax.legend()

        # derivative plotting state
        deriv_ang_times = deque(maxlen=1000)
        deriv_ang_vals = deque(maxlen=1000)
        deriv_lin_times = deque(maxlen=1000)
        deriv_lin_vals = deque(maxlen=1000)
        deriv_ang_line, = ax2.plot([], [], '-', color='tab:red', label='angular derivative')
        deriv_lin_line, = ax2.plot([], [], '-', color='tab:blue', label='linear derivative')
        ax2.legend()
        start_time = time.time()

        # autoscale to include waypoints and robot
        margin = 1.0
        if waypoints:
            min_x, max_x = min(wp_x), max(wp_x)
            min_y, max_y = min(wp_y), max(wp_y)
            ax.set_xlim(min_x - margin, max_x + margin)
            ax.set_ylim(min_y - margin, max_y + margin)
        else:
            ax.set_xlim(-5, 5)
            ax.set_ylim(-5, 5)

        def update(frame):
            now = time.time()
            t = now - start_time

            with node.lock:
                got_pose = node.latest_pose['got']
                got_cmd = node.latest_cmd['got']
                rx = node.latest_pose['x']
                ry = node.latest_pose['y']
                ryaw = node.latest_pose['yaw']
                lx = node.latest_cmd['lin_x']
                ly = node.latest_cmd['lin_y']
                az = node.latest_cmd['ang_z']

                # determine derivative values
                deriv_ang_value = None
                deriv_lin_value = None
                if node.derivative_source == 'publish':
                    if node.latest_deriv_ang['got']:
                        deriv_ang_value = node.latest_deriv_ang['val']
                    if node.latest_deriv_lin['got']:
                        deriv_lin_value = node.latest_deriv_lin['val']
                else:
                    # estimate derivative from lin_x changes (linear only for estimation mode)
                    if node._prev_cmd_lin_x is None and node.latest_cmd_time is not None:
                        # initialize previous values
                        node._prev_cmd_lin_x = lx
                        node._prev_cmd_time = node.latest_cmd_time
                        deriv_lin_value = 0.0
                    elif node._prev_cmd_lin_x is not None and node.latest_cmd_time is not None:
                        dt = node.latest_cmd_time - node._prev_cmd_time if node._prev_cmd_time is not None else None
                        if dt and dt > 1e-6:
                            deriv_lin_value = (lx - node._prev_cmd_lin_x) / dt
                        else:
                            deriv_lin_value = 0.0
                        # update prev
                        node._prev_cmd_lin_x = lx
                        node._prev_cmd_time = node.latest_cmd_time

            if got_pose:
                robot_point.set_data([rx], [ry])
                # Update quiver in-place (much faster than recreating)
                arrow_len = 0.5
                ux = math.cos(ryaw) * arrow_len
                uy = math.sin(ryaw) * arrow_len
                quiv.set_offsets([[rx, ry]])
                quiv.set_UVC(ux, uy)

            # update cmd text
            txt = f"cmd lin_x={lx:.3f}, lin_y={ly:.3f}, ang_z={az:.3f}\npose got={got_pose}, cmd got={got_cmd}"
            cmd_text.set_text(txt)

            # append derivative samples
            if deriv_ang_value is not None:
                deriv_ang_times.append(t)
                deriv_ang_vals.append(deriv_ang_value)
                deriv_ang_line.set_data(list(deriv_ang_times), list(deriv_ang_vals))
            
            if deriv_lin_value is not None:
                deriv_lin_times.append(t)
                deriv_lin_vals.append(deriv_lin_value)
                deriv_lin_line.set_data(list(deriv_lin_times), list(deriv_lin_vals))
            
            # update subplot axes (keep x window to last 30s)
            xmin = max(0.0, t - 30.0)
            ax2.set_xlim(xmin, t + 0.1)
            
            # compute y limits from both series
            all_vals = list(deriv_ang_vals) + list(deriv_lin_vals)
            if len(all_vals) > 0:
                ymin = min(all_vals)
                ymax = max(all_vals)
                if ymin == ymax:
                    ax2.set_ylim(ymin - 0.1, ymax + 0.1)
                else:
                    rng = (ymax - ymin) * 0.2
                    ax2.set_ylim(ymin - rng, ymax + rng)

            return line_wp, robot_point, quiv, cmd_text, deriv_ang_line, deriv_lin_line

        ani = animation.FuncAnimation(fig, update, interval=1000.0/args.rate, blit=False, cache_frame_data=False)
        
        plt.tight_layout()
        
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            pass

        # cleanup
        node.get_logger().info('Shutting down DebugVisualizer')
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        return 0


    if __name__ == '__main__':
        raise SystemExit(main())
