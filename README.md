# Controller Package

This package contains the motion control system for the Unitree Go2 robot, implementing waypoint-based path following with PD control and a debugging visualization tool.

## Overview

The controller package consists of two main components:

1. **motion** - A ROS2 C++ node that implements path-following control using waypoints
2. **debugwaypoints** - A Python visualization tool for debugging waypoint tracking and control performance

## Components

### motion.cpp

The `motion` node is the core path-following controller that:

- Loads waypoints from a CSV file (`/home/unitree/calib_imu/src/mover/src/waypoints.csv`)
- Implements a lookahead-based path following algorithm
- Uses PD (Proportional-Derivative) control for angular velocity
- Publishes velocity commands to control the Unitree Go2 robot
- Monitors localization rate and robot state

#### Features

- **Lookahead Distance Control**: Selects the farthest waypoint within a configurable lookahead distance
- **PD Controller**: Proportional-Derivative control for smooth angular motion with damping
- **Lateral Error Correction**: Corrects cross-track error to converge onto the path
- **Adaptive Speed Control**: Reduces forward speed based on angular error and proximity to waypoints
- **Waypoint Advancement**: Multiple criteria for advancing to the next waypoint (proximity, lookahead, path projection)
- **Final Waypoint Handling**: Special logic for stopping precisely at the final waypoint

#### ROS2 Topics

**Subscribed Topics:**
- `/localization` (nav_msgs/Odometry) - Robot pose and orientation

**Published Topics:**
- `/cmd_vel` (geometry_msgs/TwistStamped) - Velocity commands for the robot
- `/api/sport/request` (unitree_api/msg/Request) - Unitree Go2 API commands
- `/pd_derivative/angular` (std_msgs/Float32) - Angular error derivative (for debugging)
- `/pd_derivative/linear` (std_msgs/Float32) - Linear error derivative (for debugging)

#### Parameters

All parameters can be set via ROS2 parameters or have default values:

- `lookAheadDis` (default: 1.2 m) - Maximum lookahead distance for path following
- `yawRateGain` (default: 0.6) - Proportional gain for angular velocity control
- `yawDerivativeGain` (default: 0.4) - Derivative gain for angular velocity damping
- `lateralErrorGain` (default: 0.8) - Gain for lateral error correction
- `yawErrorDeadband` (default: 0.05 rad) - Deadband for small angular errors
- `maxAngErrorForForward` (default: 0.785 rad ≈ 45°) - Maximum angular error before stopping forward motion
- `maxAccel` (default: 0.5 m/s²) - Maximum acceleration for speed ramping
- `max_linear_speed` (default: 0.4 m/s) - Maximum forward speed
- `max_angular_speed` (default: 0.25 rad/s) - Maximum angular velocity

#### Waypoint File Format

The waypoint CSV file must have a header row with `x` and `y` columns:

```csv
x,y
0.0,0.0
1.0,0.0
2.0,1.0
...
```

#### Usage

```bash
# Run with default parameters
ros2 run mover motion

# Run with custom parameters
ros2 run mover motion --ros-args -p lookAheadDis:=1.5 -p max_linear_speed:=0.5
```

### debugwaypoints (debug_waypoints.py)

A Python visualization tool that provides real-time debugging of the path-following controller.

#### Features

- **Real-time Visualization**: Displays waypoints, robot pose, and trajectory history
- **Derivative Monitoring**: Visualizes PD controller derivative terms
- **Command Display**: Shows current velocity commands
- **Trajectory Tracking**: Records and displays the robot's path

#### ROS2 Topics

**Subscribed Topics:**
- `/localization` (nav_msgs/Odometry) - Robot pose
- `/cmd_vel` (geometry_msgs/TwistStamped) - Velocity commands
- `/pd_derivative/angular` (std_msgs/Float32) - Angular derivative (optional)
- `/pd_derivative/linear` (std_msgs/Float32) - Linear derivative (optional)

#### Usage

```bash
# Run with default settings
python3 debug_waypoints.py

# Run with custom CSV file and topics
python3 debug_waypoints.py --csv /path/to/waypoints.csv --odom /localization --cmd /cmd_vel

# Use estimated derivatives instead of published ones
python3 debug_waypoints.py --derivative-source estimate
```

#### Command Line Arguments

- `--csv`, `-c` - Path to waypoints CSV file (default: `/home/unitree/calib_imu/src/mover/src/waypoints.csv`)
- `--odom` - Odometry topic name (default: `/localization`)
- `--cmd` - Command velocity topic name (default: `/cmd_vel`)
- `--derivative-topic-angular` - Angular derivative topic (default: `/pd_derivative/angular`)
- `--derivative-topic-linear` - Linear derivative topic (default: `/pd_derivative/linear`)
- `--derivative-source` - Source for derivatives: `publish` (subscribe) or `estimate` (numerical differentiation) (default: `publish`)
- `--rate` - Plot update rate in Hz (default: 10.0)

## Control Algorithm

### Path Following Strategy

1. **Lookahead Point Selection**: The controller finds the farthest waypoint within the lookahead distance from the current robot position.

2. **Angular Control**: 
   - Calculates the angular error between the robot's heading and the direction to the lookahead point
   - Applies PD control: `angular_velocity = Kp * error - Kd * error_derivative + lateral_correction`
   - Includes lateral error correction to pull the robot toward the path

3. **Speed Control**:
   - Reduces forward speed when angular error is large (prevents wide arcs)
   - Reduces speed when close to waypoints
   - Reduces speed when turning (prevents spinning while moving)
   - Stops completely when angular error exceeds threshold

4. **Waypoint Advancement**:
   - Advances to next waypoint when within 15cm (x and y separately)
   - Advances when lookahead point is 2+ waypoints ahead
   - Advances when robot has passed waypoint along path direction

5. **Final Waypoint**:
   - Stops completely when within 20cm and angular error < 0.2 rad
   - Gradually reduces speed as approaching final waypoint

## Dependencies

### motion.cpp
- ROS2 (rclcpp)
- geometry_msgs
- nav_msgs
- std_msgs
- unitree_api
- go2_sport_api

### debugwaypoints
- ROS2 Python (rclpy)
- matplotlib
- numpy

## Building

```bash
cd calib_imu
colcon build --packages-select mover
source install/setup.bash
```

## Troubleshooting

### Robot not moving
- Check that odometry is being published on `/localization`
- Verify waypoints CSV file exists and is readable
- Check that waypoint file has correct `x` and `y` column headers
- Ensure robot is initialized (5-second startup delay)

### Robot oscillating around waypoints
- Reduce `lookAheadDis` parameter
- Increase `yawDerivativeGain` for more damping
- Adjust `yawErrorDeadband` to ignore small errors

### Robot overshooting turns
- Reduce `max_linear_speed`
- Reduce `maxAngErrorForForward` to stop earlier
- Increase `yawRateGain` for faster turning

### Localization rate issues
- Monitor the localization rate printed in status updates
- Ensure localization node is running and publishing at adequate rate
- Check network connectivity if using remote localization

## Status Output

The motion node prints status updates every second including:
- Localization rate (Hz)
- Current waypoint index
- Lookahead point index
- Current pose (x, y, yaw)
- Target waypoint coordinates
- Angular error
- Command velocities

## Notes

- The controller waits 5 seconds after startup before beginning motion
- The controller waits for the first odometry message before starting
- Derivative terms are published for debugging/visualization purposes
- The controller runs at 100 Hz control loop rate
