#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;

// Localization rate monitoring
auto last_odom_time = std::chrono::system_clock::now();
double odom_rate = 0.0;
int odom_count = 0;

int state = -1;

std::vector<double> waypoints_x;
std::vector<double> waypoints_y;




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("mover");

    // Declare tunable parameters (can be overridden via ROS params)
    nh->declare_parameter<double>("lookAheadDis", 1.2);  // Reduced for tighter control
    nh->declare_parameter<double>("yawRateGain", 0.6);   // Further reduced to prevent saturation
    nh->declare_parameter<double>("yawDerivativeGain", 0.4);  // Increased for more damping
    nh->declare_parameter<double>("lateralErrorGain", 0.8);  // New: gain for lateral error correction
    nh->declare_parameter<double>("yawErrorDeadband", 0.05);  // Ignore small errors (rad)
    nh->declare_parameter<double>("maxAngErrorForForward", 0.785);  // Stop forward motion if error > 45 deg (rad)
    nh->declare_parameter<double>("maxAccel", 0.5);
    nh->declare_parameter<double>("max_linear_speed", 0.4);
    nh->declare_parameter<double>("max_angular_speed", 0.25);  // Reduced from 0.35 to prevent saturation

    // Local variables to hold parameter values (defaults mirrored in declare_parameter)
    double lookAheadDis = 1.2;
    double yawRateGain = 0.6;
    double yawDerivativeGain = 0.4;
    double lateralErrorGain = 0.8;
    double yawErrorDeadband = 0.05;
    double maxAngErrorForForward = 0.785;  // ~45 degrees
    double maxAccel = 0.5;
    double max_linear_speed = 0.4;
    double max_angular_speed = 0.25;

    // Read params into local variables (overrides defaults above if provided)
    nh->get_parameter("lookAheadDis", lookAheadDis);
    nh->get_parameter("yawRateGain", yawRateGain);
    nh->get_parameter("yawDerivativeGain", yawDerivativeGain);
    nh->get_parameter("lateralErrorGain", lateralErrorGain);
    nh->get_parameter("yawErrorDeadband", yawErrorDeadband);
    nh->get_parameter("maxAngErrorForForward", maxAngErrorForForward);
    nh->get_parameter("maxAccel", maxAccel);
    nh->get_parameter("max_linear_speed", max_linear_speed);
    nh->get_parameter("max_angular_speed", max_angular_speed);

    // Load waypoints from CSV file
    std::ifstream file("/home/unitree/calib_imu/src/mover/src/waypoints.csv");
    std::string line;
    
    // Read header line and find x and y column indices
    std::getline(file, line);
    std::stringstream header_ss(line);
    std::string col_name;
    int x_col = -1, y_col = -1;
    int col_idx = 0;
    
    while (std::getline(header_ss, col_name, ',')) {
        // Trim whitespace from column name
        col_name.erase(0, col_name.find_first_not_of(" \t\r\n"));
        col_name.erase(col_name.find_last_not_of(" \t\r\n") + 1);
        
        RCLCPP_INFO(nh->get_logger(), "Column %d: '%s'", col_idx, col_name.c_str());
        
        if (col_name == "x") x_col = col_idx;
        if (col_name == "y") y_col = col_idx;
        col_idx++;
    }
    
    RCLCPP_INFO(nh->get_logger(), "Found x at column %d, y at column %d", x_col, y_col);
    
    if (x_col == -1 || y_col == -1) {
        RCLCPP_ERROR(nh->get_logger(), "Could not find 'x' and 'y' columns in CSV header");
        return -1;
    }
    // Read waypoint data
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row_values;
        
        // Read all columns into a vector
        while (std::getline(ss, value, ',')) {
            row_values.push_back(value);
        }
        
        // Extract x and y values from their respective columns
        if (row_values.size() > x_col && row_values.size() > y_col) {
            waypoints_x.push_back(std::stod(row_values[x_col]));
            waypoints_y.push_back(std::stod(row_values[y_col]));
        }
    }
    file.close();
    
    RCLCPP_INFO(nh->get_logger(), "Loaded %zu waypoints", waypoints_x.size());

    auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
    // Publishers for the raw derivative terms (not multiplied by Kd).
    // Visualizer `debug_waypoints.py` subscribes to these topics
    auto pubDerivativeAng = nh->create_publisher<std_msgs::msg::Float32>("/pd_derivative/angular", 10);
    auto pubDerivativeLin = nh->create_publisher<std_msgs::msg::Float32>("/pd_derivative/linear", 10);
    
    auto sub_odom = nh->create_subscription<nav_msgs::msg::Odometry>(
        "/localization", 10,
        [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_x = msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;

            const auto &q = msg->pose.pose.orientation;
            const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            current_yaw = std::atan2(siny_cosp, cosy_cosp);

            // Track localization rate
            auto now = std::chrono::system_clock::now();
            double dt_odom = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_odom_time).count();
            if (dt_odom > 1e-6) {
                odom_rate = 1.0 / dt_odom;
            }
            last_odom_time = now;
            odom_count++;
        });

    geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "vehicle";

    rclcpp::Rate rate(100);
    bool status = rclcpp::ok();
    auto beginning = std::chrono::system_clock::now();

    unitree_api::msg::Request req;
    SportClient sport_req;

    int i = 0;
    int pathPointID = 0;
    double vehicleSpeed = 0.0; // current commanded speed (m/s)
    double targetSpeed = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    
    // Derivative tracking for debugging/visualization
    double prev_ang_error = 0.0;  // Stores filtered error for PD controller
    double prev_ang_error_raw = 0.0;  // Stores raw error for visualization
    auto prev_ang_time = std::chrono::system_clock::now();
    double prev_lin_error = 0.0;
    auto prev_lin_time = std::chrono::system_clock::now();

    // Wait for first odometry message
    RCLCPP_INFO(nh->get_logger(), "Waiting for odometry data...");
    while (current_x == 0.0 && current_y == 0.0 && rclcpp::ok()) {
        rclcpp::spin_some(nh);
        rate.sleep();
    }
    RCLCPP_INFO(nh->get_logger(), "Odometry received: x=%.2f, y=%.2f, yaw=%.2f", current_x, current_y, current_yaw);

    // Initialize derivative state after first odometry to avoid large derivative spikes
    {
        auto now_init = std::chrono::system_clock::now();
        prev_ang_time = now_init;
        prev_lin_time = now_init;
        if (!waypoints_x.empty()) {
            double init_dx = waypoints_x[0] - current_x;
            double init_dy = waypoints_y[0] - current_y;
            double init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);
            double init_theta = std::atan2(init_dy, init_dx);
            double init_delta = init_theta - current_yaw;
            while (init_delta > M_PI) init_delta -= 2.0 * M_PI;
            while (init_delta < -M_PI) init_delta += 2.0 * M_PI;
            prev_lin_error = init_distance;
            prev_ang_error = init_delta;
            prev_ang_error_raw = init_delta;
        }
    }

    
    while (status){
        rclcpp::spin_some(nh);
        
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning).count();

        if(seconds < 5){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            sport_req.Move(req, 0, 0, 0);
            pubGo2Request->publish(req);
            
            state = 0;
        }else{
            state = 1;
        }
        if(state == 1){
        if (i < waypoints_x.size()){
            // Find lookahead point: farthest waypoint within lookahead distance
            // CRITICAL: Don't advance lookahead if robot is far from current waypoint
            // This prevents aiming at points behind/beside the robot during turns
            double dist_to_current_wp_dx = waypoints_x[i] - current_x;
            double dist_to_current_wp_dy = waypoints_y[i] - current_y;
            double dist_to_current_wp = std::sqrt(dist_to_current_wp_dx * dist_to_current_wp_dx + 
                                                   dist_to_current_wp_dy * dist_to_current_wp_dy);
            
            // Only advance lookahead if we're reasonably close to current waypoint
            // This prevents lookahead from jumping ahead during wide turns
            if (dist_to_current_wp < lookAheadDis * 1.5) {
                pathPointID = std::max(pathPointID, i);
            } else {
                // If far from current waypoint, don't advance lookahead beyond current waypoint
                pathPointID = i;
            }
            
            int bestLookaheadID = pathPointID;
            
            // Find the farthest waypoint that is still within lookahead distance
            for (int j = pathPointID; j < (int)waypoints_x.size(); j++) {
                double pdx = waypoints_x[j] - current_x;
                double pdy = waypoints_y[j] - current_y;
                double pdist = std::sqrt(pdx * pdx + pdy * pdy);
                
                if (pdist <= lookAheadDis) {
                    bestLookaheadID = j;  // This point is within lookahead distance
                } else {
                    break;  // Beyond lookahead distance, stop searching
                }
            }
            pathPointID = bestLookaheadID;

            // Advance waypoint index `i` based on multiple criteria:
            // 1. Close enough to current waypoint (proximity-based)
            // 2. Lookahead point is significantly ahead (we've likely passed the waypoint)
            double dist_i_dx = waypoints_x[i] - current_x;
            double dist_i_dy = waypoints_y[i] - current_y;
            double distance_i = std::sqrt(dist_i_dx * dist_i_dx + dist_i_dy * dist_i_dy);

            bool should_advance = false;
            
            // Criterion 1: Close enough to current waypoint (check both dx and dy separately)
            // This prevents advancing when robot is far laterally even if Euclidean distance is small
            double abs_dx = std::abs(dist_i_dx);
            double abs_dy = std::abs(dist_i_dy);
            if (abs_dx < 0.15 && abs_dy < 0.15 && i < (int)waypoints_x.size() - 1) {
                should_advance = true;
            }
            
            // Criterion 2: Lookahead point is at least 2 waypoints ahead (we've passed waypoint i)
            // This handles cases where oscillation prevents getting close to waypoint i
            if (!should_advance && pathPointID > i + 1 && i < (int)waypoints_x.size() - 1) {
                should_advance = true;
            }
            
            // Criterion 3: We're past waypoint i (check if we've crossed a line perpendicular to path)
            // This checks if the robot has moved past the waypoint along the path direction
            if (!should_advance && i < (int)waypoints_x.size() - 1) {
                // Vector from waypoint i to waypoint i+1 (path direction)
                double path_dx = waypoints_x[i+1] - waypoints_x[i];
                double path_dy = waypoints_y[i+1] - waypoints_y[i];
                double path_len = std::sqrt(path_dx * path_dx + path_dy * path_dy);
                
                if (path_len > 0.01) {  // Avoid division by zero
                    // Vector from waypoint i to robot
                    double robot_dx = current_x - waypoints_x[i];
                    double robot_dy = current_y - waypoints_y[i];
                    
                    // Project robot position onto path direction
                    double projection = (robot_dx * path_dx + robot_dy * path_dy) / path_len;
                    
                    // If projection is past waypoint i+1, we've passed waypoint i
                    if (projection > path_len * 0.8) {  // 80% past waypoint i toward i+1
                        should_advance = true;
                    }
                }
            }
            
            if (should_advance) {
                i++;
                RCLCPP_INFO(nh->get_logger(), "Advanced waypoint index to %d", i);
            }

            // Ensure we don't go beyond the last waypoint
            if (pathPointID >= (int)waypoints_x.size()) {
                pathPointID = waypoints_x.size() - 1;
            }

            // Calculate direction to lookahead point
            double tx = waypoints_x[pathPointID];
            double ty = waypoints_y[pathPointID];
            dx = tx - current_x;
            dy = ty - current_y;
            double lookahead_dist = std::sqrt(dx * dx + dy * dy);
            double pathDir = std::atan2(dy, dx);

            // Calculate lateral (cross-track) error to the current path segment
            // This is critical for converging onto the path instead of tracking parallel to it
            double lateral_error = 0.0;
            if (i < (int)waypoints_x.size() - 1) {
                // Vector from waypoint i to waypoint i+1 (path segment)
                double seg_dx = waypoints_x[i+1] - waypoints_x[i];
                double seg_dy = waypoints_y[i+1] - waypoints_y[i];
                double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
                
                if (seg_len > 0.01) {  // Avoid division by zero
                    // Vector from waypoint i to robot
                    double robot_dx = current_x - waypoints_x[i];
                    double robot_dy = current_y - waypoints_y[i];
                    
                    // Project robot position onto path segment
                    double projection = (robot_dx * seg_dx + robot_dy * seg_dy) / (seg_len * seg_len);
                    projection = std::max(0.0, std::min(1.0, projection));  // Clamp to [0, 1]
                    
                    // Closest point on path segment
                    double closest_x = waypoints_x[i] + projection * seg_dx;
                    double closest_y = waypoints_y[i] + projection * seg_dy;
                    
                    // Vector from closest point to robot (lateral error vector)
                    double lateral_dx = current_x - closest_x;
                    double lateral_dy = current_y - closest_y;
                    
                    // Calculate signed lateral error (positive = left of path, negative = right)
                    // Use cross product to determine sign: seg × robot_vector
                    double cross_product = seg_dx * lateral_dy - seg_dy * lateral_dx;
                    lateral_error = std::sqrt(lateral_dx * lateral_dx + lateral_dy * lateral_dy);
                    if (cross_product < 0) {
                        lateral_error = -lateral_error;  // Robot is to the right of path
                    }
                }
            } else {
                // At final waypoint, use distance to final waypoint as lateral error
                double final_dx = waypoints_x[i] - current_x;
                double final_dy = waypoints_y[i] - current_y;
                lateral_error = std::sqrt(final_dx * final_dx + final_dy * final_dy);
                // Could add sign based on robot orientation, but for final waypoint it's less critical
            }

            // Angular error between path direction and vehicle yaw
            double dirDiff = pathDir - current_yaw;
            // Normalize to [-pi, pi]
            while (dirDiff > M_PI) dirDiff -= 2.0 * M_PI;
            while (dirDiff < -M_PI) dirDiff += 2.0 * M_PI;

            // Angular velocity controller: PD controller (Proportional + Derivative) + Lateral Error Correction
            // Apply deadband to small errors to reduce jitter
            double dirDiffFiltered = dirDiff;
            if (std::abs(dirDiff) < yawErrorDeadband) {
                dirDiffFiltered = 0.0;
            }
            
            // Calculate derivative term (rate of change of error)
            auto now_ang = std::chrono::system_clock::now();
            double dt_ang = std::chrono::duration_cast<std::chrono::duration<double>>(now_ang - prev_ang_time).count();
            if (dt_ang <= 1e-6) dt_ang = 1.0 / 100.0;  // fallback to loop dt
            
            double error_derivative = (dirDiffFiltered - prev_ang_error) / dt_ang;
            
            // Lateral error correction: steer toward path when there's lateral offset
            // The correction is proportional to lateral error and inversely proportional to lookahead distance
            // This creates a "pull" toward the path
            double lateral_correction = lateralErrorGain * lateral_error / std::max(lookahead_dist, 0.5);
            // Limit lateral correction to prevent instability
            lateral_correction = std::max(-0.3, std::min(0.3, lateral_correction));
            
            // PD controller: P term + D term (derivative provides damping) + Lateral error correction
            double vehicleYawRate = yawRateGain * dirDiffFiltered - yawDerivativeGain * error_derivative + lateral_correction;
            
            // Adaptive angular speed limit: allow faster turning when error is large for quicker correction
            // Different limits for left (positive) and right (negative) turns due to hardware asymmetry
            double max_angular_left = max_angular_speed;  // 0.25 for left turns
            double max_angular_right = 0.4;  // 0.4 for right turns (hardware compensation)
            
            double abs_error = std::abs(dirDiff);
            if (abs_error > M_PI / 3.0) {  // Error > 60 degrees
                // Allow full speed when error is very large for faster correction
                // Keep at base values (no reduction)
                max_angular_left = max_angular_speed;  // 0.25
                max_angular_right = 0.4;
            } else if (abs_error > M_PI / 6.0) {  // Error > 30 degrees
                // Slightly reduce speed for medium errors to prevent overshoot
                max_angular_left = max_angular_speed * 0.9;  // 0.225
                max_angular_right = 0.4 * 0.9;  // 0.36
            }
            // For small errors (<= 30 degrees), use base values for precise control
            
            // Limit angular velocity with different limits for left vs right turns
            if (vehicleYawRate > max_angular_left) vehicleYawRate = max_angular_left;
            if (vehicleYawRate < -max_angular_right) vehicleYawRate = -max_angular_right;

            // Determine target forward speed based on:
            // 1. Angular error (CRITICAL: stop or nearly stop when error is large)
            // 2. Distance to lookahead point (slow down when close)
            // 3. Angular velocity magnitude (prevent spinning while moving)
            double speed_reduction_factor = 1.0;
            
            // CRITICAL FIX: Stop forward motion when angular error is too large
            // This prevents wide arcs and dangerous overshoot
            double abs_ang_error = std::abs(dirDiff);
            if (abs_ang_error > maxAngErrorForForward) {
                // Error > threshold (default 45 degrees): STOP forward motion, turn in place
                speed_reduction_factor = 0.0;
            } else if (abs_ang_error > M_PI / 3.0) {  // Error > 60 degrees (but less than maxAngErrorForForward)
                // Very aggressive speed reduction: scale from 0.05 at 60deg to 0.0 at maxAngErrorForForward
                speed_reduction_factor = 0.05 * (1.0 - (abs_ang_error - M_PI/3.0) / (maxAngErrorForForward - M_PI/3.0));
            } else if (abs_ang_error > M_PI / 4.0) {  // Error > 45 degrees
                // Aggressive speed reduction: scale from 0.15 at 45deg to 0.05 at 60deg
                speed_reduction_factor = 0.05 + 0.1 * (1.0 - (abs_ang_error - M_PI/4.0) / (M_PI/3.0 - M_PI/4.0));
            } else if (abs_ang_error > M_PI / 6.0) {  // Error > 30 degrees
                // Moderate speed reduction: scale from 0.4 at 30deg to 0.15 at 45deg
                speed_reduction_factor = 0.15 + 0.25 * (1.0 - (abs_ang_error - M_PI/6.0) / (M_PI/4.0 - M_PI/6.0));
            } else if (abs_ang_error > M_PI / 12.0) {  // Error > 15 degrees
                // Light speed reduction: scale from 0.7 at 15deg to 0.4 at 30deg
                speed_reduction_factor = 0.4 + 0.3 * (1.0 - (abs_ang_error - M_PI/12.0) / (M_PI/6.0 - M_PI/12.0));
            }
            
            // Reduce speed when close to lookahead point
            if (lookahead_dist < 1.0) {
                double proximity_factor = std::max(0.3, lookahead_dist / 1.0);
                speed_reduction_factor *= proximity_factor;
            }
            
            // Also reduce speed based on angular velocity magnitude (prevent spinning while moving)
            double abs_yaw_rate = std::abs(vehicleYawRate);
            if (abs_yaw_rate > 0.05) {  // If turning faster than 0.05 rad/s (lowered threshold)
                // More aggressive: reduce speed significantly when turning
                // Use the appropriate max angular speed based on turn direction
                double current_max_angular = (vehicleYawRate > 0) ? max_angular_left : max_angular_right;
                double yaw_rate_factor = std::max(0.3, 1.0 - (abs_yaw_rate / current_max_angular) * 0.7);
                speed_reduction_factor *= yaw_rate_factor;
            }
            
            // Check if we're at the final waypoint
            bool is_final_waypoint = (i >= (int)waypoints_x.size() - 1);
            double final_waypoint_dist = 0.0;
            bool should_stop_completely = false;
            
            if (is_final_waypoint) {
                double final_dx = waypoints_x[waypoints_x.size() - 1] - current_x;
                double final_dy = waypoints_y[waypoints_y.size() - 1] - current_y;
                final_waypoint_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy);
                
                // Stop completely when close enough to final waypoint AND angular error is small
                // This prevents oscillation around the final waypoint
                double abs_ang_error_final = std::abs(dirDiff);
                if (final_waypoint_dist < 0.2 && abs_ang_error_final < 0.2) {  // Within 20cm and < 11 deg error
                    should_stop_completely = true;
                    targetSpeed = 0.0;
                } else if (final_waypoint_dist < 0.15) {
                    // Very close - stop forward motion but allow small rotation if needed
                    targetSpeed = 0.0;
                } else if (final_waypoint_dist < 0.5) {
                    targetSpeed = max_linear_speed * (final_waypoint_dist / 0.5) * speed_reduction_factor;
                } else {
                    targetSpeed = max_linear_speed * speed_reduction_factor;
                }
            } else {
                targetSpeed = max_linear_speed * speed_reduction_factor;
            }

            // Ramp vehicleSpeed toward targetSpeed with acceleration limit
            double dt = 1.0 / 100.0; // loop dt (rate 100 Hz)
            double maxDelta = maxAccel * dt;
            if (vehicleSpeed < targetSpeed) {
                vehicleSpeed = std::min(targetSpeed, vehicleSpeed + maxDelta);
            } else if (vehicleSpeed > targetSpeed) {
                vehicleSpeed = std::max(targetSpeed, vehicleSpeed - maxDelta);
            }

            // Forward + Angular control: move forward, turn toward path
            // If we should stop completely at final waypoint, stop all motion
            if (should_stop_completely) {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.linear.y = 0.0;
                cmd_vel.twist.angular.z = 0.0;
                vehicleSpeed = 0.0;  // Reset speed for next iteration
            } else {
                cmd_vel.twist.linear.x = vehicleSpeed;  // Forward velocity in vehicle frame
                cmd_vel.twist.linear.y = 0.0;           // No lateral movement
                cmd_vel.twist.angular.z = vehicleYawRate; // Turn toward path
            }

            // Publish derivatives for debugging/visualization
            // Use the same timing as PD controller for consistency
            double dt_der = dt_ang;  // Use dt_ang calculated above for PD controller
            if (dt_der <= 1e-6) dt_der = dt;
            
            // Use raw (unfiltered) dirDiff for visualization (to see actual error rate)
            double ang_derivative = (dirDiff - prev_ang_error_raw) / dt_der;
            const double max_ang_derivative = 3.0;
            if (ang_derivative > max_ang_derivative) ang_derivative = max_ang_derivative;
            if (ang_derivative < -max_ang_derivative) ang_derivative = -max_ang_derivative;
            std_msgs::msg::Float32 ang_msg;
            ang_msg.data = static_cast<float>(ang_derivative);
            pubDerivativeAng->publish(ang_msg);

            // Linear derivative based on distance to lookahead point
            double dt_lin = std::chrono::duration_cast<std::chrono::duration<double>>(now_ang - prev_lin_time).count();
            if (dt_lin <= 1e-6) dt_lin = dt;
            double lin_derivative = (lookahead_dist - prev_lin_error) / dt_lin;
            const double max_lin_derivative = 2.0;
            if (lin_derivative > max_lin_derivative) lin_derivative = max_lin_derivative;
            if (lin_derivative < -max_lin_derivative) lin_derivative = -max_lin_derivative;
            std_msgs::msg::Float32 lin_msg;
            lin_msg.data = static_cast<float>(lin_derivative);
            pubDerivativeLin->publish(lin_msg);

            prev_ang_error = dirDiffFiltered;  // For PD controller
            prev_ang_error_raw = dirDiff;  // For visualization
            prev_ang_time = now_ang;
            prev_lin_error = lookahead_dist;
            prev_lin_time = now_ang;

            pubSpeed->publish(cmd_vel);
            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
            pubGo2Request->publish(req);

            // If we've reached the final waypoint and stopped, exit the control loop
            if (should_stop_completely) {
                RCLCPP_INFO(nh->get_logger(), "Final waypoint reached! Distance: %.3f m, Angular error: %.3f rad. Stopping robot.", 
                           final_waypoint_dist, std::abs(dirDiff));
                std::cout << "=== FINAL WAYPOINT REACHED ===" << std::endl;
                std::cout << "Final position: (" << current_x << ", " << current_y << ")" << std::endl;
                std::cout << "Target position: (" << waypoints_x[waypoints_x.size() - 1] << ", " 
                          << waypoints_y[waypoints_y.size() - 1] << ")" << std::endl;
                std::cout << "Distance to final waypoint: " << final_waypoint_dist << " m" << std::endl;
                std::cout << "Robot stopped successfully." << std::endl;
                
                // Keep sending stop commands for a few seconds to ensure robot stops
                for (int stop_count = 0; stop_count < 50; stop_count++) {  // 0.5 seconds at 100Hz
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.linear.y = 0.0;
                    cmd_vel.twist.angular.z = 0.0;
                    pubSpeed->publish(cmd_vel);
                    sport_req.Move(req, 0, 0, 0);
                    pubGo2Request->publish(req);
                    rate.sleep();
                    rclcpp::spin_some(nh);
                }
                
                break;  // Exit the main control loop
            }

            if(seconds % 1 == 0){
                std::cout << "=== Status Update ===" << std::endl;
                std::cout << "Localization rate: " << odom_rate << " Hz (received " << odom_count << " messages)" << std::endl;
                std::cout << "Current waypoint index: " << i << " / " << waypoints_x.size() << std::endl;
                std::cout << "Lookahead point index: " << pathPointID << std::endl;
                std::cout << "Current pose: (" << current_x << ", " << current_y << "), yaw=" << current_yaw << std::endl;
                std::cout << "Target waypoint: (" << waypoints_x[i] << ", " << waypoints_y[i] << ")" << std::endl;
                std::cout << "Lookahead point: (" << tx << ", " << ty << "), distance=" << lookahead_dist << std::endl;
                std::cout << "Angular error: " << dirDiff << " rad (" << dirDiff * 180.0 / M_PI << " deg)" << std::endl;
                std::cout << "Cmd: linear.x=" << cmd_vel.twist.linear.x << ", angular.z=" << cmd_vel.twist.angular.z << std::endl;
                if (is_final_waypoint) {
                    std::cout << "Final waypoint distance: " << final_waypoint_dist << " m" << std::endl;
                }
            }
        } else {
            // All waypoints reached - stop the robot
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            pubSpeed->publish(cmd_vel);
            sport_req.Move(req, 0, 0, 0);
            pubGo2Request->publish(req);
            
            std::cout << "All waypoints reached! Stopping robot." << std::endl;
            break;
        }
    }
        
        status = rclcpp::ok();
        rate.sleep();
    }
}
