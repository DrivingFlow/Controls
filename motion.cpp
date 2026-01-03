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
#include <std_msgs/msg/int32.hpp>

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

float current_x = 0.0f;
float current_y = 0.0f;
float current_yaw = 0.0f;

// Localization rate monitoring
auto last_odom_time = std::chrono::system_clock::now();
float odom_rate = 0.0f;
int odom_count = 0; 

int state = -1;

std::vector<float> waypoints_x;
std::vector<float> waypoints_y;

// ========== CONTROL CONSTANTS ==========
// Timing constants
const float CONTROL_LOOP_RATE = 100.0;  // Hz
const float CONTROL_LOOP_DT = 1.0 / CONTROL_LOOP_RATE;
const float MIN_TIME_DELTA = 1e-6f;
const int INITIAL_WAIT_TIME = 5;  // seconds
const int STATUS_UPDATE_INTERVAL = 1;  // seconds
const int FINAL_STOP_COMMAND_COUNT = 50;  // iterations (0.5s at 100Hz)

// Waypoint advancement thresholds
const float LOOKAHEAD_DISTANCE_MULTIPLIER = 1.5f;
const float WAYPOINT_PROJECTION_THRESHOLD = 0.8f;  // 80% past waypoint
const float MIN_PATH_LENGTH = 0.01f;  // meters, to avoid division by zero

// Angular error thresholds
const float ANGLE_45_DEG = M_PI / 3.0f;
const float ANGLE_30_DEG = M_PI / 4.0f;
const float ANGLE_15_DEG = M_PI / 6.0f;
const float ANGLE_5_DEG = M_PI / 18.0f;

// Speed reduction factors
const float SPEED_REDUCTION_45DEG = 0.05f;
const float SPEED_REDUCTION_30DEG = 0.15f;
const float SPEED_REDUCTION_15DEG = 0.4f;
const float SPEED_REDUCTION_5DEG = 0.7f;
const float ANGULAR_SPEED_SCALING = 0.9f;  // For medium angular errors

// Angular velocity control
const float RIGHT_TURN_COMPENSATION = 0.15f;  // rad/s, added to max for hardware asymmetry
const float YAW_RATE_THRESHOLD = 0.05f;  // rad/s, minimum rate to trigger speed reduction
const float MIN_YAW_RATE_FACTOR = 0.3f;
const float YAW_RATE_REDUCTION = 0.7f;

// Lateral error correction
const float MIN_LOOKAHEAD_FOR_LATERAL = 0.5f;  // meters
const float MAX_LATERAL_CORRECTION = 0.3f;  // rad/s
const float MIN_LATERAL_CORRECTION = -0.3f;  // rad/s

// Proximity-based speed control
const float PROXIMITY_DISTANCE = 1.0f;  // meters
const float MIN_PROXIMITY_FACTOR = 0.3f;

// Final waypoint thresholds
const float FINAL_WAYPOINT_DISTANCE_THRESHOLD = 0.2f;  // meters
const float FINAL_WAYPOINT_ANGLE_THRESHOLD = 0.2f;  // rad (~11 degrees)
const float FINAL_WAYPOINT_VERY_CLOSE = 0.15f;  // meters
const float FINAL_WAYPOINT_SLOW_DOWN = 0.5f;  // meters

// Derivative term limits (for visualization/debugging)
const float MAX_ANGULAR_DERIVATIVE = 3.0f;  // rad/s²
const float MAX_LINEAR_DERIVATIVE = 2.0f;  // m/s²




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("mover");

    // Declare tunable parameters (can be overridden via ROS params)
    nh->declare_parameter<float>("lookAheadDis", 1.2f);  // Reduced for tighter control
    nh->declare_parameter<float>("yawRateGain", 0.6f);   // Further reduced to prevent saturation
    nh->declare_parameter<float>("yawDerivativeGain", 0.4f);  // Increased for more damping
    nh->declare_parameter<float>("lateralErrorGain", 0.0f);  // Lateral error correction DISABLED (was causing oscillation)
    nh->declare_parameter<float>("yawErrorDeadband", 0.05f);  // Ignore small errors (rad)
    nh->declare_parameter<float>("maxAngErrorForForward", 0.785f);  // Stop forward motion if error > 45 deg (rad)
    nh->declare_parameter<float>("maxAccel", 0.5f);
    nh->declare_parameter<float>("max_linear_speed", 0.4f);
    nh->declare_parameter<float>("max_angular_speed", 0.35f);  // Reduced from 0.35 to prevent saturation
    nh->declare_parameter<float>("pointAcheivedDist", 0.1f); // Distance threshold to consider waypoint reached

    // Local variables to hold parameter values (defaults mirrored in declare_parameter)
    float lookAheadDis = 0.8f;
    float yawRateGain = 0.6f;
    float yawDerivativeGain = 0.4f;
    float lateralErrorGain = 0.0f;  // DISABLED
    float yawErrorDeadband = 0.05f;
    float maxAngErrorForForward = 0.785f;  // ~45 degrees
    float maxAccel = 0.5f;
    float max_linear_speed = 0.4f;
    float max_angular_speed = 0.35f;
    float pointAcheivedDist = 0.1f; // Distance threshold to consider waypoint reached

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
    // Publishers for waypoint indices for visualization
    auto pubLookaheadWaypoint = nh->create_publisher<std_msgs::msg::Int32>("/waypoint/lookahead", 10);
    auto pubCurrentWaypoint = nh->create_publisher<std_msgs::msg::Int32>("/waypoint/current", 10);
    
    auto sub_odom = nh->create_subscription<nav_msgs::msg::Odometry>(
        "/localization", 10,
        [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_x = msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;

            const auto &q = msg->pose.pose.orientation;
            const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
            const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            current_yaw = std::atan2(siny_cosp, cosy_cosp);

            // Track localization rate
            auto now = std::chrono::system_clock::now();
            float dt_odom = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_odom_time).count();
            if (dt_odom > 1e-6f) {
                odom_rate = 1.0f / dt_odom;
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
    float vehicleSpeed = 0.0; // current commanded speed (m/s)
    float targetSpeed = 0.0;
    float dx = 0.0;
    float dy = 0.0;
    
    // Derivative tracking for debugging/visualization
    float prev_ang_error = 0.0;  // Stores filtered error for PD controller
    float prev_ang_error_raw = 0.0;  // Stores raw error for visualization
    auto prev_ang_time = std::chrono::system_clock::now();
    float prev_lin_error = 0.0;
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
            float init_dx = waypoints_x[0] - current_x;
            float init_dy = waypoints_y[0] - current_y;
            float init_distance = std::sqrt(init_dx * init_dx + init_dy * init_dy);
            float init_theta = std::atan2(init_dy, init_dx);
            float init_delta = init_theta - current_yaw;
            while (init_delta > M_PI) init_delta -= 2.0f * M_PI;
            while (init_delta < -M_PI) init_delta += 2.0f * M_PI;
            prev_lin_error = init_distance;
            prev_ang_error = init_delta;
            prev_ang_error_raw = init_delta;
        }
    }

    
    while (status){
        rclcpp::spin_some(nh);
        
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning).count();

        if(seconds < INITIAL_WAIT_TIME){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            sport_req.Move(req, 0, 0, 0);
            pubGo2Request->publish(req);
            
            // Publish waypoint indices even during initial wait
            std_msgs::msg::Int32 lookahead_msg;
            lookahead_msg.data = pathPointID;
            pubLookaheadWaypoint->publish(lookahead_msg);
            
            std_msgs::msg::Int32 current_msg;
            current_msg.data = i;
            pubCurrentWaypoint->publish(current_msg);
            
            state = 0;
        }else{
            state = 1;
        }
        if(state == 1){
        if (i < waypoints_x.size()){
            // Find lookahead point: farthest waypoint within lookahead distance
            // CRITICAL: Don't advance lookahead if robot is far from current waypoint
            // This prevents aiming at points behind/beside the robot during turns
            float dist_to_current_wp_dx = waypoints_x[i] - current_x;
            float dist_to_current_wp_dy = waypoints_y[i] - current_y;
            float dist_to_current_wp = std::sqrt(dist_to_current_wp_dx * dist_to_current_wp_dx + 
                                                   dist_to_current_wp_dy * dist_to_current_wp_dy);
            
            // Only advance lookahead if we're reasonably close to current waypoint
            // This prevents lookahead from jumping ahead during wide turns
            if (dist_to_current_wp < lookAheadDis * LOOKAHEAD_DISTANCE_MULTIPLIER) {
                pathPointID = std::max(pathPointID, i);
            } else {
                // If far from current waypoint, don't advance lookahead beyond current waypoint
                pathPointID = i;
            }
            
            int bestLookaheadID = pathPointID;
            
            // Find the farthest waypoint that is still within lookahead distance
            for (int j = pathPointID; j < (int)waypoints_x.size(); j++) {
                float pdx = waypoints_x[j] - current_x;
                float pdy = waypoints_y[j] - current_y;
                float pdist = std::sqrt(pdx * pdx + pdy * pdy);
                
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
            float dist_i_dx = waypoints_x[i] - current_x;
            float dist_i_dy = waypoints_y[i] - current_y;
            float distance_i = std::sqrt(dist_i_dx * dist_i_dx + dist_i_dy * dist_i_dy);

            bool should_advance = false;
            
            // Criterion 1: Close enough to current waypoint (check both dx and dy separately)
            // This prevents advancing when robot is far laterally even if Euclidean distance is small
            float abs_dx = std::abs(dist_i_dx);
            float abs_dy = std::abs(dist_i_dy);
            if (abs_dx < pointAcheivedDist && abs_dy < pointAcheivedDist && i < (int)waypoints_x.size() - 1) {
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
                float path_dx = waypoints_x[i+1] - waypoints_x[i];
                float path_dy = waypoints_y[i+1] - waypoints_y[i];
                float path_len = std::sqrt(path_dx * path_dx + path_dy * path_dy);
                
                if (path_len > MIN_PATH_LENGTH) {  // Avoid division by zero
                    // Vector from waypoint i to robot
                    float robot_dx = current_x - waypoints_x[i];
                    float robot_dy = current_y - waypoints_y[i];
                    
                    // Project robot position onto path direction
                    float projection = (robot_dx * path_dx + robot_dy * path_dy) / path_len;
                    
                    // If projection is past waypoint i+1, we've passed waypoint i
                    if (projection > path_len * WAYPOINT_PROJECTION_THRESHOLD) {
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
            float tx = waypoints_x[pathPointID];
            float ty = waypoints_y[pathPointID];
            dx = tx - current_x;
            dy = ty - current_y;
            float lookahead_dist = std::sqrt(dx * dx + dy * dy);
            float pathDir = std::atan2(dy, dx);

            // Calculate lateral (cross-track) error to the current path segment
            // This is critical for converging onto the path instead of tracking parallel to it
            float lateral_error = 0.0;
            if (i < (int)waypoints_x.size() - 1) {
                // Vector from waypoint i to waypoint i+1 (path segment)
                float seg_dx = waypoints_x[i+1] - waypoints_x[i];
                float seg_dy = waypoints_y[i+1] - waypoints_y[i];
                float seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
                
                if (seg_len > MIN_PATH_LENGTH) {  // Avoid division by zero
                    // Vector from waypoint i to robot
                    float robot_dx = current_x - waypoints_x[i];
                    float robot_dy = current_y - waypoints_y[i];
                    
                    // Project robot position onto path segment
                    float projection = (robot_dx * seg_dx + robot_dy * seg_dy) / (seg_len * seg_len);
                    projection = std::max(0.0f, std::min(1.0f, projection));  // Clamp to [0, 1]
                    
                    // Closest point on path segment
                    float closest_x = waypoints_x[i] + projection * seg_dx;
                    float closest_y = waypoints_y[i] + projection * seg_dy;
                    
                    // Vector from closest point to robot (lateral error vector)
                    float lateral_dx = current_x - closest_x;
                    float lateral_dy = current_y - closest_y;
                    
                    // Calculate signed lateral error (positive = left of path, negative = right)
                    // Use cross product to determine sign: seg × robot_vector
                    float cross_product = seg_dx * lateral_dy - seg_dy * lateral_dx;
                    lateral_error = std::sqrt(lateral_dx * lateral_dx + lateral_dy * lateral_dy);
                    if (cross_product < 0) {
                        lateral_error = -lateral_error;  // Robot is to the right of path
                    }
                }
            } else {
                // At final waypoint, use distance to final waypoint as lateral error
                float final_dx = waypoints_x[i] - current_x;
                float final_dy = waypoints_y[i] - current_y;
                lateral_error = std::sqrt(final_dx * final_dx + final_dy * final_dy);
                // Could add sign based on robot orientation, but for final waypoint it's less critical
            }

            // === LATERAL ERROR CORRECTION VIA HEADING ADJUSTMENT (Stanley-style) ===
            // Instead of adding lateral correction to angular velocity (causes overshoot),
            // we adjust the TARGET HEADING to include path convergence.
            // This lets the PD controller handle smoothing naturally.
            
            // Calculate heading correction angle based on lateral error
            // atan2(k * lateral_error, velocity) is the Stanley controller formula
            // We use lookahead_dist as a proxy for "how far ahead we're looking"
            float heading_correction = std::atan2(lateralErrorGain * lateral_error, 
                                                   std::max(lookahead_dist, MIN_LOOKAHEAD_FOR_LATERAL));
            
            // Limit heading correction to prevent extreme adjustments (max ~8.5 degrees)
            const float MAX_HEADING_CORRECTION = 0.15f;  // radians (~8.5 degrees)
            heading_correction = std::max(-MAX_HEADING_CORRECTION, std::min(MAX_HEADING_CORRECTION, heading_correction));
            
            // Adjust target heading: subtract correction because positive lateral_error (left of path)
            // should result in turning right (reducing the target heading angle)
            float adjusted_pathDir = pathDir - heading_correction;
            
            // Angular error between ADJUSTED path direction and vehicle yaw
            float dirDiff = adjusted_pathDir - current_yaw;
            // Normalize to [-pi, pi]
            while (dirDiff > M_PI) dirDiff -= 2.0 * M_PI;
            while (dirDiff < -M_PI) dirDiff += 2.0 * M_PI;

            // Angular velocity controller: PD controller (Proportional + Derivative)
            // Apply deadband to small errors to reduce jitter
            float dirDiffFiltered = dirDiff;
            if (std::abs(dirDiff) < yawErrorDeadband) {
                dirDiffFiltered = 0.0;
            }
            
            // Calculate derivative term (rate of change of error)
            auto now_ang = std::chrono::system_clock::now();
            float dt_ang = std::chrono::duration_cast<std::chrono::duration<float>>(now_ang - prev_ang_time).count();
            if (dt_ang <= MIN_TIME_DELTA) dt_ang = CONTROL_LOOP_DT;  // fallback to loop dt
            
            float error_derivative = (dirDiffFiltered - prev_ang_error) / dt_ang;
            
            // Store lateral correction for debug logging (now it's the heading adjustment, not yaw rate)
            float lateral_correction = heading_correction;
            
            // PD controller: P term + D term (derivative provides damping)
            // Lateral error is now integrated into dirDiff via heading adjustment above
            float vehicleYawRate = yawRateGain * dirDiffFiltered - yawDerivativeGain * error_derivative;
            
            // Adaptive angular speed limit: allow faster turning when error is large for quicker correction
            // Different limits for left (positive) and right (negative) turns due to hardware asymmetry
            float max_angular_left = max_angular_speed;
            float max_angular_right = max_angular_speed + RIGHT_TURN_COMPENSATION;  // Hardware compensation
            
            float abs_error = std::abs(dirDiff);
            if (abs_error > ANGLE_45_DEG) {
                // Allow full speed when error is very large for faster correction
                // Keep at base values (no reduction)
                max_angular_left = max_angular_speed;
                max_angular_right = max_angular_speed + RIGHT_TURN_COMPENSATION;
            } else if (abs_error > ANGLE_15_DEG) {
                // Slightly reduce speed for medium errors to prevent overshoot
                max_angular_left = max_angular_speed * ANGULAR_SPEED_SCALING;
                max_angular_right = (max_angular_speed + RIGHT_TURN_COMPENSATION) * ANGULAR_SPEED_SCALING;
            }
            // For small errors (<= 30 degrees), use base values for precise control
            
            // Limit angular velocity with different limits for left vs right turns
            if (vehicleYawRate > max_angular_left) vehicleYawRate = max_angular_left;
            if (vehicleYawRate < -max_angular_right) vehicleYawRate = -max_angular_right;

            // Determine target forward speed based on:
            // 1. Angular error (CRITICAL: stop or nearly stop when error is large)
            // 2. Distance to lookahead point (slow down when close)
            // 3. Angular velocity magnitude (prevent spinning while moving)
            float speed_reduction_factor = 1.0;
            
            // CRITICAL FIX: Stop forward motion when angular error is too large
            // This prevents wide arcs and dangerous overshoot
            float abs_ang_error = std::abs(dirDiff);
            if (abs_ang_error > maxAngErrorForForward) {
                // Error > threshold (default 45 degrees): STOP forward motion, turn in place
                speed_reduction_factor = 0.0;
            } else if (abs_ang_error > ANGLE_45_DEG) {  // Error > 45 degrees (but less than maxAngErrorForForward)
                // Very aggressive speed reduction: scale from SPEED_REDUCTION_45DEG at 45DEG to 0.0 at maxAngErrorForForward
                speed_reduction_factor = SPEED_REDUCTION_45DEG * (1.0 - (abs_ang_error - ANGLE_45_DEG) / (maxAngErrorForForward - ANGLE_45_DEG));
            } else if (abs_ang_error > ANGLE_30_DEG) {  // Error > 30 degrees
                // Aggressive speed reduction: scale from SPEED_REDUCTION_30DEG at 30DEG to SPEED_REDUCTION_45DEG at 45DEG
                speed_reduction_factor = SPEED_REDUCTION_45DEG + (SPEED_REDUCTION_30DEG - SPEED_REDUCTION_45DEG) * (1.0 - (abs_ang_error - ANGLE_30_DEG) / (ANGLE_45_DEG - ANGLE_30_DEG));
            } else if (abs_ang_error > ANGLE_15_DEG) {  // Error > 15 degrees
                // Moderate speed reduction: scale from SPEED_REDUCTION_15DEG at 15DEG to SPEED_REDUCTION_30DEG at 30DEG
                speed_reduction_factor = SPEED_REDUCTION_30DEG + (SPEED_REDUCTION_15DEG - SPEED_REDUCTION_30DEG) * (1.0 - (abs_ang_error - ANGLE_15_DEG) / (ANGLE_30_DEG - ANGLE_15_DEG));
            } else if (abs_ang_error > ANGLE_5_DEG) {  // Error > 5 degrees
                // Light speed reduction: scale from SPEED_REDUCTION_5DEG at 5DEG to SPEED_REDUCTION_15DEG at 15DEG
                speed_reduction_factor = SPEED_REDUCTION_15DEG + (SPEED_REDUCTION_5DEG - SPEED_REDUCTION_15DEG) * (1.0 - (abs_ang_error - ANGLE_5_DEG) / (ANGLE_15_DEG - ANGLE_5_DEG));
            }
            
            // Reduce speed when close to lookahead point
            if (lookahead_dist < PROXIMITY_DISTANCE) {
                float proximity_factor = std::max(MIN_PROXIMITY_FACTOR, lookahead_dist / PROXIMITY_DISTANCE);
                speed_reduction_factor *= proximity_factor;
            }
            
            // Also reduce speed based on angular velocity magnitude (prevent spinning while moving)
            float abs_yaw_rate = std::abs(vehicleYawRate);
            if (abs_yaw_rate > YAW_RATE_THRESHOLD) {
                // More aggressive: reduce speed significantly when turning
                // Use the appropriate max angular speed based on turn direction
                float current_max_angular = (vehicleYawRate > 0) ? max_angular_left : max_angular_right;
                float yaw_rate_factor = std::max(MIN_YAW_RATE_FACTOR, 1.0f - (abs_yaw_rate / current_max_angular) * YAW_RATE_REDUCTION);
                speed_reduction_factor *= yaw_rate_factor;
            }
            
            // Check if we're at the final waypoint
            bool is_final_waypoint = (i >= (int)waypoints_x.size() - 1);
            float final_waypoint_dist = 0.0;
            bool should_stop_completely = false;
            
            if (is_final_waypoint) {
                float final_dx = waypoints_x[waypoints_x.size() - 1] - current_x;
                float final_dy = waypoints_y[waypoints_y.size() - 1] - current_y;
                final_waypoint_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy);
                
                // Stop completely when close enough to final waypoint AND angular error is small
                // This prevents oscillation around the final waypoint
                float abs_ang_error_final = std::abs(dirDiff);
                if (final_waypoint_dist < FINAL_WAYPOINT_DISTANCE_THRESHOLD && abs_ang_error_final < FINAL_WAYPOINT_ANGLE_THRESHOLD) {
                    should_stop_completely = true;
                    targetSpeed = 0.0;
                } else if (final_waypoint_dist < FINAL_WAYPOINT_VERY_CLOSE) {
                    // Very close - stop forward motion but allow small rotation if needed
                    targetSpeed = 0.0;
                } else if (final_waypoint_dist < FINAL_WAYPOINT_SLOW_DOWN) {
                    targetSpeed = max_linear_speed * (final_waypoint_dist / FINAL_WAYPOINT_SLOW_DOWN) * speed_reduction_factor;
                } else {
                    targetSpeed = max_linear_speed * speed_reduction_factor;
                }
            } else {
                targetSpeed = max_linear_speed * speed_reduction_factor;
            }

            // Ramp vehicleSpeed toward targetSpeed with acceleration limit
            float dt = CONTROL_LOOP_DT;
            float maxDelta = maxAccel * dt;
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
            float dt_der = dt_ang;  // Use dt_ang calculated above for PD controller
            if (dt_der <= 1e-6f) dt_der = dt;
            
            // Use raw (unfiltered) dirDiff for visualization (to see actual error rate)
            float ang_derivative = (dirDiff - prev_ang_error_raw) / dt_der;
            if (ang_derivative > MAX_ANGULAR_DERIVATIVE) ang_derivative = MAX_ANGULAR_DERIVATIVE;
            if (ang_derivative < -MAX_ANGULAR_DERIVATIVE) ang_derivative = -MAX_ANGULAR_DERIVATIVE;
            std_msgs::msg::Float32 ang_msg;
            ang_msg.data = static_cast<float>(ang_derivative);
            pubDerivativeAng->publish(ang_msg);

            // Linear derivative based on distance to lookahead point
            float dt_lin = std::chrono::duration_cast<std::chrono::duration<float>>(now_ang - prev_lin_time).count();
            if (dt_lin <= MIN_TIME_DELTA) dt_lin = dt;
            float lin_derivative = (lookahead_dist - prev_lin_error) / dt_lin;
            if (lin_derivative > MAX_LINEAR_DERIVATIVE) lin_derivative = MAX_LINEAR_DERIVATIVE;
            if (lin_derivative < -MAX_LINEAR_DERIVATIVE) lin_derivative = -MAX_LINEAR_DERIVATIVE;
            std_msgs::msg::Float32 lin_msg;
            lin_msg.data = static_cast<float>(lin_derivative);
            pubDerivativeLin->publish(lin_msg);

            prev_ang_error = dirDiffFiltered;  // For PD controller
            prev_ang_error_raw = dirDiff;  // For visualization
            prev_ang_time = now_ang;
            prev_lin_error = lookahead_dist;
            prev_lin_time = now_ang;

            // Publish waypoint indices for visualization
            std_msgs::msg::Int32 lookahead_msg;
            lookahead_msg.data = pathPointID;
            pubLookaheadWaypoint->publish(lookahead_msg);
            
            std_msgs::msg::Int32 current_msg;
            current_msg.data = i;
            pubCurrentWaypoint->publish(current_msg);

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
                for (int stop_count = 0; stop_count < FINAL_STOP_COMMAND_COUNT; stop_count++) {
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

            if(seconds % STATUS_UPDATE_INTERVAL == 0){
                std::cout << "=== Status Update ===" << std::endl;
                std::cout << "Localization rate: " << odom_rate << " Hz (received " << odom_count << " messages)" << std::endl;
                std::cout << "Current waypoint index: " << i << " / " << waypoints_x.size() << std::endl;
                std::cout << "Lookahead point index: " << pathPointID << std::endl;
                std::cout << "Current pose: (" << current_x << ", " << current_y << "), yaw=" << current_yaw << std::endl;
                std::cout << "Target waypoint: (" << waypoints_x[i] << ", " << waypoints_y[i] << ")" << std::endl;
                std::cout << "Lookahead point: (" << tx << ", " << ty << "), distance=" << lookahead_dist << std::endl;
                std::cout << "Angular error: " << dirDiff << " rad (" << dirDiff * 180.0 / M_PI << " deg)" << std::endl;
                std::cout << "Lateral error: " << lateral_error << " m (+ = left of path, - = right)" << std::endl;
                std::cout << "Heading correction: " << lateral_correction << " rad (" << lateral_correction * 180.0 / M_PI << " deg)" << std::endl;
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
            
            // Publish final waypoint indices
            std_msgs::msg::Int32 lookahead_msg;
            lookahead_msg.data = (int)waypoints_x.size() - 1;
            pubLookaheadWaypoint->publish(lookahead_msg);
            
            std_msgs::msg::Int32 current_msg;
            current_msg.data = (int)waypoints_x.size() - 1;
            pubCurrentWaypoint->publish(current_msg);
            
            std::cout << "All waypoints reached! Stopping robot." << std::endl;
            break;
        }
    }
        
        status = rclcpp::ok();
        rate.sleep();
    }
}
