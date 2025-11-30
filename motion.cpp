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
    nh->declare_parameter<double>("lookAheadDis", 1.0);
    nh->declare_parameter<double>("yawRateGain", 1.5);
    nh->declare_parameter<double>("maxAccel", 0.5);
    nh->declare_parameter<double>("max_linear_speed", 0.4);
    nh->declare_parameter<double>("max_angular_speed", 0.35);

    // Local variables to hold parameter values (defaults mirrored in declare_parameter)
    double lookAheadDis = 1.0;
    double yawRateGain = 1.5;
    double maxAccel = 0.5;
    double max_linear_speed = 0.4;
    double max_angular_speed = 0.35;

    // Read params into local variables (overrides defaults above if provided)
    nh->get_parameter("lookAheadDis", lookAheadDis);
    nh->get_parameter("yawRateGain", yawRateGain);
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
    double prev_ang_error = 0.0;
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
            // Check if we've passed the current waypoint (for smooth advancement)
            double dist_i_dx = waypoints_x[i] - current_x;
            double dist_i_dy = waypoints_y[i] - current_y;
            double distance_i = std::sqrt(dist_i_dx * dist_i_dx + dist_i_dy * dist_i_dy);

            // Advance waypoint index if we're close enough (smooth transition, don't stop)
            if (distance_i < 0.3 && i < (int)waypoints_x.size() - 1) {
                i++;
            }

            // Find lookahead point: farthest waypoint within lookahead distance
            // Start from current waypoint index and search forward
            pathPointID = std::max(pathPointID, i);
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

            // Angular error between path direction and vehicle yaw
            double dirDiff = pathDir - current_yaw;
            // Normalize to [-pi, pi]
            while (dirDiff > M_PI) dirDiff -= 2.0 * M_PI;
            while (dirDiff < -M_PI) dirDiff += 2.0 * M_PI;

            // Angular velocity controller: turn toward the path
            double vehicleYawRate = yawRateGain * dirDiff;
            // Limit angular velocity
            if (vehicleYawRate > max_angular_speed) vehicleYawRate = max_angular_speed;
            if (vehicleYawRate < -max_angular_speed) vehicleYawRate = -max_angular_speed;

            // Determine target forward speed based on:
            // 1. Distance to lookahead point (slow down when close)
            // 2. Angular error (slow down when turning sharply)
            double speed_reduction_factor = 1.0;
            
            // Reduce speed when close to lookahead point
            if (lookahead_dist < 1.0) {
                speed_reduction_factor = std::max(0.3, lookahead_dist / 1.0);
            }
            
            // Reduce speed when turning sharply (for stability)
            double abs_ang_error = std::abs(dirDiff);
            if (abs_ang_error > M_PI / 6.0) {  // More than 30 degrees
                speed_reduction_factor *= std::max(0.5, 1.0 - (abs_ang_error - M_PI/6.0) / (M_PI/3.0));
            }
            
            // Check if we're at the final waypoint
            bool is_final_waypoint = (i >= (int)waypoints_x.size() - 1);
            double final_waypoint_dist = 0.0;
            if (is_final_waypoint) {
                double final_dx = waypoints_x[waypoints_x.size() - 1] - current_x;
                double final_dy = waypoints_y[waypoints_y.size() - 1] - current_y;
                final_waypoint_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy);
                
                // Stop when very close to final waypoint
                if (final_waypoint_dist < 0.15) {
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
            cmd_vel.twist.linear.x = vehicleSpeed;  // Forward velocity in vehicle frame
            cmd_vel.twist.linear.y = 0.0;           // No lateral movement
            cmd_vel.twist.angular.z = vehicleYawRate; // Turn toward path

            // Publish derivatives for debugging/visualization
            auto now_der = std::chrono::system_clock::now();
            double dt_ang = std::chrono::duration_cast<std::chrono::duration<double>>(now_der - prev_ang_time).count();
            if (dt_ang <= 1e-6) dt_ang = dt;
            
            double ang_derivative = (dirDiff - prev_ang_error) / dt_ang;
            const double max_ang_derivative = 3.0;
            if (ang_derivative > max_ang_derivative) ang_derivative = max_ang_derivative;
            if (ang_derivative < -max_ang_derivative) ang_derivative = -max_ang_derivative;
            std_msgs::msg::Float32 ang_msg;
            ang_msg.data = static_cast<float>(ang_derivative);
            pubDerivativeAng->publish(ang_msg);

            // Linear derivative based on distance to lookahead point
            double lin_derivative = (lookahead_dist - prev_lin_error) / dt_ang;
            const double max_lin_derivative = 2.0;
            if (lin_derivative > max_lin_derivative) lin_derivative = max_lin_derivative;
            if (lin_derivative < -max_lin_derivative) lin_derivative = -max_lin_derivative;
            std_msgs::msg::Float32 lin_msg;
            lin_msg.data = static_cast<float>(lin_derivative);
            pubDerivativeLin->publish(lin_msg);

            prev_ang_error = dirDiff;
            prev_ang_time = now_der;
            prev_lin_error = lookahead_dist;
            prev_lin_time = now_der;

            pubSpeed->publish(cmd_vel);
            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
            pubGo2Request->publish(req);

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
