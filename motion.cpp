#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

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

int robot_x = 0;
int robot_y = 0;




int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("mover");

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

    
    double ang_vel = 0.8;
    double lin_vel = 0.4;
    int i =0;
    int pathPointID = 0;
    double lookAheadDis = 1.0; // meters
    double yawRateGain = 1.5;  // yaw-rate proportional gain
    double maxAccel = 0.5;     // m/s^2 for speed ramping
    double vehicleSpeed = 0.0; // current commanded speed (m/s)
    double targetSpeed = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double theta = 0.0;
    double delta_theta = 0.0;
    
    // PD gains for angular control (tuned to avoid saturation)
    double Kp_ang = 0.3;  // reduced proportional gain for heading
    double Kd_ang = 0.5;  // increased damping to reduce overshoot
    double prev_ang_error = 0.0;
    auto prev_ang_time = std::chrono::system_clock::now();
    
    // PD gains for linear control (tuned to avoid saturation)
    double Kp_lin = 0.25; // reduced proportional gain for distance
    double Kd_lin = 0.5;  // increased damping to reduce overshoot
    double prev_lin_error = 0.0;
    auto prev_lin_time = std::chrono::system_clock::now();
    
    // Speed limits (back to conservative values to avoid overshoot)
    const double max_linear_speed = 0.4;   // reduced to prevent overshoot
    const double max_angular_speed = 0.35; // reduced for smoother approach
    
    double linear_speed = 0.5;
    double z_vel = 0;
    double x_vel = 0;

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
        if ( i < waypoints_x.size()){
            // get localization output
            // convert orientation to yaw
            // TODO: fix the double start button?
            // distance to the current (i) waypoint
            double dist_i_dx = waypoints_x[i] - current_x;
            double dist_i_dy = waypoints_y[i] - current_y;
            double distance_i = std::sqrt(dist_i_dx * dist_i_dx + dist_i_dy * dist_i_dy);

            // Advance pathPointID until lookahead distance is ahead
            pathPointID = std::max(pathPointID, i);
            while (pathPointID < (int)waypoints_x.size() - 1) {
                double pdx = waypoints_x[pathPointID] - current_x;
                double pdy = waypoints_y[pathPointID] - current_y;
                double pdist = std::sqrt(pdx * pdx + pdy * pdy);
                if (pdist < lookAheadDis) pathPointID++; else break;
            }

            // target lookahead point
            double tx = waypoints_x[pathPointID];
            double ty = waypoints_y[pathPointID];
            dx = tx - current_x;
            dy = ty - current_y;
            double pathDir = atan2(dy, dx);

            // angular error (dirDiff) between path direction and vehicle yaw
            double dirDiff = pathDir - current_yaw;
            while (dirDiff > M_PI) dirDiff -= 2.0 * M_PI;
            while (dirDiff < -M_PI) dirDiff += 2.0 * M_PI;

            // Continuous yaw-rate controller
            double vehicleYawRate = -yawRateGain * dirDiff;
            if (vehicleYawRate > max_angular_speed) vehicleYawRate = max_angular_speed;
            if (vehicleYawRate < -max_angular_speed) vehicleYawRate = -max_angular_speed;

            // determine target forward speed based on distance to current waypoint (i)
            if (distance_i < 0.2) {
                // reached waypoint -> stop and advance
                i++;
                vehicleSpeed = 0.0;
                targetSpeed = 0.0;

                // Publish zero derivative when stopped / waypoint reached
                std_msgs::msg::Float32 stop_deriv_msg;
                stop_deriv_msg.data = 0.0f;
                pubDerivativeAng->publish(stop_deriv_msg);
                pubDerivativeLin->publish(stop_deriv_msg);

                // Reset prev states
                prev_ang_error = 0.0;
                prev_ang_time = std::chrono::system_clock::now();
                prev_lin_error = 0.0;
                prev_lin_time = std::chrono::system_clock::now();
            } else {
                if (distance_i < 1.0) {
                    targetSpeed = max_linear_speed * (distance_i / 1.0); // slow down near target
                } else {
                    targetSpeed = max_linear_speed;
                }

                // ramp vehicleSpeed toward targetSpeed
                double dt = 1.0 / 100.0; // loop dt (rate 100)
                double maxDelta = maxAccel * dt;
                if (vehicleSpeed < targetSpeed) vehicleSpeed = std::min(targetSpeed, vehicleSpeed + maxDelta);
                else if (vehicleSpeed > targetSpeed) vehicleSpeed = std::max(targetSpeed, vehicleSpeed - maxDelta);

                // compute commanded velocities in vehicle frame for lookahead direction
                cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
                cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
                cmd_vel.twist.angular.z = vehicleYawRate;

                // publish derivatives: angular derivative = d(dirDiff)/dt, linear derivative = d(distance)/dt
                auto now_der = std::chrono::system_clock::now();
                double dt_ang = std::chrono::duration_cast<std::chrono::duration<double>>(now_der - prev_ang_time).count();
                if (dt_ang <= 1e-6) dt_ang = dt;
                double ang_derivative = (dirDiff - prev_ang_error) / dt_ang;
                const double max_ang_derivative = 3.0;
                if (ang_derivative > max_ang_derivative) ang_derivative = max_ang_derivative;
                if (ang_derivative < -max_ang_derivative) ang_derivative = -max_ang_derivative;
                std_msgs::msg::Float32 ang_msg; ang_msg.data = static_cast<float>(ang_derivative); pubDerivativeAng->publish(ang_msg);

                double lin_derivative = (distance_i - prev_lin_error) / dt_ang;
                const double max_lin_derivative = 2.0;
                if (lin_derivative > max_lin_derivative) lin_derivative = max_lin_derivative;
                if (lin_derivative < -max_lin_derivative) lin_derivative = -max_lin_derivative;
                std_msgs::msg::Float32 lin_msg; lin_msg.data = static_cast<float>(lin_derivative); pubDerivativeLin->publish(lin_msg);

                prev_ang_error = dirDiff;
                prev_ang_time = now_der;
                prev_lin_error = distance_i;
                prev_lin_time = now_der;
            }

            pubSpeed->publish(cmd_vel);

            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
            pubGo2Request->publish(req);

            if(seconds % 1 == 0){
                std::cout << "=== Status Update ===" << std::endl;
                std::cout << "Localization rate: " << odom_rate << " Hz (received " << odom_count << " messages)" << std::endl;
                std::cout << "Waypoint " << i << ": target=(" << waypoints_x[i] << ", " << waypoints_y[i] 
                        << "), current=(" << current_x << ", " << current_y << "), yaw=" << current_yaw << "\n";
                // std::cout << "dx: " << dx << ", dy: " << dy << ", delta_theta: " << delta_theta << "\n";
                // std::cout << "cmd_vel - linear.x: " << cmd_vel.twist.linear.x << ", linear.y: " << cmd_vel.twist.linear.y << ", angular.z: " << cmd_vel.twist.angular.z << "\n";
            }
        } else {
            std::cout << "All waypoints reached!" << std::endl;
            break;
        }
    }
        
        status = rclcpp::ok();
        rate.sleep();
    }
}
