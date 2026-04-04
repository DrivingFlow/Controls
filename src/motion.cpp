#include <iostream>
#include <chrono>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cmath>
#include <mutex>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
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
std::mutex waypoints_mutex;  // Protect waypoints from concurrent access
bool waypoints_updated = false;  // Flag to indicate new waypoints received

// Emergency stop from wireless controller
std::mutex estop_mutex;  // Protect emergency stop flag
bool emergency_stop_active = false;  // Emergency stop flag
bool emergency_stop_latched = false;  // Stays true after estop until explicit resume
bool prev_estop_pressed = false;  // For edge-triggered logging
int estop_stop_count = 0;  // Counter for stop messages during e-stop activation
const int ESTOP_STOP_COUNT_MAX = 10;  // Number of stop messages to send before going silent

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

// Angular error thresholds (speed reduction bands)
const float ANGLE_45_DEG = M_PI / 4.0f;   // 45 degrees
const float ANGLE_30_DEG = M_PI / 6.0f;   // 30 degrees
const float ANGLE_15_DEG = M_PI / 12.0f;  // 15 degrees
const float ANGLE_5_DEG  = M_PI / 36.0f;  // 5 degrees

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
const float MAX_LATERAL_SPEED = 0.1f;  // m/s - max sideways velocity for direct lateral correction

// Proximity-based speed control
const float PROXIMITY_DISTANCE = 1.0f;  // meters
const float MIN_PROXIMITY_FACTOR = 0.3f;

// Lateral-error-based speed control
// Robot must slow down when far from the path to avoid hitting obstacles
const float LATERAL_ERROR_SPEED_THRESHOLD = 0.5f;  // start reducing speed above this cross-track error (m)
const float LATERAL_ERROR_SPEED_MAX = 2.0f;         // full reduction at this cross-track error (m)
const float MIN_LATERAL_ERROR_SPEED_FACTOR = 0.15f;  // minimum speed factor when far off-path

// Final waypoint thresholds (hysteresis: stop at STOP threshold, resume only beyond RESUME threshold)
const float FINAL_WAYPOINT_STOP_DISTANCE = 0.5f;   // meters - enter idle when closer than this
const float FINAL_WAYPOINT_RESUME_DISTANCE = 1.0f;  // meters - exit idle only when farther than this

// Derivative term limits (for visualization/debugging)
const float MAX_ANGULAR_DERIVATIVE = 3.0f;  // rad/s²
const float MAX_LINEAR_DERIVATIVE = 2.0f;  // m/s²

// Function to load waypoints from CSV file
// CSV format: x,y (one waypoint per line)
// Optional header row will be automatically skipped
bool loadWaypointsFromCSV(const std::string& filename, std::vector<float>& waypoints_x, std::vector<float>& waypoints_y) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open waypoints file: " << filename << std::endl;
        return false;
    }
    
    waypoints_x.clear();
    waypoints_y.clear();
    
    std::string line;
    int line_num = 0;
    bool header_skipped = false;
    
    while (std::getline(file, line)) {
        line_num++;
        
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        // Skip empty lines
        if (line.empty()) {
            continue;
        }
        
        // Try to parse as x,y
        std::istringstream iss(line);
        std::string x_str, y_str;
        
        if (std::getline(iss, x_str, ',')) {
            // Trim x_str
            x_str.erase(0, x_str.find_first_not_of(" \t"));
            x_str.erase(x_str.find_last_not_of(" \t") + 1);
            
            if (std::getline(iss, y_str)) {
                // Trim y_str
                y_str.erase(0, y_str.find_first_not_of(" \t"));
                y_str.erase(y_str.find_last_not_of(" \t") + 1);
                
                try {
                    float x = std::stof(x_str);
                    float y = std::stof(y_str);
                    waypoints_x.push_back(x);
                    waypoints_y.push_back(y);
                    header_skipped = true;  // If we successfully parsed, header was already skipped or doesn't exist
                } catch (const std::exception& e) {
                    // If parsing fails on first line, assume it's a header and skip it
                    if (!header_skipped && line_num == 1) {
                        header_skipped = true;
                        continue;
                    } else {
                        std::cerr << "Warning: Could not parse line " << line_num << ": " << line << std::endl;
                        continue;
                    }
                }
            } else {
                // Only one value on line - skip if it's the first line (likely header)
                if (!header_skipped && line_num == 1) {
                    header_skipped = true;
                    continue;
                } else {
                    std::cerr << "Warning: Line " << line_num << " does not have y coordinate: " << line << std::endl;
                    continue;
                }
            }
        }
    }
    
    file.close();
    
    if (waypoints_x.empty()) {
        std::cerr << "Error: No valid waypoints found in file: " << filename << std::endl;
        return false;
    }
    
    std::cout << "Successfully loaded " << waypoints_x.size() << " waypoints from " << filename << std::endl;
    return true;
}


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
    nh->declare_parameter<float>("maxDecel", 2.0f);           // Braking rate (m/s²), deliberately stronger than accel
    nh->declare_parameter<float>("max_linear_speed", 0.4f);
    nh->declare_parameter<float>("max_angular_speed", 0.35f);  // Reduced from 0.35 to prevent saturation
    nh->declare_parameter<float>("pointAcheivedDist", 0.1f); // Distance threshold to consider waypoint reached
    nh->declare_parameter<float>("minTurnSpeedFactor", 0.12f); // Keep some forward motion during large heading errors
    nh->declare_parameter<std::string>("waypoints_file", "");  // Path to CSV file with waypoints (x,y format)
    nh->declare_parameter<float>("lookaheadTimeGain", 0.8f);     // Seconds of lookahead per m/s of speed
    nh->declare_parameter<float>("maxLateralAccel", 0.5f);       // Max lateral accel for curvature speed limit (m/s²)
    nh->declare_parameter<float>("curvatureLookaheadDist", 5.0f); // How far ahead to scan for path curvature (m)
    nh->declare_parameter<float>("lateralIntegralGain", 0.20f);  // Ki for lateral cross-track error integral
    nh->declare_parameter<float>("lateralIntegralMax", 3.0f);    // Anti-windup clamp for lateral integral (m·s)
    nh->declare_parameter<std::string>("log_file", "");           // CSV log path (empty = disabled)
    nh->declare_parameter<int>("estop_key_exact", 4096);  // Observed controller value in this setup
    nh->declare_parameter<int>("estop_key_mask", 0);      // Optional bitmask mode; 0 disables mask check
    nh->declare_parameter<int>("resume_key_exact", 16386); // Resume motion only when estop is released

    // Local variables to hold parameter values (defaults mirrored in declare_parameter)
    float lookAheadDis = 0.8f;
    float yawRateGain = 0.6f;
    float yawDerivativeGain = 0.4f;
    float lateralErrorGain = 0.0f;  // DISABLED
    float yawErrorDeadband = 0.05f;
    float maxAngErrorForForward = 0.785f;  // ~45 degrees
    float maxAccel = 0.5f;
    float maxDecel = 2.0f;
    float max_linear_speed = 0.4f;
    float max_angular_speed = 0.35f;
    float pointAcheivedDist = 0.1f; // Distance threshold to consider waypoint reached
    float minTurnSpeedFactor = 0.12f; // Fraction of max speed to keep while turning
    float lookaheadTimeGain = 0.8f;
    float maxLateralAccel = 0.5f;
    float curvatureLookaheadDist = 5.0f;
    float lateralIntegralGain = 0.12f;
    float lateralIntegralMax = 3.0f;
    int estop_key_exact = 4096;
    int estop_key_mask = 0;
    int resume_key_exact = 16386;

    // Read params into local variables (overrides defaults above if provided)
    nh->get_parameter("lookAheadDis", lookAheadDis);
    nh->get_parameter("yawRateGain", yawRateGain);
    nh->get_parameter("yawDerivativeGain", yawDerivativeGain);
    nh->get_parameter("lateralErrorGain", lateralErrorGain);
    nh->get_parameter("yawErrorDeadband", yawErrorDeadband);
    nh->get_parameter("maxAngErrorForForward", maxAngErrorForForward);
    nh->get_parameter("maxAccel", maxAccel);
    nh->get_parameter("maxDecel", maxDecel);
    nh->get_parameter("max_linear_speed", max_linear_speed);
    nh->get_parameter("max_angular_speed", max_angular_speed);
    nh->get_parameter("minTurnSpeedFactor", minTurnSpeedFactor);
    nh->get_parameter("lookaheadTimeGain", lookaheadTimeGain);
    nh->get_parameter("maxLateralAccel", maxLateralAccel);
    nh->get_parameter("curvatureLookaheadDist", curvatureLookaheadDist);
    nh->get_parameter("lateralIntegralGain", lateralIntegralGain);
    nh->get_parameter("lateralIntegralMax", lateralIntegralMax);
    nh->get_parameter("estop_key_exact", estop_key_exact);
    nh->get_parameter("estop_key_mask", estop_key_mask);
    nh->get_parameter("resume_key_exact", resume_key_exact);
    
    // Load waypoints from CSV file if provided
    std::string waypoints_file;
    nh->get_parameter("waypoints_file", waypoints_file);
    bool use_csv_waypoints = false;
    
    if (!waypoints_file.empty()) {
        std::lock_guard<std::mutex> lock(waypoints_mutex);
        if (loadWaypointsFromCSV(waypoints_file, waypoints_x, waypoints_y)) {
            use_csv_waypoints = true;
            RCLCPP_INFO(nh->get_logger(), "Loaded %zu waypoints from CSV file: %s", waypoints_x.size(), waypoints_file.c_str());
        } else {
            RCLCPP_ERROR(nh->get_logger(), "Failed to load waypoints from CSV file: %s", waypoints_file.c_str());
        }
    }

    // CSV run logger for post-run diagnosis
    std::string log_file;
    nh->get_parameter("log_file", log_file);
    std::ofstream log_stream;
    if (!log_file.empty()) {
        log_stream.open(log_file);
        if (log_stream.is_open()) {
            log_stream << "t,x,y,yaw,vehicleSpeed,targetSpeed,dirDiff,lateral_error,lateral_integral,lookahead_dist,effectiveLookahead,pathPointID,curvature,speed_factor,mode\n";
            RCLCPP_INFO(nh->get_logger(), "Logging to CSV: %s", log_file.c_str());
        } else {
            RCLCPP_ERROR(nh->get_logger(), "Could not open log file: %s", log_file.c_str());
        }
    }

    // Subscribe to waypoints topic (only used if CSV file is not provided)
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_waypoints;
    if (!use_csv_waypoints) {
        sub_waypoints = nh->create_subscription<nav_msgs::msg::Path>(
            "/waypoints", rclcpp::QoS(10).best_effort(),
            [&](const nav_msgs::msg::Path::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(waypoints_mutex);
                
                // Clear existing waypoints
                waypoints_x.clear();
                waypoints_y.clear();
                
                // Extract waypoints from Path message
                for (const auto& pose_stamped : msg->poses) {
                    waypoints_x.push_back(pose_stamped.pose.position.x);
                    waypoints_y.push_back(pose_stamped.pose.position.y);
                }
                
                // Set flag to reset waypoint index
                waypoints_updated = true;
                
                RCLCPP_INFO(nh->get_logger(), "Received %zu new waypoints from /waypoints topic", waypoints_x.size());
            });
    } else {
        RCLCPP_INFO(nh->get_logger(), "Using waypoints from CSV file. ROS topic subscription disabled.");
    }

    auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
    // Publishers for the raw derivative terms (not multiplied by Kd).
    // Visualizer `debug_waypoints.py` subscribes to these topics
    auto pubDerivativeAng = nh->create_publisher<std_msgs::msg::Float32>("/pd_derivative/angular", 10);
    auto pubDerivativeLin = nh->create_publisher<std_msgs::msg::Float32>("/pd_derivative/linear", 10);
    // Publishers for waypoint indices for visualization
    auto pubLookaheadWaypoint = nh->create_publisher<std_msgs::msg::Int32>("/waypoint/lookahead", 10);
    auto pubCurrentWaypoint = nh->create_publisher<std_msgs::msg::Int32>("/waypoint/current", 10);
    // Publisher for controller status: true = reached final waypoint (idle), false = following path
    auto pubReached = nh->create_publisher<std_msgs::msg::Bool>("/waypoint/reached", 10);
    
    auto sub_odom = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/pcl_pose", 10,
        [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
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

    // Emergency stop callback shared by both possible topic names.
    auto wireless_cb = [&](const unitree_go::msg::WirelessController::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(estop_mutex);

        // Support either exact key matching or bitmask matching.
        const uint16_t keys = msg->keys;
        bool estop_pressed_exact = keys == static_cast<uint16_t>(estop_key_exact);
        bool estop_pressed_mask = false;
        if (estop_key_mask > 0) {
            const uint16_t mask = static_cast<uint16_t>(estop_key_mask);
            estop_pressed_mask = (keys & mask) == mask;
        }
        bool estop_pressed = estop_pressed_exact || estop_pressed_mask;
        bool resume_pressed = keys == static_cast<uint16_t>(resume_key_exact);

        // Debug: show exactly what this node receives from the wireless controller.
        // Throttled to avoid spamming at high publish rates.
        RCLCPP_INFO_THROTTLE(
            nh->get_logger(), *nh->get_clock(), 500,
            "Wireless rx: keys=%u (0x%04X) lx=%.3f ly=%.3f rx=%.3f ry=%.3f estop_pressed=%s estop_latched=%s resume_pressed=%s",
            static_cast<unsigned int>(keys),
            static_cast<unsigned int>(keys),
            static_cast<double>(msg->lx),
            static_cast<double>(msg->ly),
            static_cast<double>(msg->rx),
            static_cast<double>(msg->ry),
            estop_pressed ? "true" : "false",
            emergency_stop_latched ? "true" : "false",
            resume_pressed ? "true" : "false");

        if (estop_pressed) {
            if (!emergency_stop_latched) {
                emergency_stop_latched = true;
                estop_stop_count = 0;  // Reset stop counter for new e-stop activation
                RCLCPP_WARN(nh->get_logger(),
                            "EMERGENCY STOP ACTIVATED (latched)! keys=%u exact=%d mask=%d",
                            static_cast<unsigned int>(keys), estop_key_exact, estop_key_mask);
            }
            emergency_stop_active = true;
        } else {
            // Estop button released, but remain stopped while latched.
            if (prev_estop_pressed && emergency_stop_latched) {
                RCLCPP_WARN(nh->get_logger(),
                            "Emergency stop button released, but stop remains latched. Press resume key (%d) to continue.",
                            resume_key_exact);
            }

            if (emergency_stop_latched && resume_pressed) {
                emergency_stop_latched = false;
                emergency_stop_active = false;
                RCLCPP_INFO(nh->get_logger(),
                            "Emergency stop latch cleared by resume key. keys=%u",
                            static_cast<unsigned int>(keys));
            } else {
                emergency_stop_active = emergency_stop_latched;
            }
        }

        prev_estop_pressed = estop_pressed;
    };

    // Subscribe to both naming variants used in different stacks.
    auto sub_wireless = nh->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10, wireless_cb);
    auto sub_wireless_alt = nh->create_subscription<unitree_go::msg::WirelessController>(
        "/wireless_controller", 10, wireless_cb);

    RCLCPP_INFO(nh->get_logger(),
                "Wireless estop monitor active. Topics: /wirelesscontroller, /wireless_controller, estop_exact=%d, estop_mask=%d, resume_exact=%d",
                estop_key_exact, estop_key_mask, resume_key_exact);

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
    
    // Overshoot detection: track if we're moving away from target
    float prev_lookahead_dist = 0.0;
    int prev_pathPointID = 0;
    const float MOVING_AWAY_THRESHOLD = 0.10f;  // 10cm tolerance before triggering

    // Flag to track if we've reached the final waypoint (avoids log spam while idle)
    bool reached_final = false;

    // Mode hysteresis: prevents chattering between PP and TIP
    bool in_tip_mode = true;  // start in TIP until heading is aligned

    // Lateral error integral for persistent bias correction (e.g., rightward drift)
    float lateral_error_integral = 0.0f;

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
        
        // EMERGENCY STOP CHECK - MUST BE FIRST, OVERRIDES EVERYTHING
        bool estop_active = false;
        {
            std::lock_guard<std::mutex> lock(estop_mutex);
            estop_active = emergency_stop_active;
        }
        
        // If emergency stop is active, skip all control logic and just stop
        if (estop_active) {
            // Only publish stop commands for the first few iterations
            // After that, stop publishing to allow manual control
            if (estop_stop_count < ESTOP_STOP_COUNT_MAX) {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.linear.y = 0.0;
                cmd_vel.twist.angular.z = 0.0;
                pubSpeed->publish(cmd_vel);
                sport_req.StopMove(req);
                pubGo2Request->publish(req);
                estop_stop_count++;

                if (estop_stop_count >= ESTOP_STOP_COUNT_MAX) {
                    RCLCPP_INFO(nh->get_logger(),
                        "E-stop: Initial stop sequence complete. Robot can now be manually controlled. Press resume key to continue autonomous control.");
                }
            }
            // else: don't publish anything, allowing manual control

            // Publish waypoint indices for visualization even during emergency stop
            std_msgs::msg::Int32 lookahead_msg;
            lookahead_msg.data = pathPointID;
            pubLookaheadWaypoint->publish(lookahead_msg);

            std_msgs::msg::Int32 current_msg;
            current_msg.data = i;
            pubCurrentWaypoint->publish(current_msg);

            // Publish controller status
            std_msgs::msg::Bool reached_msg;
            reached_msg.data = reached_final;
            pubReached->publish(reached_msg);

            status = rclcpp::ok();
            rate.sleep();
            continue;  // Skip all control logic, go to next iteration
        }
        
        // Check if waypoints were updated and reset index if needed
        {
            std::lock_guard<std::mutex> lock(waypoints_mutex);
            if (waypoints_updated) {
                i = 0;
                pathPointID = 0;
                waypoints_updated = false;

                // Preserve speed if the new path direction is close to current heading.
                // This avoids the stop-accelerate cycle on every path planner update.
                if (waypoints_x.size() >= 2) {
                    float new_dir = std::atan2(waypoints_y[1] - waypoints_y[0],
                                               waypoints_x[1] - waypoints_x[0]);
                    float dir_change = new_dir - current_yaw;
                    while (dir_change > M_PI) dir_change -= 2.0 * M_PI;
                    while (dir_change < -M_PI) dir_change += 2.0 * M_PI;
                    if (std::abs(dir_change) > M_PI / 4.0f) {
                        vehicleSpeed = 0.0;
                        lateral_error_integral = 0.0f;
                    }
                    // else: keep current vehicleSpeed and integral
                } else {
                    vehicleSpeed = 0.0;
                    lateral_error_integral = 0.0f;
                }

                RCLCPP_INFO(nh->get_logger(), "Waypoints updated! Total: %zu, speed=%.2f", waypoints_x.size(), vehicleSpeed);
            }
        }
        
        auto current = std::chrono::system_clock::now();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning).count();

        if(seconds < INITIAL_WAIT_TIME){
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.linear.y = 0;
            cmd_vel.twist.angular.z = 0;
            pubSpeed->publish(cmd_vel);

            // Emergency stop already checked at top of loop, but double-check here
            if (estop_active) {
                sport_req.StopMove(req);
            } else {
                sport_req.Move(req, 0, 0, 0);
            }
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
        // Lock waypoints for the duration of this control iteration
        std::lock_guard<std::mutex> lock(waypoints_mutex);
        
        if (i < (int)waypoints_x.size()){
            // Velocity-scaled lookahead: look further ahead when moving faster
            float effectiveLookahead = lookAheadDis + vehicleSpeed * lookaheadTimeGain;

            // Lookahead: find the target waypoint ahead on the path.
            // Uses path distance (not Euclidean) to prevent "seeing through" detours,
            // but only when the robot is near the path. When far from the path,
            // target the current waypoint to avoid runaway index advancement.
            {
                float dist_to_wp_i_dx = waypoints_x[i] - current_x;
                float dist_to_wp_i_dy = waypoints_y[i] - current_y;
                float dist_to_wp_i = std::sqrt(dist_to_wp_i_dx * dist_to_wp_i_dx + dist_to_wp_i_dy * dist_to_wp_i_dy);

                if (dist_to_wp_i > effectiveLookahead * LOOKAHEAD_DISTANCE_MULTIPLIER) {
                    // Robot is far from current waypoint -- just target it, don't look further
                    pathPointID = i;
                } else {
                    // Robot is near the path -- use path-distance lookahead
                    float path_dist = 0.0f;
                    int bestLookaheadID = i;
                    for (int j = i; j < (int)waypoints_x.size(); j++) {
                        if (j > i) {
                            float seg_dx = waypoints_x[j] - waypoints_x[j-1];
                            float seg_dy = waypoints_y[j] - waypoints_y[j-1];
                            path_dist += std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
                        }
                        if (path_dist <= effectiveLookahead) {
                            bestLookaheadID = j;
                        } else {
                            break;
                        }
                    }
                    pathPointID = bestLookaheadID;
                }
            }

            // Advance waypoint index `i` based on multiple criteria:
            // 1. Close enough to current waypoint (proximity-based)
            // 2. Lookahead point is significantly ahead (we've likely passed the waypoint)
            float dist_i_dx = waypoints_x[i] - current_x;
            float dist_i_dy = waypoints_y[i] - current_y;

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

            // Calculate lateral (cross-track) error to the NEAREST path segment
            // in a local window around the current index.  Using only segment
            // [i, i+1] causes sign-flip discontinuities when i advances on a
            // curved path, producing the zigzag behaviour.
            float lateral_error = 0.0;
            if (i < (int)waypoints_x.size() - 1) {
                const int LATERAL_WINDOW = 3;  // check segments [i-W .. i+W]
                int seg_lo = std::max(0, i - LATERAL_WINDOW);
                int seg_hi = std::min((int)waypoints_x.size() - 2, i + LATERAL_WINDOW);

                float best_dist_sq = 1e9f;
                float best_lateral_error = 0.0f;

                for (int si = seg_lo; si <= seg_hi; si++) {
                    float s_dx = waypoints_x[si+1] - waypoints_x[si];
                    float s_dy = waypoints_y[si+1] - waypoints_y[si];
                    float s_len_sq = s_dx * s_dx + s_dy * s_dy;
                    if (s_len_sq < MIN_PATH_LENGTH * MIN_PATH_LENGTH) continue;

                    float r_dx = current_x - waypoints_x[si];
                    float r_dy = current_y - waypoints_y[si];

                    float proj = (r_dx * s_dx + r_dy * s_dy) / s_len_sq;
                    proj = std::max(0.0f, std::min(1.0f, proj));

                    float cx = waypoints_x[si] + proj * s_dx;
                    float cy = waypoints_y[si] + proj * s_dy;

                    float ldx = current_x - cx;
                    float ldy = current_y - cy;
                    float d_sq = ldx * ldx + ldy * ldy;

                    if (d_sq < best_dist_sq) {
                        best_dist_sq = d_sq;
                        float dist = std::sqrt(d_sq);
                        float cross = s_dx * ldy - s_dy * ldx;
                        best_lateral_error = (cross < 0) ? -dist : dist;
                    }
                }
                lateral_error = best_lateral_error;
            } else {
                float final_dx = waypoints_x[i] - current_x;
                float final_dy = waypoints_y[i] - current_y;
                lateral_error = std::sqrt(final_dx * final_dx + final_dy * final_dy);
            }

            // Accumulate lateral cross-track error integral (only on intermediate segments)
            // Gentle decay when stopped (e-stop, TIP) so stale integral doesn't corrupt recovery
            if (vehicleSpeed < 0.05f) {
                lateral_error_integral *= 0.995f;
            } else if (i < (int)waypoints_x.size() - 1) {
                lateral_error_integral += lateral_error * CONTROL_LOOP_DT;
                lateral_error_integral = std::max(-lateralIntegralMax, std::min(lateralIntegralMax, lateral_error_integral));
            }

            // === LATERAL ERROR CORRECTION VIA HEADING ADJUSTMENT (Stanley-style + Integral) ===
            // Proportional: atan2(Kp * lateral_error, lookahead) adjusts heading toward path
            // Integral: Ki * accumulated_lateral_error corrects persistent hardware bias
            float heading_correction = std::atan2(lateralErrorGain * lateral_error, 
                                                   std::max(lookahead_dist, MIN_LOOKAHEAD_FOR_LATERAL));
            
            const float MAX_HEADING_CORRECTION = 0.35f;  // radians (~20 degrees)
            heading_correction = std::max(-MAX_HEADING_CORRECTION, std::min(MAX_HEADING_CORRECTION, heading_correction));
            
            float integral_heading_correction = lateralIntegralGain * lateral_error_integral;
            const float MAX_INTEGRAL_CORRECTION = 0.25f;  // radians (~14 degrees)
            integral_heading_correction = std::max(-MAX_INTEGRAL_CORRECTION, std::min(MAX_INTEGRAL_CORRECTION, integral_heading_correction));
            
            // Subtract corrections: positive lateral_error (left of path) → turn right
            float adjusted_pathDir = pathDir - heading_correction - integral_heading_correction;
            
            // Angular error between ADJUSTED path direction and vehicle yaw
            float dirDiff = adjusted_pathDir - current_yaw;
            // Normalize to [-pi, pi]
            while (dirDiff > M_PI) dirDiff -= 2.0 * M_PI;
            while (dirDiff < -M_PI) dirDiff += 2.0 * M_PI;

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
            
            // Store lateral correction for debug logging
            float lateral_correction = heading_correction;
            
            // === PURE PURSUIT CURVATURE CALCULATION ===
            // This is the key to smooth arcs instead of zig-zag!
            // curvature = 2 * sin(alpha) / L, where alpha is angle to target, L is distance
            float pure_pursuit_curvature = 2.0f * std::sin(dirDiff) / std::max(lookahead_dist, 0.1f);
            
            // Adaptive angular speed limits
            float max_angular_left = max_angular_speed;
            float max_angular_right = max_angular_speed + RIGHT_TURN_COMPENSATION;
            
            // PLACEHOLDER: vehicleYawRate will be calculated after we know vehicleSpeed
            // For now, calculate what it would be in turn-in-place mode (P controller)
            float turnInPlaceYawRate = yawRateGain * dirDiffFiltered - yawDerivativeGain * error_derivative;
            
            // Limit turn-in-place yaw rate
            if (turnInPlaceYawRate > max_angular_left) turnInPlaceYawRate = max_angular_left;
            if (turnInPlaceYawRate < -max_angular_right) turnInPlaceYawRate = -max_angular_right;
            
            // vehicleYawRate will be set after speed calculation (see below)
            float vehicleYawRate = 0.0f;

            // === CURVATURE-BASED SPEED LIMITING ===
            // Scan upcoming waypoints, compute max curvature, limit speed to v = sqrt(a_lat / k)
            // Uses wide-baseline Menger curvature: for each point B, pick A and C at least
            // MIN_CURVATURE_BASELINE apart to filter noise from closely-spaced waypoints.
            float max_upcoming_curvature = 0.0f;
            {
                const float MIN_CURVATURE_BASELINE = 0.4f;
                float scan_dist = 0.0f;
                int n_wp = (int)waypoints_x.size();
                for (int ci = std::max(i, 1); ci < n_wp - 1 && scan_dist < curvatureLookaheadDist; ci++) {
                    // Find point A: walk backward from ci until >= MIN_CURVATURE_BASELINE
                    int ai = ci - 1;
                    float dist_back = 0.0f;
                    while (ai > 0) {
                        float dx_b = waypoints_x[ai] - waypoints_x[ai + 1];
                        float dy_b = waypoints_y[ai] - waypoints_y[ai + 1];
                        dist_back += std::sqrt(dx_b * dx_b + dy_b * dy_b);
                        if (dist_back >= MIN_CURVATURE_BASELINE) break;
                        ai--;
                    }

                    // Find point C: walk forward from ci until >= MIN_CURVATURE_BASELINE
                    int cci = ci + 1;
                    float dist_fwd = 0.0f;
                    while (cci < n_wp - 1) {
                        float dx_f = waypoints_x[cci] - waypoints_x[cci - 1];
                        float dy_f = waypoints_y[cci] - waypoints_y[cci - 1];
                        dist_fwd += std::sqrt(dx_f * dx_f + dy_f * dy_f);
                        if (dist_fwd >= MIN_CURVATURE_BASELINE) break;
                        cci++;
                    }

                    float ax_c = waypoints_x[ai], ay_c = waypoints_y[ai];
                    float bx_c = waypoints_x[ci], by_c = waypoints_y[ci];
                    float cx_c = waypoints_x[cci], cy_c = waypoints_y[cci];

                    float ab = std::sqrt((bx_c - ax_c) * (bx_c - ax_c) + (by_c - ay_c) * (by_c - ay_c));
                    float bc = std::sqrt((cx_c - bx_c) * (cx_c - bx_c) + (cy_c - by_c) * (cy_c - by_c));
                    float ac = std::sqrt((cx_c - ax_c) * (cx_c - ax_c) + (cy_c - ay_c) * (cy_c - ay_c));

                    float cross = std::abs((bx_c - ax_c) * (cy_c - ay_c) - (by_c - ay_c) * (cx_c - ax_c));
                    float denom = ab * bc * ac;
                    if (denom > 1e-6f) {
                        float k = 2.0f * cross / denom;
                        if (k > max_upcoming_curvature) max_upcoming_curvature = k;
                    }

                    // Advance scan distance using original consecutive spacing
                    float seg_dx = waypoints_x[ci + 1] - waypoints_x[ci];
                    float seg_dy = waypoints_y[ci + 1] - waypoints_y[ci];
                    scan_dist += std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
                }
            }

            float curvature_speed_limit = max_linear_speed;
            if (max_upcoming_curvature > 1e-4f) {
                curvature_speed_limit = std::sqrt(maxLateralAccel / max_upcoming_curvature);
                curvature_speed_limit = std::max(curvature_speed_limit, max_linear_speed * 0.1f);
            }

            float speed_reduction_factor = 1.0;

            // Apply curvature speed cap before other reductions
            if (curvature_speed_limit < max_linear_speed) {
                speed_reduction_factor = curvature_speed_limit / max_linear_speed;
            }

            // Stop forward motion when angular error is too large
            // This prevents wide arcs and dangerous overshoot
            float abs_ang_error = std::abs(dirDiff);
            if (abs_ang_error > maxAngErrorForForward) {
                // Error > threshold (default 45 degrees): keep a small forward speed to avoid stop-turn-go
                speed_reduction_factor = minTurnSpeedFactor;
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

            // Reduce speed based on lateral (cross-track) error
            // This is critical: without this, the robot can be meters off-path at full speed
            float abs_lateral_error = std::abs(lateral_error);
            if (abs_lateral_error > LATERAL_ERROR_SPEED_THRESHOLD) {
                float lat_fraction = (abs_lateral_error - LATERAL_ERROR_SPEED_THRESHOLD)
                                   / (LATERAL_ERROR_SPEED_MAX - LATERAL_ERROR_SPEED_THRESHOLD);
                lat_fraction = std::min(lat_fraction, 1.0f);
                float lat_error_speed_factor = 1.0f - lat_fraction * (1.0f - MIN_LATERAL_ERROR_SPEED_FACTOR);
                speed_reduction_factor *= lat_error_speed_factor;
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
            
            // OVERSHOOT DETECTION: If distance to lookahead is INCREASING, we're moving away - STOP!
            // Only trigger when the lookahead target hasn't changed — a waypoint index
            // jump naturally increases the distance and is not a real overshoot.
            bool moving_away = false;
            if (prev_lookahead_dist > 0.0f
                && pathPointID == prev_pathPointID
                && lookahead_dist > prev_lookahead_dist + MOVING_AWAY_THRESHOLD) {
                moving_away = true;
                speed_reduction_factor = 0.0f;
            }
            prev_lookahead_dist = lookahead_dist;
            prev_pathPointID = pathPointID;
            
            // ALWAYS check distance to the LAST waypoint, regardless of current index
            float final_dx = waypoints_x[waypoints_x.size() - 1] - current_x;
            float final_dy = waypoints_y[waypoints_y.size() - 1] - current_y;
            float final_waypoint_dist = std::sqrt(final_dx * final_dx + final_dy * final_dy);
            bool should_stop_completely = false;
            
            // Hysteresis: once idle, only resume if final waypoint is beyond RESUME distance
            if (reached_final) {
                // Already idle — stay idle unless final waypoint is far enough away
                if (final_waypoint_dist < FINAL_WAYPOINT_RESUME_DISTANCE) {
                    should_stop_completely = true;
                    targetSpeed = 0.0;
                } else {
                    // New waypoints are far enough — exit idle
                    reached_final = false;
                    targetSpeed = max_linear_speed * speed_reduction_factor;
                }
            } else if (final_waypoint_dist < FINAL_WAYPOINT_STOP_DISTANCE) {
                // Not idle yet, but close enough to final destination → enter idle
                should_stop_completely = true;
                targetSpeed = 0.0;
            } else {
                targetSpeed = max_linear_speed * speed_reduction_factor;
            }

            // Ramp vehicleSpeed toward targetSpeed with acceleration limit
            // EXCEPTION: If we need to STOP due to overshoot or final waypoint, stop immediately
            float dt = CONTROL_LOOP_DT;
            if (moving_away || should_stop_completely) {
                vehicleSpeed = 0.0f;
                if (moving_away) {
                    lateral_error_integral *= 0.5f;
                }
            } else {
                if (vehicleSpeed < targetSpeed) {
                    float maxUp = maxAccel * dt;
                    vehicleSpeed = std::min(targetSpeed, vehicleSpeed + maxUp);
                } else if (vehicleSpeed > targetSpeed) {
                    float maxDown = maxDecel * dt;
                    vehicleSpeed = std::max(targetSpeed, vehicleSpeed - maxDown);
                }
            }
            
            // === HYBRID CONTROL: Choose between Turn-in-Place and Pure Pursuit ===
            // Hysteresis band prevents rapid mode switching (chattering)
            const float TIP_ENTER_THRESHOLD = 0.52f;  // radians (~30 deg) — enter TIP above this
            const float TIP_EXIT_THRESHOLD  = 0.26f;  // radians (~15 deg) — exit TIP below this
            
            float abs_ang_error_for_mode = std::abs(dirDiff);
            
            if (in_tip_mode) {
                if (vehicleSpeed >= 0.01f && abs_ang_error_for_mode < TIP_EXIT_THRESHOLD) {
                    in_tip_mode = false;
                }
            } else {
                if (vehicleSpeed < 0.01f || abs_ang_error_for_mode > TIP_ENTER_THRESHOLD) {
                    in_tip_mode = true;
                }
            }
            
            if (in_tip_mode) {
                vehicleYawRate = turnInPlaceYawRate;
            } else {
                vehicleYawRate = vehicleSpeed * pure_pursuit_curvature;
                
                if (vehicleYawRate > max_angular_left) vehicleYawRate = max_angular_left;
                if (vehicleYawRate < -max_angular_right) vehicleYawRate = -max_angular_right;
            }
            
            // === ANGULAR OVERSHOOT DETECTION ===
            // If angular error is INCREASING while we're turning, we're turning the wrong way - STOP!
            // Same sign of error and derivative means overshooting:
            //   - Positive error (target left) + positive derivative (error growing) = overshoot
            //   - Negative error (target right) + negative derivative (error growing) = overshoot
            const float DERIVATIVE_THRESHOLD = 0.05f;  // Minimum derivative to trigger (reduces noise)
            bool angular_overshoot = (dirDiff * error_derivative > 0) && (std::abs(error_derivative) > DERIVATIVE_THRESHOLD);
            
            if (angular_overshoot) {
                // STOP turning completely - we're going the wrong way!
                vehicleYawRate = 0.0f;
            }

            // === LATERAL VELOCITY CORRECTION ===
            // Use linear.y to directly slide toward the path (holonomic motion)
            // This is much more effective than trying to correct through heading changes!
            // lateral_error > 0 means robot is LEFT of path → need to move RIGHT (negative Y)
            float lateral_velocity = -lateralErrorGain * lateral_error;
            
            // Clamp to max lateral speed (gentle sideways motion)
            if (lateral_velocity > MAX_LATERAL_SPEED) lateral_velocity = MAX_LATERAL_SPEED;
            if (lateral_velocity < -MAX_LATERAL_SPEED) lateral_velocity = -MAX_LATERAL_SPEED;
            
            // Forward + Lateral + Angular control
            // If we should stop completely at final waypoint, stop all motion
            if (should_stop_completely) {
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.linear.y = 0.0;
                cmd_vel.twist.angular.z = 0.0;
                vehicleSpeed = 0.0;  // Reset speed for next iteration
                vehicleYawRate = 0.0;
            } else {
                cmd_vel.twist.linear.x = vehicleSpeed;      // Forward velocity in vehicle frame
                cmd_vel.twist.linear.y = lateral_velocity;  // Sideways correction toward path
                cmd_vel.twist.angular.z = vehicleYawRate;   // Face the path direction
            }

            // CSV log row (every iteration, lightweight)
            if (log_stream.is_open()) {
                float t_sec = std::chrono::duration_cast<std::chrono::duration<float>>(
                    std::chrono::system_clock::now() - beginning).count();
                const char* mode = in_tip_mode ? "TIP" : "PP";
                log_stream << t_sec << ',' << current_x << ',' << current_y << ',' << current_yaw
                           << ',' << vehicleSpeed << ',' << targetSpeed << ',' << dirDiff
                           << ',' << lateral_error << ',' << lateral_error_integral
                           << ',' << lookahead_dist << ',' << effectiveLookahead << ',' << pathPointID
                           << ',' << max_upcoming_curvature << ',' << speed_reduction_factor
                           << ',' << mode << '\n';
            }

            // Publish derivatives for debugging/visualization
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

            // Emergency stop already checked at top of loop, but ensure we don't publish Move commands
            pubSpeed->publish(cmd_vel);
            
            // EMERGENCY STOP OVERRIDE: If emergency stop is active, ONLY publish StopMove
            // (This should never happen since we check at top of loop, but safety check)
            if (estop_active) {
                // Emergency stop active - stop all movement immediately
                sport_req.StopMove(req);
                pubGo2Request->publish(req);
            } else if (should_stop_completely) {
                // Use StopMove to halt the locomotion gait entirely (no jitter)
                // Move(0,0,0) keeps the gait controller active → causes weight-shifting jitter
                sport_req.StopMove(req);
                pubGo2Request->publish(req);
            } else {
                sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
                pubGo2Request->publish(req);
            }

            // If we've reached the final waypoint, log once and stay idle
            if (should_stop_completely && !reached_final) {
                reached_final = true;
                RCLCPP_INFO(nh->get_logger(), "Final waypoint reached! Distance: %.3f m. Robot idle, waiting for new waypoints.", 
                           final_waypoint_dist);
                std::cout << "=== FINAL WAYPOINT REACHED ===" << std::endl;
                std::cout << "Final position: (" << current_x << ", " << current_y << ")" << std::endl;
                std::cout << "Target position: (" << waypoints_x[waypoints_x.size() - 1] << ", " 
                          << waypoints_y[waypoints_y.size() - 1] << ")" << std::endl;
                std::cout << "Distance to final waypoint: " << final_waypoint_dist << " m" << std::endl;
                std::cout << "Robot idle. Waiting for new waypoints on /waypoints..." << std::endl;
            }

            if(seconds % STATUS_UPDATE_INTERVAL == 0){
                // Check emergency stop status for logging
                bool estop_active_log = false;
                {
                    std::lock_guard<std::mutex> lock(estop_mutex);
                    estop_active_log = emergency_stop_active;
                }
                
                if (estop_active_log) {
                    std::cout << "*** EMERGENCY STOP ACTIVE (LATCHED) *** All movement stopped. Waiting for resume key "
                              << resume_key_exact << " with estop released..." << std::endl;
                } else if (reached_final) {
                    // Minimal output when idle — just confirm we're stopped
                    std::cout << "=== IDLE === Cmd: 0,0,0 | Pose: (" << current_x << ", " << current_y 
                              << ") | Dist to final: " << final_waypoint_dist << " m | Waiting for new waypoints (resume > " 
                              << FINAL_WAYPOINT_RESUME_DISTANCE << " m) ===" << std::endl;
                } else {
                    std::cout << "=== Status Update ===" << std::endl;
                    std::cout << "Localization rate: " << odom_rate << " Hz (received " << odom_count << " messages)" << std::endl;
                    std::cout << "Current waypoint index: " << i << " / " << waypoints_x.size() << std::endl;
                    std::cout << "Lookahead point index: " << pathPointID << std::endl;
                    std::cout << "Current pose: (" << current_x << ", " << current_y << "), yaw=" << current_yaw << std::endl;
                    std::cout << "Target waypoint: (" << waypoints_x[i] << ", " << waypoints_y[i] << ")" << std::endl;
                    std::cout << "Lookahead point: (" << tx << ", " << ty << "), distance=" << lookahead_dist << std::endl;
                    std::cout << "Angular error: " << dirDiff << " rad (" << dirDiff * 180.0 / M_PI << " deg)" << std::endl;
                    std::cout << "Lateral error: " << lateral_error << " m (+ = left of path, - = right)" << std::endl;
                    std::cout << "Lateral integral: " << lateral_error_integral << " m·s (correction: " << integral_heading_correction * 180.0 / M_PI << " deg)" << std::endl;
                    std::cout << "Heading correction: " << lateral_correction << " rad (" << lateral_correction * 180.0 / M_PI << " deg)" << std::endl;
                    std::cout << "Cmd: linear.x=" << cmd_vel.twist.linear.x << ", linear.y=" << cmd_vel.twist.linear.y << ", angular.z=" << cmd_vel.twist.angular.z << std::endl;
                    std::cout << "Control mode: " << (in_tip_mode ? "TURN-IN-PLACE" : "PURE PURSUIT") << std::endl;
                    std::cout << "Pure pursuit curvature: " << pure_pursuit_curvature << " m^-1" << std::endl;
                    std::cout << "Upcoming path curvature: " << max_upcoming_curvature << " m^-1 (speed limit: " << curvature_speed_limit << " m/s)" << std::endl;
                    std::cout << "Effective lookahead: " << effectiveLookahead << " m (base: " << lookAheadDis << " + speed*" << lookaheadTimeGain << ")" << std::endl;
                    if (moving_away) {
                        std::cout << "*** OVERSHOOT DETECTED: Moving away from target! ***" << std::endl;
                    }
                    if (angular_overshoot) {
                        std::cout << "*** ANGULAR OVERSHOOT: Error increasing, stopped turning! ***" << std::endl;
                    }
                    std::cout << "Final waypoint distance: " << final_waypoint_dist << " m" << std::endl;
                }
            }
        } else {
            // All waypoints reached (or none loaded) - stay idle and wait
            cmd_vel.twist.linear.x = 0.0;
            cmd_vel.twist.linear.y = 0.0;
            cmd_vel.twist.angular.z = 0.0;
            pubSpeed->publish(cmd_vel);
            
            // Emergency stop already checked at top of loop
            // Always use StopMove when idle (or emergency stop)
            sport_req.StopMove(req);
            pubGo2Request->publish(req);
            
            if (!waypoints_x.empty()) {
                std_msgs::msg::Int32 lookahead_msg;
                lookahead_msg.data = (int)waypoints_x.size() - 1;
                pubLookaheadWaypoint->publish(lookahead_msg);
                
                std_msgs::msg::Int32 current_msg;
                current_msg.data = (int)waypoints_x.size() - 1;
                pubCurrentWaypoint->publish(current_msg);
            }
            
            if (!reached_final) {
                reached_final = true;
                if (waypoints_x.empty()) {
                    RCLCPP_INFO(nh->get_logger(), "No waypoints loaded. Waiting for waypoints on /waypoints...");
                } else {
                    RCLCPP_INFO(nh->get_logger(), "All waypoints reached! Waiting for new waypoints on /waypoints...");
                }
            }
        }
    }
        
        // Publish controller status every iteration
        // true = done (idle, waiting for new path), false = following waypoints
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = reached_final;
        pubReached->publish(reached_msg);

        status = rclcpp::ok();
        rate.sleep();
    }
}
