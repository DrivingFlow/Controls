#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

struct Goal {
    double x;
    double y;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("waypoint_sequencer");

    // Define the waypoint sequence (loops forever)
    std::vector<Goal> goals = {
        { 2.9,   15.74},
        {-7.8,   -3.86},
        { 6.3,   13.44},
        { 0.0,   -5.16}
    };

    int current_index = 0;
    std::mutex mtx;

    // State machine:
    //   PUBLISH_GOAL   → publish the current goal, then move to WAIT_FOR_MOVING
    //   WAIT_FOR_MOVING → wait for reached=false (controller started following the path)
    //   WAIT_FOR_REACHED → wait for reached=true (controller finished, idle)
    enum class State { PUBLISH_GOAL, WAIT_FOR_MOVING, WAIT_FOR_REACHED };
    State state = State::PUBLISH_GOAL;

    bool latest_reached = false;  // latest value from /waypoint/reached

    auto pub_end_pose = node->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);

    // Helper to publish the current goal
    auto publish_goal = [&]() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = node->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = goals[current_index].x;
        msg.pose.position.y = goals[current_index].y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.w = 1.0;  // neutral orientation
        pub_end_pose->publish(msg);

        RCLCPP_INFO(node->get_logger(), "Published goal %d/%zu: (%.2f, %.2f)",
                     current_index + 1, goals.size(),
                     goals[current_index].x, goals[current_index].y);
    };

    // Subscribe to /waypoint/reached (true = controller is idle / goal reached)
    auto sub_reached = node->create_subscription<std_msgs::msg::Bool>(
        "/waypoint/reached", 10,
        [&](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx);
            latest_reached = msg->data;
        });

    // Wait a moment for publishers/subscribers to connect
    RCLCPP_INFO(node->get_logger(), "Waypoint sequencer started. %zu goals in cycle.", goals.size());
    RCLCPP_INFO(node->get_logger(), "Waiting 2 seconds for connections...");

    rclcpp::Rate rate(10);  // 10 Hz
    auto start = node->get_clock()->now();
    while (rclcpp::ok() && (node->get_clock()->now() - start).seconds() < 2.0) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Main loop: state machine driven by /waypoint/reached
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        std::lock_guard<std::mutex> lock(mtx);

        switch (state) {
            case State::PUBLISH_GOAL:
                publish_goal();
                state = State::WAIT_FOR_MOVING;
                RCLCPP_INFO(node->get_logger(), "Waiting for controller to start moving (reached=false)...");
                break;

            case State::WAIT_FOR_MOVING:
                // Controller must report reached=false (it started following the new path)
                if (!latest_reached) {
                    state = State::WAIT_FOR_REACHED;
                    RCLCPP_INFO(node->get_logger(), "Controller is moving! Waiting for goal to be reached...");
                }
                break;

            case State::WAIT_FOR_REACHED:
                // Controller reports reached=true → goal achieved
                if (latest_reached) {
                    RCLCPP_INFO(node->get_logger(), "Goal %d/%zu reached!",
                                 current_index + 1, goals.size());
                    current_index = (current_index + 1) % static_cast<int>(goals.size());
                    state = State::PUBLISH_GOAL;
                }
                break;
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

