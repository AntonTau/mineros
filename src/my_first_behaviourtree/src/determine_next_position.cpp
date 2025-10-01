#include "my_first_behaviourtree/determine_next_position.hpp"

BT::NodeStatus DetermineNextPosition::tick() {
    BT::Expected<geometry_msgs::msg::PoseStamped> current_position = getInput<geometry_msgs::msg::PoseStamped>("current_position");
    if (!current_position) {
        RCLCPP_ERROR(rclcpp::get_logger("DetermineNextPosition"), "DetermineNextPosition: missing required input [current_position]: %s", current_position.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<int> step = getInput<int>("step");
    if (!step) {
        RCLCPP_ERROR(rclcpp::get_logger("DetermineNextPosition"), "DetermineNextPosition: missing required input [step]: %s", step.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    BT::Expected<int> direction = getInput<int>("direction");
    if (!direction) {
        RCLCPP_ERROR(rclcpp::get_logger("DetermineNextPosition"), "DetermineNextPosition: missing required input [direction]: %s", direction.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped target_point;
    target_point.pose.position.x = current_position.value().pose.position.x;
    target_point.pose.position.y = current_position.value().pose.position.y;
    target_point.pose.position.z = -1.0;
    if (direction.value() == 0) { // Move in x direction
        target_point.pose.position.x += step.value();
    } else if (direction.value() == 1) { // Move in y direction
        target_point.pose.position.y += step.value();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("DetermineNextPosition"), "DetermineNextPosition: invalid direction value: %d", direction.value());
        return BT::NodeStatus::FAILURE;
    }

    setOutput("target_point", target_point);
    return BT::NodeStatus::SUCCESS;
    
}