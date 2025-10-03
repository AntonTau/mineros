#include "my_first_behaviourtree/current_position.hpp"

BT::NodeStatus CurrentPosition::onTick(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& msg) {
    geometry_msgs::msg::PoseStamped current_position;
    current_position.pose = msg->pose;
    setOutput("current_position", current_position);
    return BT::NodeStatus::SUCCESS;
}