#include "my_first_behaviourtree/move_to.hpp"

bool MoveTo::setRequest(Request::SharedPtr& request) {
    BT::Expected<geometry_msgs::msg::Point> msg = getInput<geometry_msgs::msg::Point>("target_point");
    if (!msg) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveTo"), "MoveTo: missing required input [target_point]: %s", msg.error().c_str());
        return false;
    }
    request->pose.pose.position = msg.value();
    setOutput("currently_moving", true);
    return true;
}

BT::NodeStatus MoveTo::onResponseReceived(const Response::SharedPtr& response) {
    setOutput("currently_moving", false);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus MoveTo::onFailure(BT::ServiceNodeErrorCode error) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveTo"), "MoveTo: Service call failed: %d", error);
    return BT::NodeStatus::FAILURE;
}