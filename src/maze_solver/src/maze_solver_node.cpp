// source install/setup.bash
// colcon build --packages-select maze_solver
// ros2 run maze_solver maze_solver_node

#include <chrono>
#include <cstdio>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mineros_inter/srv/move_to.hpp"
#include "rclcpp/rclcpp.hpp"

class MazeSolverNode : public rclcpp::Node {
public:
  MazeSolverNode() : Node("maze_solver_node") {
    RCLCPP_INFO(this->get_logger(), "MazeSolverNode has been created");

    // Create service clients
    move_client_ = this->create_client<mineros_inter::srv::MoveTo>(
        "/mineros/set_position");

    // Wait for services to be available
    while (!move_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(), "Services are ready!");

    move_to_position_3d(125, 111, 108);
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered");
  }

  void move_to_position_3d(double x, double y, double z) {
    auto request = std::make_shared<mineros_inter::srv::MoveTo::Request>();

    request->pose.header.stamp = this->now();
    request->pose.header.frame_id = "map";

    request->pose.pose.position.x = x;
    request->pose.pose.position.y = y;
    request->pose.pose.position.z = z;

    request->pose.pose.orientation.w = 1.0;
    request->pose.pose.orientation.x = 0.0;
    request->pose.pose.orientation.y = 0.0;
    request->pose.pose.orientation.z = 0.0;

    RCLCPP_INFO(this->get_logger(),
                "Moving to 3D position: x= %.2f, y= %.2f, z= %.2f", x, y, z);

    auto future = move_client_->async_send_request(
        request, std::bind(&MazeSolverNode::move_response_callback, this,
                           std::placeholders::_1));
  }

  void move_response_callback(
      rclcpp::Client<mineros_inter::srv::MoveTo>::SharedFuture future) {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully reached target position!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach target position.");
    }
  }

  rclcpp::Client<mineros_inter::srv::MoveTo>::SharedPtr move_client_;
};

// #include ""
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  printf("hello world maze_solver package\n");

  auto node = std::make_shared<MazeSolverNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
