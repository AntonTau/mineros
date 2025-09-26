#include "rclcpp/rclcpp.hpp"
#include "mineros_inter/srv/move_to.hpp" // Service-definisjonen
#include "geometry_msgs/msg/pose_stamped.hpp"

class MoveBotTest : public rclcpp::Node
{
public:
  MoveBotTest() : Node("move_bot_test")
  {
    client_ = this->create_client<mineros_inter::srv::MoveTo>("/mineros/set_position");

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mineros/local_position/pose",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
      }
    );

    
  }

private:
  geometry_msgs::msg : PoseStamped getPos() const
  {
    return current_pose_;
  }

void moveRelative()
{
  while (!client_->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Venter på service... ");
    }

    auto pos = this ->getPos();
    auto request = std::make_shared<mineros_inter::srv::MoveTo::Request>();

    auto future = client_-> async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
      == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        RCLCPP_NFO(this->get_logger(), result->success ? "Flytting ok" : "Flytting feilet" );
      }

  }

private:
  rclcpp::Client<mineros_inter::srv::MoveTo>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  geometry_msgs::msg::PoseStamped current_pose_;
}
