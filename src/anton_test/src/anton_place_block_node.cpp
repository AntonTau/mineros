#include <rclcpp/rclcpp.hpp>

// Service
#include <mineros_inter/srv/place_block.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mineros_inter/msg/block_pose.hpp>
#include <mineros_inter/msg/item.hpp>


using namespace std::chrono_literals;

class PlaceBlock : public rclcpp::Node
{
public:
     PlaceBlock() : Node("place_block")
    {
        client_ = this->create_client<mineros_inter::srv::PlaceBlock>("/mineros/interaction/place_block");

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Venter på place_block service");
        }

        auto request = std::make_shared<mineros_inter::srv::PlaceBlock::Request>();

        // Block pose and orientation
        request->block.block_pose.position.x = 119;
        request->block.block_pose.position.y = 101;
        request->block.block_pose.position.z = 126;
        request->block.block_pose.orientation.w = 1;
        
        // Item id
        request->block.block.id = 22;

        // Vector orient
        request->block.face_vector.y = 1;

        auto future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) 
        == rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto result = future.get();
            if(result->success) {
                RCLCPP_INFO(this->get_logger(), "Plassering av blokk fullført");
            } else {
                RCLCPP_INFO(this->get_logger(), "Plassering av blokk feilet");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Kunne ikke kalle servicen");
        }


    }

private:
    rclcpp::Client<mineros_inter::srv::PlaceBlock>::SharedPtr client_;


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<PlaceBlock>();
  rclcpp::shutdown();
  return 0;
};