#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <mineros_inter/srv/detail/inventory__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// Service
#include <mineros_inter/srv/place_block.hpp>
#include <mineros_inter/srv/simple_place_block.hpp>
#include <mineros_inter/srv/inventory.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mineros_inter/msg/block_pose.hpp>
#include <mineros_inter/msg/item.hpp>



using namespace std::chrono_literals;

class PlaceBlockNode : public rclcpp::Node
{
public:
     PlaceBlockNode() : Node("place_block")
    {
        // Add build date for sanity checks.
		RCLCPP_INFO(this->get_logger(), "Running PlaceBlockNode built on %s at %s", __DATE__, __TIME__);


        client_ = this->create_client<mineros_inter::srv::PlaceBlock>("/mineros/interaction/place_block");

        while (!client_->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Venter på place_block service");
        }

        inventory_client_= this->create_client<mineros_inter::srv::Inventory>("/mineros/inventory/contents");

        while(!inventory_client_->wait_for_service(1s)){
        }

        auto request = std::make_shared<mineros_inter::srv::PlaceBlock::Request>();

        // Block pose and orientation
        request->block.block_pose.position.x = 115;
        request->block.block_pose.position.y = 103;
        request->block.block_pose.position.z = 124;
        request->block.block_pose.orientation.w = 1;
        
        // Item id
        request->block.block.id = 22;

        // Vector orient
        request->block.face_vector.z = -1;


        try
        {
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
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception caught %s", e.what());
        }
        
    }


    
private:
    rclcpp::Client<mineros_inter::srv::PlaceBlock>::SharedPtr client_;
    rclcpp::Service<mineros_inter::srv::SimplePlaceBlock>::SharedPtr placeblock_service_;
    rclcpp::Client<mineros_inter::srv::Inventory>::SharedPtr inventory_client_;

    void placeBlockCallback(const std::shared_ptr<mineros_inter::srv::SimplePlaceBlock::Request> request,
    std::shared_ptr<mineros_inter::srv::SimplePlaceBlock::Response> response) {

        auto inventory_request = std::make_shared<mineros_inter::srv::Inventory::Request>();

        auto future = inventory_client_->async_send_request(inventory_request);

        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)!= rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call inventory service");
            response->success = false;
            return;
        }

        auto inventory_response = future.get();
        bool found_block = false;
        
        // for (const auto &item : inventory_response->items) {
        //     if(item.id == 22 || item.id == 24) {
        //         found_block = true;
        //         break;
        //     }
        // }



        
    }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::make_shared<PlaceBlockNode>();
  rclcpp::shutdown();
  return 0;
}