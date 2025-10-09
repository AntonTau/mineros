// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/wait_for_message.hpp>

// std Headers
#include <chrono>

// services
#include "mineros_inter/srv/bot_pos.hpp"
#include "mineros_inter/srv/find_blocks.hpp"
#include "mineros_inter/srv/mine_block.hpp"

// messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "mineros_inter/msg/item.hpp"

#define MINEROS_GETPOS_TOPIC "/mineros/local_position/pose"
#define MINEROS_GETBLOCKS_TOPIC "/mineros/mining/find_blocks"
#define MINEROS_MINEBLOCK_TOPIC "/mineros/mining/mine_block"


using std::placeholders::_1;
using std::placeholders::_2;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Pose;
using mineros_inter::srv::MineBlock;
using mineros_inter::srv::FindBlocks;
using mineros_inter::msg::Item;

using namespace std::chrono_literals;

class MinerosNode : public rclcpp::Node {

    public:
        MinerosNode() : Node("mining_node") {
					// Add build date for sanity checks.
					RCLCPP_INFO(this->get_logger(), "Running mining node built on %s at %s", __DATE__, __TIME__);

					// Subscriber: Get bot position.
					pose_subscriber_ = this->create_subscription<PoseStamped>(
						MINEROS_GETPOS_TOPIC, 
						10,
						std::bind(&MinerosNode::pose_callback, this, _1));
          RCLCPP_INFO(this->get_logger(), "[+] Subscribed to %s", MINEROS_GETPOS_TOPIC);

					// Client: Mine.
					this->mineblock_client_ = this->create_client<MineBlock>(MINEROS_MINEBLOCK_TOPIC);

					while (!mineblock_client_->wait_for_service(1s)) {
						RCLCPP_INFO(this->get_logger(), "Waiting for MineBlock service...");
					}

					// Client: Find block to mine.
					this->findblocks_client_ = this->create_client<FindBlocks>(MINEROS_GETBLOCKS_TOPIC);

					while (!findblocks_client_->wait_for_service(1s)) {
						RCLCPP_INFO(this->get_logger(), "Waiting for FindBlock service...");
					}
        }
			private:
				rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscriber_;
				rclcpp::Client<MineBlock>::SharedPtr mineblock_client_;
				rclcpp::Client<FindBlocks>::SharedPtr findblocks_client_;
				PoseStamped pose;
				PoseStamped previous_pose;
				PoseArray blocks_to_mine;
				bool success = true;
				void pose_callback(const PoseStamped::SharedPtr msg) {
					RCLCPP_INFO(this->get_logger(), "Received pose: [%.2f,%.2f,%.2f]",
							msg->pose.position.x,
							msg->pose.position.y,
							msg->pose.position.z);
					this->pose = *msg;
					// Only look for blocks to mine if bot has moved
					if (this->pose.pose.position.x != this->previous_pose.pose.position.x ||
							this->pose.pose.position.y != this->previous_pose.pose.position.y ||
							this->pose.pose.position.z != this->previous_pose.pose.position.z) {
						// Make request to FindBlock service
						auto request = std::make_shared<FindBlocks::Request>();
						request->blockid = 68;
						request->max_distance = 4;
						request->count = 10;

						auto future = this->findblocks_client_->async_send_request(request, [this](rclcpp::Client<FindBlocks>::SharedFuture result) {
							this->blocks_to_mine = result.get()->blocks;
							for (const auto& block : this->blocks_to_mine.poses) {
								auto mine_request = std::make_shared<MineBlock::Request>();
								mine_request->block.position.x = block.position.x;
								mine_request->block.position.y = block.position.y;
								mine_request->block.position.z = block.position.z;
								this->mineblock_client_->async_send_request(mine_request, [this](rclcpp::Client<MineBlock>::SharedFuture mine_result) {
									this->success = mine_result.get()->success;
								});
							}
						});
					}
					this->previous_pose = this->pose;
					
				}
};