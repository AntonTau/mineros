#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "mineros_inter/srv/move_to.hpp"

using MoveToService = mineros_inter::srv::MoveTo;

class MoveTo : public BT::RosServiceNode<MoveToService> {
    public:
        explicit MoveTo(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
            : BT::RosServiceNode<MoveToService>(name, config, params)
        {
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({ BT::OutputPort<bool>("currently_moving"), 
                                        BT::InputPort<geometry_msgs::msg::PoseStamped>("target_point") });
        }

        bool setRequest(Request::SharedPtr& request) override;

        BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

        virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

};