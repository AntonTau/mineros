#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

class DetermineNextPosition : public BT::SyncActionNode {
    public:
        DetermineNextPosition(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<int>("step"),
                     BT::InputPort<geometry_msgs::msg::PoseStamped>("current_position"),
                     BT::InputPort<int>("direction"), 
                     BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_point") };
        }

        BT::NodeStatus tick() override;
};