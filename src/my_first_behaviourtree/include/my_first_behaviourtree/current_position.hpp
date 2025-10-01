#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

class CurrentPosition : public BT::RosTopicSubNode<geometry_msgs::msg::PoseStamped> {
    public:
        CurrentPosition(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
            : BT::RosTopicSubNode<geometry_msgs::msg::PoseStamped>(name, config, params)
        {
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({ BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_position") });
        }

        BT::NodeStatus onTick(const std::shared_ptr<geometry_msgs::msg::PoseStamped>& msg) override;
};