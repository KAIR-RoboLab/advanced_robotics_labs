#pragma once

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

namespace btn_state_node {

class BTNStateNode : public BT::RosTopicSubNode<std_msgs::msg::Bool> {
 public:
  BTNStateNode(const std::string &name,
               const BT::NodeConfig &conf,
               const BT::RosNodeParams &params) : BT::RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params){};

  static BT::PortsList providedPorts() {
    // this node does not have any custom ports
    // port with topic name is defined in parent class
    return providedBasicPorts({});
  }

  BT::NodeStatus onTick(const typename std_msgs::msg::Bool::SharedPtr &last_msg) {
    // if any message war received and and that message contained true value return SUCCESS;
    if (last_msg != nullptr && last_msg->data) {
      return BT::NodeStatus::SUCCESS;
    }
    // if no message was received or message contained false return FAILURE
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace btn_state_node