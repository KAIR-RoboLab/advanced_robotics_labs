#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav_to_goal_node {
class NavToGoalNode : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
 public:
  NavToGoalNode(const std::string& name,
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params) : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params){};

  static BT::PortsList providedPorts() {
    // create custom ports with goal and nav_bt
    BT::PortsList ports = {
        BT::InputPort<float>("goal_x", "Goal position in m for x direction"),
        BT::InputPort<float>("goal_y", "Goal position in m for y direction"),
        BT::InputPort<float>("goal_theta", "Goal heading angle in radians"),
        // suggested to leave empty. Empty string will use default tree
        BT::InputPort<std::string>("nav_bt", "Behaviour Tree to be used by Navigation 2")};
    return providedBasicPorts(ports);
    ;
  }

  // callback to set new goal for the action
  bool setGoal(nav2_msgs::action::NavigateToPose::Goal& goal) {
    // get values from input ports
    BT::Expected<float> goal_x = getInput<float>("goal_x");
    BT::Expected<float> goal_y = getInput<float>("goal_y");
    BT::Expected<float> goal_theta = getInput<float>("goal_theta");
    BT::Expected<std::string> nav_bt = getInput<std::string>("nav_bt");

    // fail if any of ports is invalid
    if (!goal_x || !goal_y || !goal_theta || !nav_bt) {
      RCLCPP_INFO(node_->get_logger(), "Could not get goal from the blackboard!");
      return false;
    }

    // if everything is all right navigate to the goal
    RCLCPP_INFO(node_->get_logger(),
                "Navigating to: '%.2f', '%.2f'.", goal_x.value(), goal_y.value());

    // setup goal message
    goal.behavior_tree = nav_bt.value();

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->get_clock()->now();
    goal.pose.pose.position.x = double(goal_x.value());
    goal.pose.pose.position.y = double(goal_y.value());

    // create quaternion with rotation around z axis
    tf2::Quaternion quat(tf2::Vector3(0.0, 0.0, 1.0), goal_theta.value());
    goal.pose.pose.orientation.w = quat.w();
    goal.pose.pose.orientation.x = quat.x();
    goal.pose.pose.orientation.y = quat.y();
    goal.pose.pose.orientation.z = quat.z();
    return true;
  }

  // handles for the action result
  BT::NodeStatus onResultReceived(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    }
    // in case of UNKNOWN, CANCELED or ABORTED
    return BT::NodeStatus::FAILURE;
  }

  // handler for the action feedback
  BT::NodeStatus onFeedback(const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/) {
    return BT::NodeStatus::RUNNING;
  }

  // handler for the action failure
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode /*error*/) {
    return BT::NodeStatus::FAILURE;
  }
};
}  // namespace nav_to_goal_node