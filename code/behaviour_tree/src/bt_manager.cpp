#include "bt_lab/bt_manager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "bt_lab/btn_state_node.hpp"
#include "bt_lab/nav_to_goal_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"

using namespace std::chrono_literals;

BTManager::BTManager() : Node("bt_manager"), count_(0) {
  // get ROS 2 install path to bt_lab package
  auto bt_lab_pkg_path_str = ament_index_cpp::get_package_share_directory("bt_lab");
  // find path to bt_test.xml file
  rcpputils::fs::path bt_xml_path =
      rcpputils::fs::path(bt_lab_pkg_path_str) / "config" / "bt_test.xml";

  // create noded handle pointer to pass to bt nodes
  std::shared_ptr<rclcpp::Node> sub_nh = this->create_sub_node("bt_nodes");
  // define timeout for action calls
  auto timeout = std::chrono::seconds(5000);

  // register navigate to goal pose bt node
  factory_.registerNodeType<nav_to_goal_node::NavToGoalNode>(
      "NavToGoalNode", BT::RosNodeParams({sub_nh, "/navigate_to_pose", timeout}));
  // register button state bt node
  factory_.registerNodeType<btn_state_node::BTNStateNode>(
      "ButtonNode", BT::RosNodeParams({sub_nh, "", timeout}));

  // inform user bt nodes were registered
  RCLCPP_INFO(this->get_logger(), "Nodes registerd");

  // load and parse the behaviour tree xml file
  bt_tree_ = factory_.createTreeFromFile(bt_xml_path.string());
  // start timer to tick the tree
  timer_ = this->create_wall_timer(100ms, std::bind(&BTManager::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "ROS node started");
}

void BTManager::timer_callback() {
  // log tree state
  BT::StdCoutLogger logger_cout(bt_tree_);
  // tick tree
  bt_tree_.tickOnce();
}