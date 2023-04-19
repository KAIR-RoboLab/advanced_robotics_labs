#pragma once

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class BTManager : public rclcpp::Node {
 public:
  BTManager();

 private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree bt_tree_;
  std::size_t count_;
};