// Copyright 2021 Carlos Caminero (Carlosalpha1)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "bumpgo/ForwardAction.hpp"

namespace BT
{
ForwardAction::ForwardAction(const std::string & name)
: BT::ActionNodeBase(name, {})
{
  node_ = rclcpp::Node::make_shared("ForwardActionBT");
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void ForwardAction::halt()
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0;
  velocity.angular.z = 0;
  publisher_->publish(velocity);
  RCLCPP_INFO(node_->get_logger(), "Halt ForwardAction");
}

BT::NodeStatus ForwardAction::tick()
{
  rclcpp::spin_some(node_);
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0.3;
  velocity.angular.z = 0;
  publisher_->publish(velocity);
  RCLCPP_INFO(node_->get_logger(), "Tick ForwardAction");
  return BT::NodeStatus::FAILURE;
}

}  // namespace BT
