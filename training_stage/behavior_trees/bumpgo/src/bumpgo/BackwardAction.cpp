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

#include <memory>
#include <string>
#include "bumpgo/BackwardAction.hpp"

namespace BT
{
BackwardAction::BackwardAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  node_ = rclcpp::Node::make_shared("BackwardActionBT");
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

BT::PortsList BackwardAction::providedPorts()
{
  return {BT::InputPort<std::string>("duration")};
}

void BackwardAction::halt()
{
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 0;
  velocity.angular.z = 0;
  publisher_->publish(velocity);
  RCLCPP_INFO(node_->get_logger(), "Halt BackwardAction");
}

BT::NodeStatus BackwardAction::tick()
{
  rclcpp::spin_some(node_);
  static bool init = false;
  if (!init) {
    BT::Optional<double> msg = getInput<double>("duration");
    if (!msg) {
      throw BT::RuntimeError("error reading port [target]:", msg.error());
    }
    duration_time_ = msg.value();
    start_time_ = (clock_->now());
    init = true;
  }

  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = -0.3;
  velocity.angular.z = 0;
  publisher_->publish(velocity);
  RCLCPP_INFO(node_->get_logger(), "Tick BackwardAction");

  if ((clock_->now() - start_time_).seconds() > duration_time_) {
    init = false;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace BT
