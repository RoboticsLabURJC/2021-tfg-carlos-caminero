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

#ifndef BUMPGO__BACKWARDACTION_HPP_
#define BUMPGO__BACKWARDACTION_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace BT
{

class BackwardAction : public BT::ActionNodeBase
{
public:
  BackwardAction(const std::string & name, const BT::NodeConfiguration & config);
  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Time start_time_;
  float actual_time_;
  float duration_time_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace BT

#endif  // BUMPGO__BACKWARDACTION_HPP_
