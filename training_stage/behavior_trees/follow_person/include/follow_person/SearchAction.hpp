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

#ifndef FOLLOWPERSON__SEARCHACTION_HPP_
#define FOLLOWPERSON__SEARCHACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace BT
{

class SearchAction : public BT::ActionNodeBase
{
public:
  SearchAction(const std::string & name, const BT::NodeConfiguration & config);
  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

private:
  void cameraCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velpub_;
  rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr camerasub_;
  std::string objective_;
  bool objective_detected_;
};

}  // namespace BT

#endif  // FOLLOWPERSON__SEARCHACTION_HPP_

