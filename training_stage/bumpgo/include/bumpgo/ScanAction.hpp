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

#ifndef BUMPGO__SCANACTION_HPP_
#define BUMPGO__SCANACTION_HPP_

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace BT
{

namespace laser_config
{
int LASER_MIN_SAMPLE = 250;
int LASER_MAX_SAMPLE = 470;

void set_range(const int min_sample, const int max_sample);
}

class ScanAction : public BT::ActionNodeBase
{
public:
  explicit ScanAction(const std::string & name);
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void halt() override;
  BT::NodeStatus tick() override;

private:
  double laser_average_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
};

}  // namespace BT

#endif  // BUMPGO__SCANACTION_HPP_
