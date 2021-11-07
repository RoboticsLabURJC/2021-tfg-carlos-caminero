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

#ifndef TEST_BTS__VACUUM_CLEANER__CHECKBATTERY_HPP_
#define TEST_BTS__VACUUM_CLEANER__CHECKBATTERY_HPP_

#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/battery_option.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace BT
{

class CheckBattery : public BT::ActionNodeBase
{
public:
  explicit CheckBattery(const std::string & name);
  void halt() override;
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<example_interfaces::srv::BatteryOption>::SharedPtr battery_client;
};

}  // namespace BT

#endif  // TEST_BTS__VACUUM_CLEANER__CHECKBATTERY_HPP_
