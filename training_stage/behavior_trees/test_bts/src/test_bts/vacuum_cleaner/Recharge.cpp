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
#include "test_bts/vacuum_cleaner/Recharge.hpp"

using namespace std::chrono_literals;

namespace BT
{

Recharge::Recharge(const std::string & name)
: BT::ActionNodeBase(name, {})
{
  node_ = rclcpp::Node::make_shared("RechargeBT");
  battery_client = node_->create_client<example_interfaces::srv::BatteryOption>("battery_service");
  while (!battery_client->wait_for_service(1s)) {
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }
}

void Recharge::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Halt Recharge");
}

BT::NodeStatus Recharge::tick()
{
  auto request = std::make_shared<example_interfaces::srv::BatteryOption::Request>();
  request->operation = "Recharge";

  auto result = battery_client->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, result);

  RCLCPP_INFO(node_->get_logger(), "Tick Recharge [%d]", result.get()->level);
  if (result.get()->level < 90) {
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace BT
