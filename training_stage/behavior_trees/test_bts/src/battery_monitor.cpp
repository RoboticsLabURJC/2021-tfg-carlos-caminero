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

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/battery_option.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Battery : public rclcpp::Node
{
public:
  Battery()
  : Node("battery_monitor_server"), level(100)
  {
    service_ = create_service<example_interfaces::srv::BatteryOption>(
      "battery_service", std::bind(&Battery::action_battery, this, _1, _2));
    RCLCPP_INFO(get_logger(), "Battery Ready.");
  }

  void action_battery(
    const std::shared_ptr<example_interfaces::srv::BatteryOption::Request> request,
    std::shared_ptr<example_interfaces::srv::BatteryOption::Response> response)
  {
    if (request->operation == "Get") {
      response->level = level;
    } else if (request->operation == "Consume") {
      level -= 8;
      response->level = level;
    } else if (request->operation == "Recharge") {
      level += 5;
      response->level = level;
    }

    RCLCPP_INFO(get_logger(), "Operation request [%s]", request->operation.c_str());
    RCLCPP_INFO(get_logger(), "Battery send: [%d]", response->level);
  }

private:
  int level;
  rclcpp::Service<example_interfaces::srv::BatteryOption>::SharedPtr service_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Battery>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
