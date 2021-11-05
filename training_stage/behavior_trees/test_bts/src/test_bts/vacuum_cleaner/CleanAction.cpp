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
#include "test_bts/vacuum_cleaner/CleanAction.hpp"

namespace BT
{

CleanAction::CleanAction(const std::string & name)
: BT::ActionNodeBase(name, {})
{
  node_ = rclcpp::Node::make_shared("CleanActionBT");
}

void CleanAction::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Halt CleanAction");
}

BT::NodeStatus CleanAction::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Tick CleanAction");
  return BT::NodeStatus::SUCCESS;
}

}
