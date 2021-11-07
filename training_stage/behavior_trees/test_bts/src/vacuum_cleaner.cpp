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

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "test_bts/vacuum_cleaner/CheckBattery.hpp"
#include "test_bts/vacuum_cleaner/CleanAction.hpp"
#include "test_bts/vacuum_cleaner/Recharge.hpp"

#ifndef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vacuum_cleaner");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<BT::CheckBattery>("CheckBattery");
  factory.registerNodeType<BT::CleanAction>("CleanAction");
  factory.registerNodeType<BT::Recharge>("Recharge");

  std::string pkgpath = ament_index_cpp::get_package_prefix("test_bts");
  std::string xml_file = pkgpath + "/bts_xml/vacuum_cleaner.xml";

  auto tree = factory.createTreeFromFile(xml_file);

  // Creating logger that saves state changes.
  BT::FileLogger logger_file(tree, "/tmp/vc_bt.fbl");
  // Creating logger that stores the execution time of each node
  BT::MinitraceLogger logger_minitrace(tree, "/tmp/vc_bt.json");

  rclcpp::Rate loop_rate(10);
  BT::NodeStatus status;
  while (rclcpp::ok() && status != BT::NodeStatus::SUCCESS) {
    status = tree.rootNode()->executeTick();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
