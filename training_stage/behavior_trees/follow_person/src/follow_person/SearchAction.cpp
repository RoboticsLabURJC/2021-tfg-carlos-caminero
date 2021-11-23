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

#include "follow_person/SearchAction.hpp"

using std::placeholders::_1;


namespace BT
{

SearchAction::SearchAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  node_ = rclcpp::Node::make_shared("search_action_bt_node");
  node_->declare_parameter<std::string>("velocity_topic", "/cmd_vel");
  node_->declare_parameter<std::string>("darknet_bounding_boxes_topic", "/darknet_ros/bounding_boxes");
  
  std::string velocity_topic;
  node_->get_parameter("velocity_topic", velocity_topic);
  velpub_ = node_->create_publisher<geometry_msgs::msg::Twist>(velocity_topic, 10);

  std::string darknet_bounding_boxes_topic;
  node_->get_parameter("darknet_bounding_boxes_topic", darknet_bounding_boxes_topic);
  camerasub_ = node_->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
    darknet_bounding_boxes_topic, 1, std::bind(&SearchAction::cameraCallback, this, _1));

  BT::Optional<std::string> obj = getInput<std::string>("objective");
  if (!obj) {
    throw BT::RuntimeError("error reading port [target]:", obj.error());
  }
  objective_ = obj.value();
  
  objective_detected_ = false;
}

void SearchAction::cameraCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
{
  size_t i = 0;
  std::cout << "Strlen: bounding_boxes: " << msg->bounding_boxes.size() << std::endl;
  while (i < msg->bounding_boxes.size() && !objective_detected_) {
    if (msg->bounding_boxes[i].class_id == objective_) {
      objective_detected_ = true;
    }
    i++;
  }
}

BT::PortsList SearchAction::providedPorts()
{
  return {BT::InputPort<std::string>("objective")};
}

void SearchAction::halt()
{
  return;
}

BT::NodeStatus SearchAction::tick()
{
  rclcpp::spin_some(node_);
  geometry_msgs::msg::Twist velocity;

  velocity.linear.x = 0;
  if (objective_detected_) {
    objective_detected_ = false;
    velocity.angular.z = 0.0;
    velpub_->publish(velocity);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(), "Searching a %s", objective_.c_str());
  velocity.angular.z = 0.3;
  velpub_->publish(velocity);

  return BT::NodeStatus::RUNNING;
}

}
