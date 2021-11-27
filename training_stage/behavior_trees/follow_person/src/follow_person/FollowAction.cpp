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

#include <cmath>

#include "follow_person/FollowAction.hpp"

using std::placeholders::_1;


namespace BT
{

FollowAction::FollowAction(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
  node_ = rclcpp::Node::make_shared("search_action_bt_node");
  node_->declare_parameter<std::string>("velocity_topic", "/cmd_vel");
  node_->declare_parameter<std::string>("darknet_bounding_boxes_topic", "/darknet_ros/bounding_boxes");
  node_->declare_parameter<std::string>("darknet_found_object_topic", "/darknet_ros/found_object");
  
  std::string velocity_topic;
  node_->get_parameter("velocity_topic", velocity_topic);
  velpub_ = node_->create_publisher<geometry_msgs::msg::Twist>(velocity_topic, 10);

  std::string darknet_bounding_boxes_topic;
  node_->get_parameter("darknet_bounding_boxes_topic", darknet_bounding_boxes_topic);
  bboxessub_ = node_->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
    darknet_bounding_boxes_topic, 1, std::bind(&FollowAction::boundingBoxesCallback, this, _1));
  
  std::string darknet_found_object_topic;
  node_->get_parameter("darknet_found_object_topic", darknet_found_object_topic);
  objcountsub_ = node_->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
    darknet_found_object_topic, 1, std::bind(&FollowAction::countObjectsCallback, this, _1));

  BT::Optional<std::string> obj = getInput<std::string>("objective");
  if (!obj) {
    throw BT::RuntimeError("error reading port [target]:", obj.error());
  }

  objective_.id = obj.value();
  objective_.detected = false;
}

void FollowAction::boundingBoxesCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg)
{
  static int err_cont = 0;
  int xmin, ymin, xmax, ymax, cx, cy;
  bool detection = false;
  for (size_t i= 0; i < msg->bounding_boxes.size(); i++) {
    if (msg->bounding_boxes[i].class_id == objective_.id) {
      xmin = msg->bounding_boxes[i].xmin;
      ymin = msg->bounding_boxes[i].ymin;
      xmax = msg->bounding_boxes[i].xmax;
      ymax = msg->bounding_boxes[i].ymax;
      cx = xmin + (xmax-xmin)/2;
      cy = ymin + (ymin+ymax)/2;
      if (!objective_.detected ||
          sqrt(pow(abs(cx - objective_.cx), 2) + pow(abs(cy - objective_.cy), 2)) < 60) {
        detection = true;
        objective_.cx = cx;
        objective_.cy = cy;
      }
    }
  }
  if (!detection) {
    if (err_cont > 10) {
      err_cont = 0;
      objective_.detected = false;

    } else {
      err_cont++;
      objective_.detected = true;
    }
  } else {
    objective_.detected = true;
  }
  std::cout << "odec: " << objective_.detected << " y err: " << err_cont << std::endl;
}

void FollowAction::countObjectsCallback(const darknet_ros_msgs::msg::ObjectCount::SharedPtr msg)
{
  if (msg->count == 0)
    objective_.detected = false;
}

BT::PortsList FollowAction::providedPorts()
{
  return {BT::InputPort<std::string>("objective")};
}

void FollowAction::halt()
{
  return;
}

BT::NodeStatus FollowAction::tick()
{
  int width = 640;

  static float velocities[11] = {0.7, 0.45, 0.3, 0.2, 0.1, 0, -0.1, -0.2, -0.3, -0.45, -0.7};

  rclcpp::spin_some(node_);
  geometry_msgs::msg::Twist velocity;

  velocity.angular.z = 0.0;

  RCLCPP_INFO(node_->get_logger(), "Follow %s, status: [%d]\n", objective_.id.c_str(), objective_.detected);
  if (objective_.detected) {
    
    std::vector<float> ranges;
    
    for (int i = 0; i < 11; i++) {
      ranges.push_back(i*(float(width)/11));
    }

    for (size_t i = 1; i < ranges.size(); i++) {
      if (objective_.cx < ranges[i]) {
        velocity.angular.z = velocities[i];
        break;
      }
    }
    velocity.linear.x = 0.3;
    std::cout << "velx: " << velocity.linear.x << std::endl;
    velpub_->publish(velocity);
    return BT::NodeStatus::RUNNING;
  }

  velocity.linear.x = 0.0;
  velpub_->publish(velocity);

  return BT::NodeStatus::FAILURE;
}

}
