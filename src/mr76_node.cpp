/**
* @file         mr76_node.cpp
* @author       David Hu (hmd_hubei_cn@163.com)
* @brief         
* @version      0.1
* @date         2025.06.17
* @note          
* @copyright    Copyright (c) 2022 DAVID HU All rights reserved. Licensed under the MIT License (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License in the file LICENSE
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**/

#include "mr76_node.h"

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>

using namespace std::chrono_literals;

class Mr76RadarNode: public rclcpp::Node {
 public:
  Mr76RadarNode()
  : Node("mr76_radar"),
    can_messages_topic_name_("can_frame_messages") {

    // 声明 ROS2 参数
    this->declare_parameter<std::string>("can_messages_topic_name", can_messages_topic_name_);
    // 获取 ROS2 参数
    if (this->get_parameter("can_messages_topic_name", can_messages_topic_name_)) {
      RCLCPP_INFO(this->get_logger(), "get param <can_messages_topic_name>:%s", can_messages_topic_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "use default param <can_messages_topic_name>:%s", can_messages_topic_name_.c_str());
    }
    
    // 创建订阅者，订阅CAN数据帧
    can_frame_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      can_messages_topic_name_,
      rclcpp::QoS(10),
      std::bind(&Mr76RadarNode::CanFrameCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Node:%s start success.", this->get_name());
  }

 private:
  // 回调函数：接收CAN数据帧
  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received CAN frame: ID=0x%x, DLC=%u, Data=[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]",
                msg->id, msg->dlc,
                msg->data[0], msg->data[1], msg->data[2], msg->data[3],
                msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
  }

  std::string can_messages_topic_name_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscription_;  // can数据帧订阅
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mr76RadarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
