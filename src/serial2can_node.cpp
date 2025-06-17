/**
* @file         serial2can_node.cpp
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

#include "serial2can_node.h"

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>

using namespace std::chrono_literals;

class Serial2CanNode: public rclcpp::Node {
 public:
  Serial2CanNode(std::string port_name, int32_t com_baudrate)
  : Node("serial2can"),
    serial2can_(new Serial2CanInterface()),
    port_name_(port_name),
    com_baudrate_(com_baudrate) {

    // 声明 ROS2 参数
    this->declare_parameter<std::string>("port_name", port_name_);
    this->declare_parameter<int>("port_baudrate", com_baudrate_);
    // 获取 ROS2 参数
    if (this->get_parameter("port_name", port_name_)) {
      RCLCPP_INFO(this->get_logger(), "get param <port_name>:%s", port_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "use default param <port_name>:%s", port_name_.c_str());
    }

    if (this->get_parameter("port_baudrate", com_baudrate_)) {
      RCLCPP_INFO(this->get_logger(), "get param <port_baudrate>:%d", com_baudrate_);
    } else {
      RCLCPP_INFO(this->get_logger(), "use default <port_baudrate>:%d", com_baudrate_);
    }
    
    
    if (!serial2can_->Open(port_name_, com_baudrate_)) {
      RCLCPP_ERROR(this->get_logger(), "open %s error.", port_name_.c_str());
      RCLCPP_INFO(this->get_logger(), "Node:%s start fail.", this->get_name());
    } else {
      can_frame_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_frame_messages", 10);

      topic_message_pub_timer_ = this->create_wall_timer(10ms, std::bind(&Serial2CanNode::topic_message_pub_timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "Node:%s start success.", this->get_name());
    }
  }

 private:
  void topic_message_pub_timer_callback(void) {
    serial2can_msg::CanMessageData can_msg_rx;
    can_msgs::msg::Frame can_frame_msg;

    if (serial2can_->ReadFromBus(can_msg_rx)) {
      can_frame_msg.header.stamp = this->now();
      can_frame_msg.id = can_msg_rx.frame_id_;
      can_frame_msg.is_rtr = (can_msg_rx.frame_type_ == serial2can_msg::kSerial2CAN_FRAME_TYPE_REMOTE);
      can_frame_msg.is_extended = (can_msg_rx.frame_id_type_ == serial2can_msg::kSerial2CAN_FRAME_ID_TYPE_EXTENDED);
      can_frame_msg.is_error = false;  // 假设没有错误帧
      can_frame_msg.dlc = can_msg_rx.frame_dlc_;
      std::copy(std::begin(can_msg_rx.frame_data_buff_), std::end(can_msg_rx.frame_data_buff_), can_frame_msg.data.begin());

      can_frame_publisher_->publish(can_frame_msg);
    }
  }

  Serial2CanInterface* serial2can_;
  std::string port_name_;
  uint32_t com_baudrate_;
  rclcpp::TimerBase::SharedPtr topic_message_pub_timer_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_publisher_;  // can数据帧发布
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Serial2CanNode>("/dev/ttyUSB0", 230400);
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}


/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
