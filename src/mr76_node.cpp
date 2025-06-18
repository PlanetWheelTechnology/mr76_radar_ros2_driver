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
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <chrono>

using namespace std::chrono_literals;

class Mr76RadarNode: public rclcpp::Node {
 public:
  Mr76RadarNode()
  : Node("mr76_radar"),
    can_messages_topic_name_("can_frame_messages") {

    // 声明 ROS2 参数
    this->declare_parameter<std::string>("can_messages_topic_name", 
      can_messages_topic_name_);
    // 获取 ROS2 参数
    if (this->get_parameter("can_messages_topic_name", can_messages_topic_name_)) {
      RCLCPP_INFO(this->get_logger(), "get param <can_messages_topic_name>:%s", 
        can_messages_topic_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "use default param <can_messages_topic_name>:%s", 
        can_messages_topic_name_.c_str());
    }
    
    // 创建订阅者，订阅CAN数据帧
    can_frame_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      can_messages_topic_name_, 
      300,
      std::bind(&Mr76RadarNode::CanFrameCallback, this, std::placeholders::_1));

    // 创建发布者，发布MR76 Radar扫描数据
    mr76_radar_scan_publisher_ = this->create_publisher<radar_msgs::msg::RadarScan>(
      "mr76_radar_scan",
      10);

    RCLCPP_INFO(this->get_logger(), "Node:%s start success.", this->get_name());
  }

 private:

  radar_msgs::msg::RadarScan MakeRadarScanMessage(const RadarData& radar_data) {
    radar_msgs::msg::RadarScan radar_scan_msg;
    radar_msgs::msg::RadarReturn radar_return_msg;

    radar_scan_msg.header.stamp = this->now();
    radar_scan_msg.header.frame_id = "mr76_radar_frame";

    // 填充雷达返回数据
    float obj_dist = 0; // 目标径向距离
    float obj_angle = 0; // 目标角度
    float obj_vel = 0; // 目标径向速度

    int16_t num_targets = static_cast<int16_t>(radar_data.targets.size());

    for (int16_t i = 0; i < num_targets; i++) {
      obj_dist = std::sqrt(radar_data.targets[i].dist_long * radar_data.targets[i].dist_long +
        radar_data.targets[i].dist_lat * radar_data.targets[i].dist_lat); // 单位 m
      obj_angle = std::atan2(radar_data.targets[i].dist_lat, 
        radar_data.targets[i].dist_long); // 单位弧度  转换为角度(* 180.0 / M_PI)
      obj_vel = radar_data.targets[i].vel_rel_long * std::cos(obj_angle) +
        radar_data.targets[i].vel_rel_lat * std::sin(obj_angle); // 单位 m/s
      
      radar_return_msg.range = obj_dist; // 目标径向距离
      radar_return_msg.azimuth = obj_angle; // 目标角度，单位弧度
      radar_return_msg.elevation = 0.0f; // 假设没有垂直方向数据
      radar_return_msg.doppler_velocity = obj_vel; // 目标径向速度
      radar_return_msg.amplitude = radar_data.targets[i].rcs; // 雷达散射截面
      
      // 将雷达返回数据添加到扫描消息中
      radar_scan_msg.returns.push_back(radar_return_msg);
    }

    return radar_scan_msg;
  }


  // 回调函数：接收CAN数据帧
  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), 
    //   "Received CAN frame:ID=0x%x,DLC=%u,Data=[0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x]",
    //   msg->id, msg->dlc,
    //   msg->data[0], msg->data[1], msg->data[2], msg->data[3],
    //   msg->data[4], msg->data[5], msg->data[6], msg->data[7]);

    if (msg->is_rtr || msg->is_error) {
      RCLCPP_WARN(this->get_logger(), "Received invalid CAN frame: ID=0x%x, is_rtr=%d, is_error=%d",
                  msg->id, msg->is_rtr, msg->is_error);
      return; // 忽略RTR和错误帧
    }

    can_msgs::msg::Frame can_frame_msg;
    can_frame_msg.header = msg->header;  // 复制消息头
    can_frame_msg.id = msg->id;
    can_frame_msg.is_rtr = msg->is_rtr;
    can_frame_msg.is_extended = msg->is_extended;
    can_frame_msg.is_error = msg->is_error;
    can_frame_msg.dlc = msg->dlc;
    std::copy(msg->data.begin(), msg->data.end(), can_frame_msg.data.begin());

    if (mr76_radar_.ExtractDetections(can_frame_msg, curr_radar_out_)) {
      radar_scan_msg_ = MakeRadarScanMessage(curr_radar_out_);
      mr76_radar_scan_publisher_->publish(radar_scan_msg_);
    }
  }

  

  std::string can_messages_topic_name_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_subscription_;  // can数据帧订阅
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr mr76_radar_scan_publisher_;  // mr76 Radar扫描数据发布
  Mr76Driver mr76_radar_;    // mr76 radar driver
  RadarData curr_radar_out_; // latest extracted scene
  radar_msgs::msg::RadarScan radar_scan_msg_; // radar scan message
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mr76RadarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}

/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
