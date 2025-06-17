/**
* @file         serial2can_interface.cpp
* @author       David Hu (hmd_hubei_cn@163.com)
* @brief        泥人电子 NiRen USB-CAN-V3 串口转CAN
* @version      0.1
* @date         2025.06.16
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

#include "serial2can_interface.h"

#ifdef USE_ROS2LOGGER
#include <rclcpp/rclcpp.hpp>
#endif

#include <cstring>

using namespace serial2can_msg;

Serial2CanInterface::Serial2CanInterface() 
  : serial_interface_ (new SerialInterfaceLinux()) {
  std::memset(&uart2can_msg_, 0, sizeof(uart2can_msg_));
  is_get_can_message_flag_ = false;
}

Serial2CanInterface::~Serial2CanInterface() {
  Close();
  if (serial_interface_ != nullptr) {
    delete serial_interface_;
  }
}

bool Serial2CanInterface::Open(std::string &port_name, uint32_t com_baudrate) {
  serial_interface_->SetReadCallback(std::bind(&Serial2CanInterface::CommReadCallback, 
    this, std::placeholders::_1, std::placeholders::_2));
  if (!serial_interface_->Open(port_name, com_baudrate)) {
#ifdef USE_ROS2LOGGER
    RCLCPP_ERROR(rclcpp::get_logger("serial2can"), "Serial2CanInterface::Open() failed.");
#else
    LOG_ERROR("Serial2CanInterface::Open() failed.", "");
#endif
    return false;
  } 
    
  return true;
}

bool Serial2CanInterface::Close() {
  return serial_interface_->Close();
}

bool Serial2CanInterface::ReadFromBus(CanMessageData &can_msg) {
  std::lock_guard<std::mutex> lg(can_message_list_mutex_);
  if (is_get_can_message_flag_) {
    is_get_can_message_flag_ = false;
    if (!can_message_list_.empty()) {
      can_msg = can_message_list_.front();  // 读取队列头数据
      can_message_list_.pop_front();  // 删除队列头数据
      return true;
    }
  } 
  
  return false;
}

bool Serial2CanInterface::WriteToBus(const CanMessageData &can_msg) {
  uart2can_pack_t pack;

  std::memset(&pack, 0, sizeof(pack));

  pack.header = 0xAA;
  pack.tail = 0x7A;
  pack.msg.frame_dlc = can_msg.frame_dlc_;

  if (can_msg.frame_dlc_ > 8) {
    return false;
  }

  pack.msg.frame_id[0] = can_msg.frame_id_ >> 24 & 0xFF;
  pack.msg.frame_id[1] = can_msg.frame_id_ >> 16 & 0xFF;
  pack.msg.frame_id[2] = can_msg.frame_id_ >> 8 & 0xFF;
  pack.msg.frame_id[3] = can_msg.frame_id_ & 0xFF;
  pack.msg.frame_type = can_msg.frame_type_;
  pack.msg.frame_id_type = can_msg.frame_id_type_;
  std::memcpy(pack.msg.frame_data_buff, can_msg.frame_data_buff_, can_msg.frame_dlc_);

  uint32_t tx_len = 0;
  if (!serial_interface_->WriteToIo((uint8_t *)&pack, sizeof(uart2can_pack_t), &tx_len)) {
#ifdef USE_ROS2LOGGER
    RCLCPP_ERROR(rclcpp::get_logger("serial2can"), "Serial2CanInterface::WriteToBus: WriteToIo failed.");
#else
    LOG_ERROR("Serial2CanInterface::WriteToBus: WriteToIo failed.", "");
#endif
    return false;
  } 

  return true;
}

static uint32_t Big2LittleEndian(uint8_t *byte4) {
  uint32_t bigendian = 0;
  bigendian = (uint32_t)((byte4[0] << 24) | (byte4[1] << 16) | (byte4[2] << 8) | (byte4[3]));
  return bigendian;
} 

void Serial2CanInterface::CommReadCallback(const char *byte, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (AnalysisOne(byte[i])) {
      CanMessageData can_msg;
      std::memset(&can_msg, 0, sizeof(CanMessageData));
      can_msg.frame_dlc_ = uart2can_msg_.frame_dlc;
      if (uart2can_msg_.frame_dlc > 8) {
#ifdef USE_ROS2LOGGER
        RCLCPP_ERROR(rclcpp::get_logger("serial2can"), "Serial2CanInterface::CommReadCallback: frame_dlc > 8.");
#else
        LOG_ERROR("Serial2CanInterface::CommReadCallback: frame_dlc > 8.","");
#endif
        return;
      }
      can_msg.frame_id_ = Big2LittleEndian(uart2can_msg_.frame_id);
      can_msg.frame_id_type_ = uart2can_msg_.frame_id_type;
      can_msg.frame_type_ = uart2can_msg_.frame_type;
      std::memcpy(can_msg.frame_data_buff_, uart2can_msg_.frame_data_buff, uart2can_msg_.frame_dlc);
      SetCanMessageData(can_msg);
      return;
    }
  }
}

void Serial2CanInterface::SetCanMessageData(const CanMessageData &can_msg) {
  std::lock_guard<std::mutex> lg(can_message_list_mutex_);
  can_message_list_.push_back(can_msg);
  is_get_can_message_flag_ = true;
}

bool Serial2CanInterface::AnalysisOne(uint8_t byte) {
  static enum {
    FRAME_HEADER,
    FRAME_DATA
  } state = FRAME_HEADER;
  static uint8_t rx_buff[100] = {0};
  static uint8_t rx_index = 0;
 
  uint8_t in_data = byte;
  switch (state) {
    case FRAME_HEADER: {
      if (0xAA == in_data) {
        rx_buff[rx_index++] = in_data;
        state = FRAME_DATA;
      }
      break;
    }
    case FRAME_DATA: {
      rx_buff[rx_index++] = in_data;
      if (rx_index >= 17) {
        rx_index = 0;
        state = FRAME_HEADER;
        if (0x7A == in_data) {
          std::memcpy(&uart2can_msg_, &rx_buff[1], sizeof(uart2can_msg_));
          return true;
        }
      } 
      break;
    }
  }

  return false;
}

/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
