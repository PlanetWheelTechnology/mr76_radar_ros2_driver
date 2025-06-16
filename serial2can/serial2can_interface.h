/**
* @file         serial2can_interface.h
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

#ifndef __SERIAL2CAN_INTERFACE_H__
#define __SERIAL2CAN_INTERFACE_H__

#include <chrono>
#include <list>

#include "serial_interface_linux.h"


#pragma pack(1)

typedef struct {
  uint8_t frame_id_type;   // 帧ID类型： 0x00： 标准帧； 0x01： 扩展帧
  uint8_t frame_type;     // 帧数据类型: 0x00： 数据帧； 0x01： 远程帧
  uint8_t frame_dlc;      // 帧数据区有效长度   0x01 ~ 0x08
  uint8_t frame_id[4];    // 帧ID  , 标准帧ID占11位，扩展帧ID占29位，大端模式
  uint8_t frame_data_buff[8];  // 帧数据区
} uart2can_msg_t;

typedef struct {
  uint8_t header;         // 包首 0xAA
  uart2can_msg_t msg;     // CAN消息帧
  uint8_t tail;           // 包尾 0x7A
} uart2can_pack_t;

#pragma pack()

namespace serial2can_msg {

const uint8_t kSerial2CAN_FRAME_ID_TYPE_STANDARD  = 0x00; // 标准ID帧
const uint8_t kSerial2CAN_FRAME_ID_TYPE_EXTENDED  = 0x01; // 扩展ID帧
const uint8_t kSerial2CAN_FRAME_TYPE_DATA        = 0x00;  // 数据帧
const uint8_t kSerial2CAN_FRAME_TYPE_REMOTE      = 0x01; // 远程帧

struct CanMessageData {
  uint8_t frame_type_; // 帧数据类型: 0x00： 数据帧； 0x01： 远程帧
  uint8_t frame_id_type_; // 帧ID类型： 0x00： 标准帧； 0x01： 扩展帧
  uint32_t frame_id_;  // 帧ID  , 标准帧ID占11位，扩展帧ID占29位
  uint8_t frame_dlc_; // 帧数据区有效长度   0x01 ~ 0x08
  union {
    uint8_t frame_data_buff_[8];
    struct {
      uint8_t data1_;
      uint8_t data2_;
      uint8_t data3_;
      uint8_t data4_;
      uint8_t data5_;
      uint8_t data6_;
      uint8_t data7_;
      uint8_t data8_;
    };
  };
};

typedef std::list<CanMessageData> CanMessageList;

} //  serial2can_msg



class Serial2CanInterface {
public:
  Serial2CanInterface();

  ~Serial2CanInterface();

  bool Open(std::string &port_name, uint32_t com_baudrate = 230400);

  bool Close();

  bool ReadFromBus(serial2can_msg::CanMessageData &can_msg);

  bool WriteToBus(const serial2can_msg::CanMessageData &can_msg);

private:
  SerialInterfaceLinux *serial_interface_;

  uart2can_msg_t uart2can_msg_;

  serial2can_msg::CanMessageList can_message_list_;

  bool is_get_can_message_flag_;

  std::mutex can_message_list_mutex_;


  void CommReadCallback(const char *byte, size_t len);

  void SetCanMessageData(serial2can_msg::CanMessageData &can_msg);

  bool AnalysisOne(uint8_t byte);
};


#endif //__SERIAL2CAN_INTERFACE_H__
/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
