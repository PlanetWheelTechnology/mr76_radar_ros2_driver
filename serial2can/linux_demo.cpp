#include <unistd.h>
#include <stdio.h>

#include "serial2can_interface.h"

static void sleep_ms(size_t milliseconds) {
	usleep(milliseconds * 1000);
}

int main(int argc, char const *argv[]) {

	Serial2CanInterface *serial2can = new Serial2CanInterface();

	std::string port_name = "/dev/ttyUSB0";
	uint32_t com_baudrate = 230400;
	if (!serial2can->Open(port_name, com_baudrate)) {
		LOG_ERROR("open error\n","");
		return -1;
	}

	serial2can_msg::CanMessageData can_msg_rx;
	serial2can_msg::CanMessageData can_msg_tx;

	auto last_time = std::chrono::steady_clock::now();

  while (1) {

		if (serial2can->ReadFromBus(can_msg_rx)) {
			if (serial2can_msg::kSerial2CAN_FRAME_TYPE_DATA == can_msg_rx.frame_type_) {
				if (serial2can_msg::kSerial2CAN_FRAME_ID_TYPE_STANDARD == can_msg_rx.frame_id_type_) {
					LOG_DEBUG("standard frame,%d", can_msg_rx.frame_id_type_);
				} else {
					LOG_DEBUG("extended frame,%d", can_msg_rx.frame_id_type_);
				}
				LOG_DEBUG("rx_id: 0x%x, rx_dlc: %d, rx_data: ", can_msg_rx.frame_id_, can_msg_rx.frame_dlc_,"");
				for (int i = 0; i < can_msg_rx.frame_dlc_; i++) {
					LOG_DEBUG("0x%x ", can_msg_rx.frame_data_buff_[i]);
				}
				LOG_DEBUG("-------","");
			}
		}

		if (std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - last_time).count() > 1000) {
			last_time = std::chrono::steady_clock::now();
			can_msg_tx.frame_id_ = 0x123;
			can_msg_tx.frame_dlc_ = 8;
			can_msg_tx.frame_type_ = serial2can_msg::kSerial2CAN_FRAME_TYPE_DATA;
			can_msg_tx.frame_id_type_ = serial2can_msg::kSerial2CAN_FRAME_ID_TYPE_STANDARD;
			for (int i = 0; i < can_msg_tx.frame_dlc_; i++) {
				can_msg_tx.frame_data_buff_[i] = i;
			}
			serial2can->WriteToBus(can_msg_tx);

			can_msg_tx.frame_id_ = 0x1234;
			can_msg_tx.frame_dlc_ = 8;
			can_msg_tx.frame_type_ = serial2can_msg::kSerial2CAN_FRAME_TYPE_DATA;
			can_msg_tx.frame_id_type_ = serial2can_msg::kSerial2CAN_FRAME_ID_TYPE_EXTENDED;
			for (int i = 0; i < can_msg_tx.frame_dlc_; i++) {
				can_msg_tx.frame_data_buff_[i] = i;
			}
			serial2can->WriteToBus(can_msg_tx);
		}

		sleep_ms(10);
  }

  serial2can->Close();

  return 0;
}
