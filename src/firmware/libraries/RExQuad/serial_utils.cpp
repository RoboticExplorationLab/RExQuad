#include "serial_utils.hpp"

namespace rexquad {

int VerifyRead(char* buf, int len, uint8_t msg_id) {
  // Search for start of the message
  int start_index;
  uint8_t byte;
  for (start_index = 0; start_index < len; ++start_index) {
    byte  = buf[start_index];
    if (byte == msg_id) {
      break;
    }
  }

  if (start_index > 0) {
    // How many bytes of the message did we receive?
    int received_length = len - start_index;  

    // Shift the data over to the start of the buffer
    for (int i = 0; i < received_length; ++i) {
      buf[i] = buf[i + start_index];
    }
  }
  return start_index;
}


void PrintAtRate::print(const std::string &msg) {
  int time_since_last_print_ms = millis() - timestamp_last_print_ms_;
  if (time_since_last_print_ms > delay_ms_) {
    Serial.print(msg.c_str());
    timestamp_last_print_ms_ = millis();
  }
}
void PrintAtRate::println(const std::string &msg) {
  int time_since_last_print_ms = millis() - timestamp_last_print_ms_;
  if (time_since_last_print_ms > delay_ms_) {
    Serial.println(msg.c_str());
    timestamp_last_print_ms_ = millis();
  }
}

void print_rate() {
  static int time_start = 0; 
  static int count = 0;
  count++;
  if (count == 100) {
    float time_s = (micros() - time_start) / 1e6;
    Serial.print("Average rate: ");
    Serial.println(count / time_s);
    time_start = micros();
    count = 0;
  }
}
} // namespace rexquad