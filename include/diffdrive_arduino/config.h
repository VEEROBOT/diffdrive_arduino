#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
/*   std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel"; */
  std::string front_left_wheel_name = "front_left_link";
  std::string front_right_wheel_name = "front_right_link";
  std::string back_left_wheel_name = "back_left_link";
  std::string back_right_wheel_name = "back_right_link";  
  float loop_rate = 28;
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 560;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H