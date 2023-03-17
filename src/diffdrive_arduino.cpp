#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"




DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}





return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();
  
  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  f_l_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  f_r_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  b_l_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  b_r_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);  

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(f_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &f_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(f_l_wheel_.name, hardware_interface::HW_IF_POSITION, &f_l_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(f_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &f_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(f_r_wheel_.name, hardware_interface::HW_IF_POSITION, &f_r_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(b_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &b_l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(b_l_wheel_.name, hardware_interface::HW_IF_POSITION, &b_l_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(b_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &b_r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(b_r_wheel_.name, hardware_interface::HW_IF_POSITION, &b_r_wheel_.pos));  

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(f_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &f_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(f_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &f_r_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(b_l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &b_l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(b_r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &b_r_wheel_.cmd));
  

  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // arduino.setPidValues(9,7,0,100);
  // arduino.setPidValues(14,7,0,100);
  arduino_.setPidValues(30, 20, 0, 100);

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.readEncoderValues(f_l_wheel_.enc, f_r_wheel_.enc, b_l_wheel_.enc, b_r_wheel_.enc);

  double pos_prev = f_l_wheel_.pos;
  f_l_wheel_.pos = f_l_wheel_.calcEncAngle();
  f_l_wheel_.vel = (f_l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = f_r_wheel_.pos;
  f_r_wheel_.pos = f_r_wheel_.calcEncAngle();
  f_r_wheel_.vel = (f_r_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = b_l_wheel_.pos;
  b_l_wheel_.pos = b_l_wheel_.calcEncAngle();
  b_l_wheel_.vel = (b_l_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = b_r_wheel_.pos;
  b_r_wheel_.pos = b_r_wheel_.calcEncAngle();
  b_r_wheel_.vel = (b_r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveArduino::write()
{

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  arduino_.setMotorValues(f_l_wheel_.cmd / f_l_wheel_.rads_per_count / cfg_.loop_rate, f_r_wheel_.cmd / f_r_wheel_.rads_per_count / cfg_.loop_rate, b_l_wheel_.cmd / b_l_wheel_.rads_per_count / cfg_.loop_rate, b_r_wheel_.cmd / b_r_wheel_.rads_per_count / cfg_.loop_rate);




  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)