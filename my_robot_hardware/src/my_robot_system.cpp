#include "my_robot_hardware/my_robot_system.hpp"
#include <chrono>
#include <iostream>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn MyRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Example: read serial port parameters from info_.hardware_parameters
  if (info_.hardware_parameters.find("port_name") != info_.hardware_parameters.end()) {
    port_name_ = info_.hardware_parameters.at("port_name");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Missing serial port parameter");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    baudrate_ = std::stoul(info_.hardware_parameters.at("baudrate"));
  }

  // Open serial port
  try {
    serial_port_ = std::make_shared<serial::Serial>(port_name_, baudrate_, serial::Timeout::simpleTimeout(1000));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Serial port opened: %s @ %lu", port_name_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_r_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_l_state_));
  // Kick velocity maybe no real state, but publish as a state too
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &vel_kick_state_));
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_r_cmd_));
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_l_cmd_));
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[2].name, hardware_interface::HW_IF_VELOCITY, &vel_kick_cmd_));
  return interfaces;
}

hardware_interface::return_type MyRobotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // If you have sensors / encoders, you would read them here and update vel_r_state_, vel_l_state_, vel_kick_state_
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Format the command to serial: "VEL,vel_r,vel_l,vel_kick\n"
  std::ostringstream oss;
  oss << "VEL," << vel_r_cmd_ << "," << vel_l_cmd_ << "," << vel_kick_cmd_ << "\n";
  std::string out = oss.str();
  try {
    serial_port_->write(out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Serial write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// plugin export
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotSystemHardware,
  hardware_interface::SystemInterface
)

}  // namespace my_robot_hardware
