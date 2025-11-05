#include "my_robot_hardware/my_robot_system.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept> 
#include <cstring> // For strerror

namespace my_robot_hardware
{

hardware_interface::CallbackReturn MyRobotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read serial port parameters from hardware_parameters
  if (info_.hardware_parameters.find("port_name") != info_.hardware_parameters.end()) {
    port_name_ = info_.hardware_parameters.at("port_name");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Missing serial port parameter");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    // Note: std::stoul may throw if the string is invalid
    try {
      baudrate_ = std::stoul(info_.hardware_parameters.at("baudrate"));
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "Invalid baudrate string, defaulting to 115200: %s", e.what());
      baudrate_ = 115200;
    }
  } else {
    baudrate_ = 115200;  // default
  }

  // Open serial port
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Cannot open serial port %s: %s", port_name_.c_str(), strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure serial port
  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to get serial attributes: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Set baud rate (using B115200 for simplicity as it's a common default)
  // In a production system, you'd map baudrate_ integer to a BXXXX macro
  cfsetospeed(&tty, B115200); 
  cfsetispeed(&tty, B115200);
  
  tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver and ignore modem control lines
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 data bits
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;  // No flow control

  tty.c_lflag = 0;          // No canonical mode, echo, etc.
  tty.c_oflag = 0;          // No output processing
  
  // Set VMIN (minimum characters to read) and VTIME (timeout in 0.1s units)
  tty.c_cc[VMIN] = 0;       // Read does not block
  tty.c_cc[VTIME] = 5;      // Timeout after 0.5s

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to set serial attributes: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"),
              "Serial port opened: %s @ %lu", port_name_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
} 

// CORRECT Implementation matching the header's lifecycle signature (const rclcpp_lifecycle::State &)
hardware_interface::CallbackReturn MyRobotSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_fd_ >= 0) {
    // Use global close function
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Serial port closed on cleanup.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MyRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, "velocity", &vel_r_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, "velocity", &vel_l_state_));
  interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[2].name, "velocity", &vel_kick_state_));
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, "velocity", &vel_r_cmd_));
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, "velocity", &vel_l_cmd_));
  interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[2].name, "velocity", &vel_kick_cmd_));
  return interfaces;
}

hardware_interface::return_type MyRobotSystemHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // For simplicity, we just keep last commanded velocities as state
  vel_r_state_ = vel_r_cmd_;
  vel_l_state_ = vel_l_cmd_;
  vel_kick_state_ = vel_kick_cmd_;
  return hardware_interface::return_type::OK;
} 

hardware_interface::return_type MyRobotSystemHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) return hardware_interface::return_type::ERROR;

  std::ostringstream oss;
  // Protocol: VEL,R_VEL,L_VEL,KICK_VEL\n
  oss << "VEL," << vel_r_cmd_ << "," << vel_l_cmd_ << "," << vel_kick_cmd_ << "\n";
  std::string out = oss.str();

  // Call global write to avoid conflict with member function
  ssize_t n = ::write(serial_fd_, out.c_str(), out.size());
  if (n < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Serial write failed: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// plugin export
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotSystemHardware,
  hardware_interface::SystemInterface
); 

}  // namespace my_robot_hardware