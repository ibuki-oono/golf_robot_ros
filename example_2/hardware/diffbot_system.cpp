#include "ros2_control_demo_example_2/diffbot_system.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <cstring>
#include "rclcpp/logging.hpp" 

namespace ros2_control_demo_example_2
{
using namespace std::chrono_literals;

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel_vel_ = 0.0;
  right_wheel_vel_ = 0.0;
  kick_vel_ = 0.0;

  port_ = info.hardware_parameters.at("serial_port");
  baud_ = std::stoi(info.hardware_parameters.at("baud_rate"));

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"),"Failed to open serial port: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty {};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfsetospeed(&tty, baud_);
  cfsetispeed(&tty, baud_);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tcsetattr(serial_fd_, TCSANOW, &tty);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(info_.joints[0].name, "position", &left_wheel_vel_);
  state_interfaces.emplace_back(info_.joints[0].name, "velocity", &left_wheel_vel_);
  state_interfaces.emplace_back(info_.joints[1].name, "position", &right_wheel_vel_);
  state_interfaces.emplace_back(info_.joints[1].name, "velocity", &right_wheel_vel_);
  state_interfaces.emplace_back("kick_joint", "velocity", &kick_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(info_.joints[0].name, "velocity", &left_wheel_cmd_);
  command_interfaces.emplace_back(info_.joints[1].name, "velocity", &right_wheel_cmd_);
  command_interfaces.emplace_back("kick_joint", "velocity", &kick_cmd_);
  return command_interfaces;
}


hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Placeholder for future encoder read
  left_wheel_vel_ = left_wheel_cmd_;
  right_wheel_vel_ = right_wheel_cmd_;
  kick_vel_ = kick_cmd_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ >= 0)
  {
    // Convert wheel commands to integers
    int left_cmd_int  = static_cast<int>(left_wheel_cmd_);
    int right_cmd_int = static_cast<int>(right_wheel_cmd_);

    // Build serial command string
    std::ostringstream ss;
    ss << "VEL," << left_cmd_int << "," << right_cmd_int << ",0.0" << "\n";
    std::string msg = ss.str();

    // Print debug info
    RCLCPP_INFO(rclcpp::get_logger("HW"),
                "Sending to serial_fd_: %s", msg.c_str());
    ss << "VEL," << left_cmd_int << "," << right_cmd_int << "\n";

   ::write(serial_fd_, msg.c_str(), msg.size());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating hardware...");

  // reset commands when starting
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;
  kick_cmd_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating hardware...");

  // stop robot safely
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;
  kick_cmd_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}


}  // namespace ros2_control_demo_example_2

PLUGINLIB_EXPORT_CLASS(ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
