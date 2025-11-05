#ifndef MY_ROBOT_HARDWARE__MY_ROBOT_SYSTEM_HPP_
#define MY_ROBOT_HARDWARE__MY_ROBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <serial/serial.h>  // assume using e.g. serial package
#include <memory>
#include <vector>
#include <string>

namespace my_robot_hardware
{

class MyRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // command storage
  double vel_r_cmd_{0.0};
  double vel_l_cmd_{0.0};
  double vel_kick_cmd_{0.0};

  // state storage (if needed)
  double vel_r_state_{0.0};
  double vel_l_state_{0.0};
  double vel_kick_state_{0.0};

  // serial interface
  std::shared_ptr<serial::Serial> serial_port_;
  std::string port_name_;
  unsigned long baudrate_{115200};
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE__MY_ROBOT_SYSTEM_HPP_
