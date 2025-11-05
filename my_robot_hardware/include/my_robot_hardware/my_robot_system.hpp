#ifndef MY_ROBOT_HARDWARE_MY_ROBOT_SYSTEM_HPP_
#define MY_ROBOT_HARDWARE_MY_ROBOT_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp" // Required for lifecycle callbacks
#include <string>
#include <vector>

namespace my_robot_hardware
{

class MyRobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  
  // Corrected signature for on_cleanup
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  // Export joint state interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Export joint command interfaces
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read from hardware (update states)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write commands to hardware (send to Arduino)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial port parameters
  std::string port_name_;
  unsigned long baudrate_{115200};
  int serial_fd_{-1};

  // Joint states
  double vel_r_state_{0.0};
  double vel_l_state_{0.0};
  double vel_kick_state_{0.0};

  // Joint commands
  double vel_r_cmd_{0.0};
  double vel_l_cmd_{0.0};
  double vel_kick_cmd_{0.0};
};

}  // namespace my_robot_hardware

#endif  // MY_ROBOT_HARDWARE_MY_ROBOT_SYSTEM_HPP_