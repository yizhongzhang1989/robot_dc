#ifndef DUCO_HARDWARE__DUCO_HARDWARE_INTERFACE_HPP_
#define DUCO_HARDWARE__DUCO_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Forward declaration
namespace DucoRPC {
class DucoCobot;
}

namespace duco_hardware
{
class DucoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DucoHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // RPC connection to DUCO robot
  std::shared_ptr<DucoRPC::DucoCobot> duco_robot_;
  
  // Joint state data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_prev_;
  
  // Connection parameters
  std::string robot_ip_;
  int robot_port_;
  
  // Control flags
  bool controller_connected_flag_;
  bool start_move_;
  
  // Joint names mapping
  std::vector<std::string> joint_names_;
};

}  // namespace duco_hardware

#endif  // DUCO_HARDWARE__DUCO_HARDWARE_INTERFACE_HPP_
