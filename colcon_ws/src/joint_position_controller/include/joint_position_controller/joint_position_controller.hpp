#ifndef JOINT_POSITION_CONTROLLER__JOINT_POSITION_CONTROLLER_HPP_
#define JOINT_POSITION_CONTROLLER__JOINT_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace joint_position_controller
{

class JointPositionController : public controller_interface::ControllerInterface
{
public:
  JointPositionController();

  virtual ~JointPositionController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Callback for joint command subscription
  void joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Joint names
  std::vector<std::string> joint_names_;

  // Command interfaces (positions to write)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> 
    joint_command_interface_;

  // State interfaces (current positions to read)
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
    joint_state_interface_;

  // Target and current positions
  std::vector<double> target_positions_;
  std::vector<double> current_positions_;
  std::vector<double> start_positions_;

  // Interpolation parameters
  double interpolation_time_;  // Total time for interpolation (seconds)
  double elapsed_time_;        // Elapsed time since new command
  bool new_command_received_;
  
  // Maximum velocity and acceleration limits
  double max_velocity_;
  double max_acceleration_;

  // ROS2 subscribers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_sub_;
  
  // ROS2 publishers (for feedback)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
};

}  // namespace joint_position_controller

#endif  // JOINT_POSITION_CONTROLLER__JOINT_POSITION_CONTROLLER_HPP_
