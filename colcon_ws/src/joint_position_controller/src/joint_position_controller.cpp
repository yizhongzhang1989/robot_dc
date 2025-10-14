#include "joint_position_controller/joint_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace joint_position_controller
{

JointPositionController::JointPositionController()
: controller_interface::ControllerInterface(),
  interpolation_time_(5.0),
  elapsed_time_(0.0),
  new_command_received_(false),
  max_velocity_(1.0),
  max_acceleration_(1.0)
{
}

controller_interface::CallbackReturn JointPositionController::on_init()
{
  try
  {
    // Declare and get parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("interpolation_time", 5.0);
    auto_declare<double>("max_velocity", 1.0);
    auto_declare<double>("max_acceleration", 1.0);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  interpolation_time_ = get_node()->get_parameter("interpolation_time").as_double();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint names specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Resize vectors
  target_positions_.resize(joint_names_.size(), 0.0);
  current_positions_.resize(joint_names_.size(), 0.0);
  start_positions_.resize(joint_names_.size(), 0.0);

  // Create subscriber for joint commands
  joint_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/joint_command", 
    rclcpp::SystemDefaultsQoS(),
    std::bind(&JointPositionController::joint_command_callback, this, std::placeholders::_1)
  );

  // Create publisher for state feedback
  state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/state", 
    rclcpp::SystemDefaultsQoS()
  );

  RCLCPP_INFO(
    get_node()->get_logger(), 
    "Configured JointPositionController with %zu joints", 
    joint_names_.size()
  );

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
JointPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::InterfaceConfiguration 
JointPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get command interfaces
  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      joint_names_.size(), command_interfaces_.size()
    );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get state interfaces
  if (state_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, got %zu",
      joint_names_.size(), state_interfaces_.size()
    );
    return controller_interface::CallbackReturn::ERROR;
  }

  // Clear previous references
  joint_command_interface_.clear();
  joint_state_interface_.clear();

  // Store references to interfaces
  for (auto & command_interface : command_interfaces_)
  {
    joint_command_interface_.emplace_back(std::ref(command_interface));
  }

  for (auto & state_interface : state_interfaces_)
  {
    joint_state_interface_.emplace_back(std::ref(state_interface));
  }

  // Initialize positions from current state
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    current_positions_[i] = joint_state_interface_[i].get().get_value();
    target_positions_[i] = current_positions_[i];
    start_positions_[i] = current_positions_[i];
    
    // Set initial command to current position
    joint_command_interface_[i].get().set_value(current_positions_[i]);
  }

  elapsed_time_ = 0.0;
  new_command_received_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "JointPositionController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release interfaces
  joint_command_interface_.clear();
  joint_state_interface_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "JointPositionController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read current joint positions
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    current_positions_[i] = joint_state_interface_[i].get().get_value();
  }

  // Trajectory planning and interpolation logic (movej-style)
  if (new_command_received_)
  {
    elapsed_time_ += period.seconds();
    
    // Calculate normalized time (0 to 1)
    double t = std::min(elapsed_time_ / interpolation_time_, 1.0);
    
    // Use quintic (5th order) polynomial for smooth position, velocity, and acceleration
    // This is similar to movej trajectory planning
    // s(t) = 10t³ - 15t⁴ + 6t⁵
    // This ensures: s(0)=0, s(1)=1, s'(0)=0, s'(1)=0, s''(0)=0, s''(1)=0
    double t3 = t * t * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double smooth_t = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
    
    // Interpolate each joint
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      double interpolated_position = start_positions_[i] + 
        smooth_t * (target_positions_[i] - start_positions_[i]);
      
      // Write to command interface (servoj will execute this at hardware level)
      joint_command_interface_[i].get().set_value(interpolated_position);
    }
    
    // Check if interpolation is complete
    if (t >= 1.0)
    {
      new_command_received_ = false;
      elapsed_time_ = 0.0;
      
      RCLCPP_INFO(get_node()->get_logger(), "Target position reached");
    }
  }
  else
  {
    // Hold current position if no new command
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_command_interface_[i].get().set_value(current_positions_[i]);
    }
  }

  // Publish state feedback
  auto state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  state_msg->header.stamp = get_node()->now();
  state_msg->name = joint_names_;
  state_msg->position = current_positions_;
  
  state_publisher_->publish(std::move(state_msg));

  return controller_interface::return_type::OK;
}

void JointPositionController::joint_command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Received command with %zu values, expected %zu. Ignoring.",
      msg->data.size(), joint_names_.size()
    );
    return;
  }

  // Store current position as start position for interpolation
  // Calculate maximum joint displacement to determine motion time
  double max_displacement = 0.0;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    start_positions_[i] = current_positions_[i];
    target_positions_[i] = msg->data[i];
    double displacement = std::abs(target_positions_[i] - start_positions_[i]);
    max_displacement = std::max(max_displacement, displacement);
  }

  // Adaptive interpolation time based on movement distance (movej-style)
  // Time = distance / max_velocity, with minimum time to ensure smooth motion
  double adaptive_time = max_displacement / max_velocity_;
  interpolation_time_ = std::max(adaptive_time, 1.0);  // At least 1 second
  
  // Reset interpolation
  elapsed_time_ = 0.0;
  new_command_received_ = true;

  RCLCPP_INFO(
    get_node()->get_logger(), 
    "New joint command received (max displacement: %.3f rad, duration: %.2f s)", 
    max_displacement, interpolation_time_
  );
}

}  // namespace joint_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_position_controller::JointPositionController,
  controller_interface::ControllerInterface
)
