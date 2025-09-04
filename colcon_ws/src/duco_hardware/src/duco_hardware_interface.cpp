#include "duco_hardware/duco_hardware_interface.hpp"
#include "duco_ros_driver/DucoCobot.h"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace duco_hardware
{

hardware_interface::CallbackReturn DucoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize hardware state arrays
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_prev_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Get connection parameters
  robot_ip_ = info_.hardware_parameters.at("robot_ip");
  robot_port_ = std::stoi(info_.hardware_parameters.at("robot_port"));

  // Initialize joint names
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Initialize flags
  controller_connected_flag_ = false;
  start_move_ = false;

  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), 
    "DucoHardwareInterface initialized for robot at %s:%d", 
    robot_ip_.c_str(), robot_port_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DucoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "Configuring DUCO hardware interface...");
  
  // Create DUCO robot connection
  duco_robot_ = std::make_shared<DucoRPC::DucoCobot>(robot_ip_, robot_port_);
  
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "DUCO hardware interface configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DucoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), 
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DucoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), 
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn DucoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "Activating DUCO hardware interface...");
  
  // Connect to robot
  if (duco_robot_->open() != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DucoHardwareInterface"), 
      "Failed to connect to DUCO robot at %s:%d", robot_ip_.c_str(), robot_port_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  controller_connected_flag_ = true;
  
  // Power on and enable robot
  duco_robot_->power_on(true);
  usleep(1000000);  // 1 second delay
  duco_robot_->enable(true);
  usleep(1000000);  // 1 second delay
  
  // Initialize command positions to current positions
  std::vector<double> current_positions;
  duco_robot_->get_actual_joints_position(current_positions);
  
  for (std::size_t i = 0; i < hw_commands_.size() && i < current_positions.size(); i++)
  {
    hw_positions_[i] = current_positions[i];
    hw_commands_[i] = current_positions[i];
    hw_commands_prev_[i] = current_positions[i];
    hw_velocities_[i] = 0.0;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "DUCO hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DucoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "Deactivating DUCO hardware interface...");
  
  if (duco_robot_)
  {
    duco_robot_->close();
  }
  controller_connected_flag_ = false;
  
  RCLCPP_INFO(rclcpp::get_logger("DucoHardwareInterface"), "DUCO hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DucoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!controller_connected_flag_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Read joint positions and velocities from DUCO robot
  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  
  try
  {
    duco_robot_->get_actual_joints_position(joint_positions);
    duco_robot_->get_actual_joints_speed(joint_velocities);
    
    for (std::size_t i = 0; i < hw_positions_.size() && i < joint_positions.size(); i++)
    {
      hw_positions_[i] = joint_positions[i];
      // Use velocity from DUCO if available, otherwise set to 0
      if (i < joint_velocities.size()) {
        hw_velocities_[i] = joint_velocities[i];
      } else {
        hw_velocities_[i] = 0.0;
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DucoHardwareInterface"), 
      "Error reading from DUCO robot: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DucoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!controller_connected_flag_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Check if commands have changed
  bool commands_changed = false;
  for (std::size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::abs(hw_commands_[i] - hw_commands_prev_[i]) > 1e-6)
    {
      commands_changed = true;
      break;
    }
  }

  if (commands_changed)
  {
    try
    {
      // Send joint commands to DUCO robot
      std::vector<double> commands(hw_commands_.begin(), hw_commands_.end());
      
      // Use movej for position control (joint space movement)
      duco_robot_->movej(commands, 1.5, 1.0, 0.0, false);  // speed=1.5, acceleration=1.0, radius=0.0, non-blocking
      
      // Update previous commands
      hw_commands_prev_ = hw_commands_;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DucoHardwareInterface"), 
        "Error writing to DUCO robot: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace duco_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  duco_hardware::DucoHardwareInterface, hardware_interface::SystemInterface)
