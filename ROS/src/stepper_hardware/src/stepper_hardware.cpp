#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "stepper_hardware/stepper_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_hardware
{

CallbackReturn StepperHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  
  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "joint %d", i);
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "baud_rate: %d", baud_rate);

  // TODO: initialize the serial port
  // TODO: check ping

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepperHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StepperHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}

CallbackReturn StepperHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "start");

  // TODO: set home position

  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.position = 0.0;
    joints_[i].state.velocity = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn StepperHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "stop");
  return CallbackReturn::SUCCESS;
}

return_type StepperHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // TODO: read states from serial port
  return return_type::OK;
}

return_type StepperHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // TODO: write commands to serial port
  return return_type::OK;
}

}  // namespace stepper_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stepper_hardware::StepperHardware, hardware_interface::SystemInterface)
