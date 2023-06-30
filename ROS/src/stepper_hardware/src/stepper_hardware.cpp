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
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepperHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StepperHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  return command_interfaces;
}

CallbackReturn StepperHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "start");
  return CallbackReturn::SUCCESS;
}

CallbackReturn StepperHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("StepperHardware"), "stop");
  return CallbackReturn::SUCCESS;
}

return_type StepperHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  return return_type::OK;
}

return_type StepperHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  return return_type::OK;
}

}  // namespace stepper_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stepper_hardware::StepperHardware, hardware_interface::SystemInterface)
