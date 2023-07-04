#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "stepper_hardware/stepper_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

#include "stepper_hardware/port_handler.h"
#include "stepper_hardware/port_handler_linux.h"

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

  // RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "usb_port: %s", usb_port.c_str());
  // RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "baud_rate: %d", baud_rate);

  port = PortHandler::getPortHandler(usb_port.c_str());
  port->setBaudRate(baud_rate);
  RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "serial: %s", port->getPortName());
  RCLCPP_INFO(rclcpp::get_logger("StepperHardware"), "serial baud: %d", port->getBaudRate());
  port->openPort();
  // TODO: check if open
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
    joints_[i].state.position = 1.0;
    joints_[i].state.velocity = 1.0;
    joints_[i].command.position = 0.0;
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
  // CAUTION: The ControllerManager needs to go faster than the arduino
  while (port->getBytesAvailable() > 0) {
    port->readPort(&buffer[available], 1);
    if (buffer[available] == '\n' || buffer[available] == '\r' || buffer[available] == '\0') {
      if (available > 1){
        if (buffer[0] == 'P' && buffer[1] == '0') {
          std::string buffer_str(reinterpret_cast<char*>(buffer), available);

          // Split the string by commas
          std::istringstream iss(buffer_str);
          std::vector<std::string> tokens;
          std::string token;
          while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
          }

          for (const std::string& t : tokens) {
            size_t colon_pos = t.find(':');
            if (colon_pos != std::string::npos) {
              std::string key = t.substr(0, colon_pos); // Extract key
              std::string value = t.substr(colon_pos + 1); // Extract value
              if (key == "P0"){joints_[0].state.position = std::stof(value);} 
              if (key == "P1"){joints_[1].state.position = std::stof(value);}
              if (key == "P2"){joints_[2].state.position = std::stof(value);}
              if (key == "P3"){joints_[3].state.position = std::stof(value);}
              if (key == "V0"){joints_[0].state.velocity = std::stof(value);}
              if (key == "V1"){joints_[1].state.velocity = std::stof(value);}
              if (key == "V2"){joints_[2].state.velocity = std::stof(value);}
              if (key == "V3"){joints_[3].state.velocity = std::stof(value);}
            }
          }
        }
      }
      available = 0;
    }
    else{available++;}
  }

  return return_type::OK;
}

return_type StepperHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  std::ostringstream msg_str;
  msg_str << std::fixed << std::setprecision(3)
          << joints_[0].command.position << ","
          << joints_[1].command.position << ","
          << joints_[2].command.position << ","
          << joints_[3].command.position << "\n";

  std::string msg = msg_str.str();
  std::vector<uint8_t> msg_bytes(msg.begin(), msg.end());
  port->writePort(msg_bytes.data(), msg_bytes.size());

  return return_type::OK;
}

}  // namespace stepper_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(stepper_hardware::StepperHardware, hardware_interface::SystemInterface)
