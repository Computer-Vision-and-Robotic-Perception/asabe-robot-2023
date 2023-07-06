#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mecanum_hardware/mecanum_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mecanum_hardware/serial.hpp"

namespace mecanum_hardware
{

CallbackReturn MecanumHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  
  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "joint %d", i);
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "baud_rate: %d", baud_rate);

  fd = serial_open(usb_port.c_str(), B115200);
  if (fd < 0) {
      RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Error opening the port: %s", usb_port.c_str());
  }
  else{
    RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"), "Port: %s", usb_port.c_str());
  }

  MSP_RAW_IMU(fd, &imu_state);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"accX: %d\n", imu_state.accX);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"accY: %d\n", imu_state.accY);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"accZ: %d\n", imu_state.accZ);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"gyrX: %d\n", imu_state.gyrX);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"gyrY: %d\n", imu_state.gyrY);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"gyrZ: %d\n", imu_state.gyrZ);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"magX: %d\n", imu_state.magX);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"magY: %d\n", imu_state.magY);
  RCLCPP_INFO(rclcpp::get_logger("MecanumHardware"),"magZ: %d\n", imu_state.magZ);

  // TODO: check if open
  // TODO: check ping

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }
  return command_interfaces;
}

CallbackReturn MecanumHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "start");

  // TODO: set home position

  for (uint i = 0; i < info_.joints.size(); i++) {
    joints_[i].state.velocity = 0.0;
    joints_[i].command.velocity = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger("MecanumHardware"), "stop");
  return CallbackReturn::SUCCESS;
}

return_type MecanumHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // CAUTION: The ControllerManager needs to go faster than the arduino
  MSP_RAW_IMU(fd, &imu_state);
  for(int i = 0; i < int(joints_.size()); i++){
    joints_[i].state.velocity = joints_[i].command.velocity;
  }
  return return_type::OK;
}

return_type MecanumHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // 1400rmp/V, 12V, ratio 8:1, rad = 0.00635 mm = 1400 * (1/60) * 12 * (1/8) * 2 * pi
  for(int i = 0; i < int(joints_.size()); i++){
    vel_command.motor[i] = 1500 + uint16_t(joints_[i].command.velocity * 2.2727); // max: 220[rad/s] 
  }
  // MSP_SET_MOTOR(fd, &vel_command);
  return return_type::OK;
}

}  // namespace mecanum_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mecanum_hardware::MecanumHardware, hardware_interface::SystemInterface)
