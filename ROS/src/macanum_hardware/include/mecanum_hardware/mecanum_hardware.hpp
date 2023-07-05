#ifndef MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_
#define MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>

#include "mecanum_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

#include "mecanum_hardware/port_handler.h"
#include "mecanum_hardware/port_handler_linux.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace mecanum_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
};

enum class ControlMode {
  Position,
  Velocity,
};

class MecanumHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumHardware)

  MECANUM_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  MECANUM_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MECANUM_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MECANUM_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  MECANUM_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  MECANUM_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MECANUM_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  PortHandler *port;
  uint8_t buffer[1000];
  int available = 0;
  std::vector<Joint> joints_;
};

}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_
