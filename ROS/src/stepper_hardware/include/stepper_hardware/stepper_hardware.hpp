#ifndef STEPPER_HARDWARE__STEPPER_HARDWARE_HPP_
#define STEPPER_HARDWARE__STEPPER_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>

#include "stepper_hardware/visiblity_control.h"
#include "rclcpp/macros.hpp"

#include "stepper_hardware/port_handler.h"
#include "stepper_hardware/port_handler_linux.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace stepper_hardware
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

class StepperHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StepperHardware)

  STEPPER_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  STEPPER_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  STEPPER_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  STEPPER_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  STEPPER_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  STEPPER_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  STEPPER_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  PortHandler *port;
  std::vector<Joint> joints_;
};

}  // namespace stepper_hardware

#endif  // STEPPER_HARDWARE__STEPPER_HARDWARE_HPP_
