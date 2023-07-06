#ifndef MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_
#define MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>

#include "visiblity_control.h"
#include "rclcpp/macros.hpp"

#include "MSP.hpp"
#include "serial.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace mecanum_hardware
{
struct JointValue
{
  double velocity{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
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
  int fd;
  std::vector<Joint> joints_;
  MSP_SET_MOTOR_t vel_command;
  MSP_RAW_IMU_t imu_state;
};
}  // namespace mecanum_hardware

#endif  // MECANUM_HARDWARE__MECANUM_HARDWARE_HPP_
