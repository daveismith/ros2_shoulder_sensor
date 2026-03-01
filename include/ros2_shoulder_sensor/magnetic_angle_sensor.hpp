// Minimal MagneticAngleSensor SystemInterface
#ifndef ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
#define ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace ros2_shoulder_sensor
{

class MagneticAngleSensor : public hardware_interface::SystemInterface
{
public:

  MagneticAngleSensor();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<bool> invert_position_{};
  std::vector<double> positions_{};
  std::vector<double> static_positions_{};
  hardware_interface::HardwareInfo info_{};

  double myAngle_;
  double minAngle_;  // -2 degrees in radians
  double maxAngle_;  // 36 degrees in radians
  double angleStep_;  // 1 degree in radians
  size_t myCounter_;
};

}  // namespace ros2_shoulder_sensor

#endif  // ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
