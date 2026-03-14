#ifndef ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
#define ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "ros2_shoulder_sensor/magnetic_angle_sensor_core.hpp"

namespace ros2_shoulder_sensor
{

class MagneticAngleSensor : public hardware_interface::SystemInterface
{
public:
  MagneticAngleSensor();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<double> positions_{};
  std::vector<double> temperatures_{};
  hardware_interface::HardwareInfo info_{};

  std::string can_interface_{};
  uint32_t stale_timeout_ms_{0U};
  uint32_t startup_timeout_ms_{0U};

  int can_socket_fd_{-1};
  MagneticAngleSensorCore core_{};

  bool open_can_socket();
  void close_can_socket();
  std::chrono::steady_clock::time_point now_steady() const;
};

}  // namespace ros2_shoulder_sensor

#endif  // ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
