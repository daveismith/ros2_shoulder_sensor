#include "ros2_shoulder_sensor/magnetic_angle_sensor.hpp"

#include <string>
#include <vector>
#include <limits>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace ros2_shoulder_sensor
{
//0.01745 radians = 1 degree
//0.000581667 radians = 0.033333 degrees = 2 minutes of arc (this is because we're updating 30x second, so 1 degree per second would be 0.033333 degrees per update)
MagneticAngleSensor::MagneticAngleSensor() : myAngle_(0.0), minAngle_(-0.0349), maxAngle_(0.6283), angleStep_(0.000581667), myCounter_(0)
{
}

hardware_interface::CallbackReturn MagneticAngleSensor::on_init(const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (info.joints.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "No joints specified in hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read hardware-level default
  double hw_default = 0.0;
  try {
    auto it = info.hardware_parameters.find("static_position");
    if (it != info.hardware_parameters.end()) {
      hw_default = std::stod(it->second);
    }
  } catch (...) {
    RCLCPP_WARN(rclcpp::get_logger("MagneticAngleSensor"), "Failed to parse hardware-level static_position; using 0.0");
  }

  RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "hw_default");


  positions_.resize(info.joints.size(), hw_default);
  static_positions_.resize(info.joints.size(), hw_default);

  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];
    try {
      auto it = joint.parameters.find("static_position");
      if (it != joint.parameters.end()) {
        static_positions_[i] = std::stod(it->second);
      } else {
        static_positions_[i] = hw_default;
      }
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("MagneticAngleSensor"), "Failed to parse static_position for joint %s; using default", joint.name.c_str());
      static_positions_[i] = hw_default;
    }

    try {
      auto it = joint.parameters.find("invert_position");
      if (it != joint.parameters.end()) {
        invert_position_.push_back(it->second == "true");
      } else {
        invert_position_.push_back(false);
      }
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("MagneticAngleSensor"), "Failed to parse invert_position for joint %s; using false", joint.name.c_str());
      invert_position_.push_back(false);
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MagneticAngleSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < positions_.size(); ++i) {
    const auto & name = this->info_.joints[i].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, "position", &positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MagneticAngleSensor::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type MagneticAngleSensor::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // For this static sensor, just copy configured static positions into the state storage
  myAngle_ += angleStep_;
  if (myAngle_ > maxAngle_) {
    angleStep_ = -angleStep_;
    myAngle_ = maxAngle_;
  } else if (myAngle_ < minAngle_) {
    angleStep_ = -angleStep_;
    myAngle_ = minAngle_;
  }

  RCLCPP_INFO(rclcpp::get_logger("MagneticAngleSensor"), "Updating positions to: %f (%.3f.%ld)", myAngle_, period.seconds(), period.nanoseconds());

  for (size_t i = 0; i < positions_.size(); ++i) {
    //positions_[i] = static_positions_[i];
    positions_[i] = invert_position_[i] ? -myAngle_ : myAngle_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MagneticAngleSensor::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // sensor provides state only
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_shoulder_sensor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_shoulder_sensor::MagneticAngleSensor, hardware_interface::SystemInterface)
