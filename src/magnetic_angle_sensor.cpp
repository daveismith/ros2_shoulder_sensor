#include "ros2_shoulder_sensor/magnetic_angle_sensor.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>
#include <cstdio>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace ros2_shoulder_sensor
{
constexpr std::size_t kMaxFramesPerRead = 100U;

MagneticAngleSensor::MagneticAngleSensor()
{
}

hardware_interface::CallbackReturn MagneticAngleSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (info.joints.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "No joints specified in hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto interface_it = info.hardware_parameters.find("can_interface");
  if (interface_it == info.hardware_parameters.end() || interface_it->second.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Missing required hardware parameter 'can_interface'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = interface_it->second;

  const auto stale_it = info.hardware_parameters.find("stale_timeout_ms");
  const auto startup_it = info.hardware_parameters.find("startup_timeout_ms");

  try {
    stale_timeout_ms_ = stale_it ==
      info.hardware_parameters.end() ? 200U : std::stoul(stale_it->second);
    startup_timeout_ms_ =
      startup_it ==
      info.hardware_parameters.end() ? stale_timeout_ms_ : std::stoul(startup_it->second);
  } catch (...) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Invalid timeout parameters: stale_timeout_ms and startup_timeout_ms must be unsigned integers");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<MagneticAngleSensorJointConfig> joint_configs;
  joint_configs.reserve(info.joints.size());
  positions_.assign(info.joints.size(), 0.0);
  temperatures_.assign(info.joints.size(), 0.0);

  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto & joint = info.joints[i];

    const auto node_id_it = joint.parameters.find("node_id");
    if (node_id_it == joint.parameters.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("MagneticAngleSensor"),
        "Joint '%s' is missing required parameter 'node_id'",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    MagneticAngleSensorJointConfig cfg;
    cfg.joint_name = joint.name;

    try {
      const unsigned long parsed_node = std::stoul(node_id_it->second);
      if (parsed_node > 255U) {
        throw std::out_of_range("node_id out of range");
      }
      cfg.node_id = static_cast<uint8_t>(parsed_node);
    } catch (...) {
      RCLCPP_ERROR(
        rclcpp::get_logger("MagneticAngleSensor"),
        "Joint '%s' has invalid node_id '%s' (expected 0..255)",
        joint.name.c_str(),
        node_id_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto invert_it = joint.parameters.find("invert_position");
    cfg.invert_position = (invert_it != joint.parameters.end() && invert_it->second == "true");

    const auto offset_it = joint.parameters.find("zero_offset_deg");
    if (offset_it != joint.parameters.end()) {
      try {
        cfg.zero_offset_deg = std::stod(offset_it->second);
      } catch (...) {
        RCLCPP_ERROR(
          rclcpp::get_logger("MagneticAngleSensor"),
          "Joint '%s' has invalid zero_offset_deg '%s'",
          joint.name.c_str(),
          offset_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    joint_configs.push_back(cfg);
  }

  std::string configure_error;
  if (!core_.configure(joint_configs, stale_timeout_ms_, startup_timeout_ms_, &configure_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to configure sensor core: %s",
      configure_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MagneticAngleSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MagneticAngleSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_can_socket()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  core_.mark_started(now_steady());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MagneticAngleSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_can_socket();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MagneticAngleSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(positions_.size() * 2U);

  for (size_t i = 0; i < positions_.size(); ++i) {
    const auto & name = this->info_.joints[i].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, "position",
        &positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, "temperature",
        &temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MagneticAngleSensor::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type MagneticAngleSensor::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  (void)period;
  if (can_socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "CAN socket is not open");
    return hardware_interface::return_type::ERROR;
  }

  std::string diagnostic_message;
  std::size_t frames_read = 0;
  while (frames_read < kMaxFramesPerRead) {
    struct can_frame frame {};
    const ssize_t bytes_read = ::recv(can_socket_fd_, &frame, sizeof(frame), 0);
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      RCLCPP_ERROR(
        rclcpp::get_logger("MagneticAngleSensor"),
        "Error while reading CAN socket: %s",
        std::strerror(errno));
      return hardware_interface::return_type::ERROR;
    }

    if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
      continue;
    }

    // Skip remote transmission request (RTR) and error frames; only process data frames.
    if ((frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
      continue;
    }

    MagneticAngleSensorFrame decoded_frame;
    decoded_frame.is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    decoded_frame.can_id = decoded_frame.is_extended ?
      (frame.can_id & CAN_EFF_MASK) : (frame.can_id & CAN_SFF_MASK);
    decoded_frame.dlc = frame.can_dlc;
    for (size_t i = 0; i < decoded_frame.data.size(); ++i) {
      decoded_frame.data[i] = frame.data[i];
    }

    std::string maybe_diag;
    const bool consumed = core_.process_frame(decoded_frame, now_steady(), &maybe_diag);
    if (consumed && !maybe_diag.empty()) {
      diagnostic_message = maybe_diag;
    }

    ++frames_read;
  }

  if (!diagnostic_message.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("MagneticAngleSensor"), "%s", diagnostic_message.c_str());
  }

  std::string state_error;
  if (!core_.compute_states(now_steady(), &positions_, &temperatures_, &state_error)) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "%s", state_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MagneticAngleSensor::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // sensor provides state only
  return hardware_interface::return_type::OK;
}

bool MagneticAngleSensor::open_can_socket()
{
  close_can_socket();

  can_socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to create CAN socket: %s",
      std::strerror(errno));
    return false;
  }

  struct ifreq ifr {};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());

  if (::ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to lookup CAN interface '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_can_socket();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(can_socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to bind CAN socket to '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_can_socket();
    return false;
  }

  const int current_flags = ::fcntl(can_socket_fd_, F_GETFL, 0);
  if (current_flags < 0 || ::fcntl(can_socket_fd_, F_SETFL, current_flags | O_NONBLOCK) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to set CAN socket non-blocking: %s",
      std::strerror(errno));
    close_can_socket();
    return false;
  }

  return true;
}

void MagneticAngleSensor::close_can_socket()
{
  if (can_socket_fd_ >= 0) {
    ::close(can_socket_fd_);
    can_socket_fd_ = -1;
  }
}

std::chrono::steady_clock::time_point MagneticAngleSensor::now_steady() const
{
  return std::chrono::steady_clock::now();
}

}  // namespace ros2_shoulder_sensor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_shoulder_sensor::MagneticAngleSensor,
  hardware_interface::SystemInterface)
