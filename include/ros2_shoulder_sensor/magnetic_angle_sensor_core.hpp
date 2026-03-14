#ifndef ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_
#define ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_shoulder_sensor
{

struct MagneticAngleSensorFrame
{
  uint32_t can_id{0U};
  bool is_extended{false};
  uint8_t dlc{0U};
  std::array<uint8_t, 8> data{};
};

struct MagneticAngleSensorJointConfig
{
  std::string joint_name;
  uint8_t node_id{0U};
  bool invert_position{false};
  double zero_offset_deg{0.0};
};

class MagneticAngleSensorCore
{
public:
  bool configure(
    const std::vector<MagneticAngleSensorJointConfig> & joint_configs,
    uint32_t stale_timeout_ms,
    uint32_t startup_timeout_ms,
    std::string * error_message);

  void mark_started(const std::chrono::steady_clock::time_point now);

  bool process_frame(
    const MagneticAngleSensorFrame & frame,
    const std::chrono::steady_clock::time_point now,
    std::string * diagnostic_message);

  bool compute_states(
    const std::chrono::steady_clock::time_point now,
    std::vector<double> * positions_rad,
    std::vector<double> * temperatures_c,
    std::string * error_message) const;

  size_t joint_count() const;

  static bool decode_payload(
    const MagneticAngleSensorFrame & frame,
    uint8_t * sequence,
    double * angle_deg,
    double * temperature_c);

private:
  struct JointRuntime
  {
    double position_rad{0.0};
    double temperature_c{0.0};
    bool seen{false};
    uint8_t last_sequence{0U};
    std::chrono::steady_clock::time_point last_update{};
  };

  static uint32_t expected_can_id_for_node(const uint8_t node_id);
  static double deg_to_rad(const double angle_deg);

  std::vector<MagneticAngleSensorJointConfig> joint_configs_{};
  std::vector<JointRuntime> runtime_{};
  std::unordered_map<uint32_t, size_t> id_to_joint_index_{};

  uint32_t stale_timeout_ms_{0U};
  uint32_t startup_timeout_ms_{0U};
  std::chrono::steady_clock::time_point started_at_{};
};

}  // namespace ros2_shoulder_sensor

#endif  // ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_
