#include "ros2_shoulder_sensor/magnetic_angle_sensor_core.hpp"

#include <cmath>
#include <sstream>

namespace ros2_shoulder_sensor
{

namespace
{
constexpr uint32_t kAddressMask = 0x1FU;
constexpr uint32_t kAddressZero = 0U;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

bool MagneticAngleSensorCore::configure(
  const std::vector<MagneticAngleSensorJointConfig> & joint_configs,
  const uint32_t stale_timeout_ms,
  const uint32_t startup_timeout_ms,
  std::string * error_message)
{
  joint_configs_ = joint_configs;
  runtime_.assign(joint_configs.size(), JointRuntime{});
  id_to_joint_index_.clear();

  stale_timeout_ms_ = stale_timeout_ms;
  startup_timeout_ms_ = startup_timeout_ms;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const uint32_t expected_id = expected_can_id_for_node(joint_configs_[i].node_id);
    if ((expected_id & kAddressMask) != kAddressZero) {
      if (error_message != nullptr) {
        *error_message = "Only CAN address 0 is supported";
      }
      return false;
    }

    const auto inserted = id_to_joint_index_.emplace(expected_id, i);
    if (!inserted.second) {
      if (error_message != nullptr) {
        std::ostringstream oss;
        oss << "Duplicate node_id (CAN ID collision) for node "
            << static_cast<int>(joint_configs_[i].node_id);
        *error_message = oss.str();
      }
      return false;
    }
  }

  return true;
}

void MagneticAngleSensorCore::mark_started(const std::chrono::steady_clock::time_point now)
{
  started_at_ = now;
}

bool MagneticAngleSensorCore::process_frame(
  const MagneticAngleSensorFrame & frame,
  const std::chrono::steady_clock::time_point now,
  std::string * diagnostic_message)
{
  if (!frame.is_extended || frame.dlc != 5U) {
    return false;
  }

  const auto it = id_to_joint_index_.find(frame.can_id);
  if (it == id_to_joint_index_.end()) {
    return false;
  }

  uint8_t sequence = 0U;
  double angle_deg = 0.0;
  double temperature_c = 0.0;
  if (!decode_payload(frame, &sequence, &angle_deg, &temperature_c)) {
    return false;
  }

  const size_t joint_index = it->second;
  const auto & config = joint_configs_[joint_index];
  auto & state = runtime_[joint_index];

  if (state.seen) {
    const uint8_t expected_seq = static_cast<uint8_t>(state.last_sequence + 1U);
    if (expected_seq != sequence && diagnostic_message != nullptr) {
      std::ostringstream oss;
      oss << "Sequence discontinuity for joint '" << config.joint_name
          << "' (node " << static_cast<int>(config.node_id)
          << "): expected " << static_cast<int>(expected_seq)
          << ", got " << static_cast<int>(sequence);
      *diagnostic_message = oss.str();
    }
  }

  const double normalized_angle_deg =
    (config.invert_position ? -angle_deg : angle_deg) + config.zero_offset_deg;

  state.position_rad = deg_to_rad(normalized_angle_deg);
  state.temperature_c = temperature_c;
  state.last_sequence = sequence;
  state.last_update = now;
  state.seen = true;

  return true;
}

bool MagneticAngleSensorCore::compute_states(
  const std::chrono::steady_clock::time_point now,
  std::vector<double> * positions_rad,
  std::vector<double> * temperatures_c,
  std::string * error_message) const
{
  if (positions_rad == nullptr || temperatures_c == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Output containers cannot be null";
    }
    return false;
  }

  positions_rad->resize(runtime_.size());
  temperatures_c->resize(runtime_.size());

  for (size_t i = 0; i < runtime_.size(); ++i) {
    const auto & config = joint_configs_[i];
    const auto & state = runtime_[i];

    if (!state.seen) {
      const auto startup_elapsed_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now - started_at_).count());
      if (startup_elapsed_ms > startup_timeout_ms_) {
        if (error_message != nullptr) {
          std::ostringstream oss;
          oss << "No message received for joint '" << config.joint_name
              << "' (node " << static_cast<int>(config.node_id)
              << ") within startup timeout " << startup_timeout_ms_ << " ms";
          *error_message = oss.str();
        }
        return false;
      }

      // Startup grace period still active.
      (*positions_rad)[i] = state.position_rad;
      (*temperatures_c)[i] = state.temperature_c;
      continue;
    }

    const auto age_ms = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_update).count());
    if (age_ms > stale_timeout_ms_) {
      if (error_message != nullptr) {
        std::ostringstream oss;
        oss << "Stale data for joint '" << config.joint_name
            << "' (node " << static_cast<int>(config.node_id)
            << "): age " << age_ms
            << " ms exceeds timeout " << stale_timeout_ms_ << " ms";
        *error_message = oss.str();
      }
      return false;
    }

    (*positions_rad)[i] = state.position_rad;
    (*temperatures_c)[i] = state.temperature_c;
  }

  return true;
}

size_t MagneticAngleSensorCore::joint_count() const
{
  return joint_configs_.size();
}

bool MagneticAngleSensorCore::decode_payload(
  const MagneticAngleSensorFrame & frame,
  uint8_t * sequence,
  double * angle_deg,
  double * temperature_c)
{
  if (frame.dlc != 5U || sequence == nullptr || angle_deg == nullptr || temperature_c == nullptr) {
    return false;
  }

  *sequence = frame.data[0];

  const auto angle_raw = static_cast<int16_t>(
    (static_cast<uint16_t>(frame.data[1]) << 8U) |
    static_cast<uint16_t>(frame.data[2]));

  const auto temperature_raw = static_cast<int16_t>(
    (static_cast<uint16_t>(frame.data[3]) << 8U) |
    static_cast<uint16_t>(frame.data[4]));

  *angle_deg = static_cast<double>(angle_raw) / 100.0;
  *temperature_c = static_cast<double>(temperature_raw) / 100.0;

  return true;
}

uint32_t MagneticAngleSensorCore::expected_can_id_for_node(const uint8_t node_id)
{
  return static_cast<uint32_t>(node_id) << 5U;
}

double MagneticAngleSensorCore::deg_to_rad(const double angle_deg)
{
  return angle_deg * (kPi / 180.0);
}

}  // namespace ros2_shoulder_sensor
