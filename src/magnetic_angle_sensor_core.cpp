#include "ros2_shoulder_sensor/magnetic_angle_sensor_core.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace ros2_shoulder_sensor
{

bool MagneticAngleSensorCore::configure(
  const std::vector<MagneticAngleSensorJointConfig> & joint_configs,
  const uint32_t stale_timeout_ms,
  const uint32_t startup_timeout_ms,
  std::string * error_message)
{
  joint_configs_ = joint_configs;
  runtime_.assign(joint_configs.size(), JointRuntime{});
  node_id_to_joint_index_.clear();
  getinfo_queried_nodes_.clear();

  stale_timeout_ms_ = stale_timeout_ms;
  startup_timeout_ms_ = startup_timeout_ms;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & cfg = joint_configs_[i];

    if (!cfg.has_node_id && !cfg.has_unique_id) {
      if (error_message != nullptr) {
        *error_message = "Joint '" + cfg.joint_name +
          "': at least one of 'node_id' or 'unique_id' must be specified";
      }
      return false;
    }

    if (cfg.has_node_id) {
      const auto inserted = node_id_to_joint_index_.emplace(cfg.node_id, i);
      if (!inserted.second) {
        if (error_message != nullptr) {
          std::ostringstream oss;
          oss << "Duplicate node_id " << static_cast<int>(cfg.node_id)
              << " in joint '" << cfg.joint_name << "'";
          *error_message = oss.str();
        }
        return false;
      }
      runtime_[i].node_id_resolved = true;
      runtime_[i].resolved_node_id = cfg.node_id;
    }
    // Joints with only unique_id start unresolved; their node_id is filled in
    // by on_getinfo_response() after GetInfo discovery.
  }

  return true;
}

void MagneticAngleSensorCore::mark_started(const std::chrono::steady_clock::time_point now)
{
  started_at_ = now;
  getinfo_queried_nodes_.clear();

  // Reset runtime state for all joints so startup timeout applies fresh.
  for (size_t i = 0; i < runtime_.size(); ++i) {
    auto & rt = runtime_[i];
    const auto & cfg = joint_configs_[i];

    rt.angle_seen = false;
    rt.temperature_seen = false;
    rt.position_rad = 0.0;
    rt.temperature_k = 0.0;
    rt.last_angle_update = {};

    // Re-resolve static node_ids; clear dynamic resolutions so they are re-discovered.
    if (cfg.has_node_id) {
      rt.node_id_resolved = true;
      rt.resolved_node_id = cfg.node_id;
    } else {
      if (rt.node_id_resolved) {
        node_id_to_joint_index_.erase(rt.resolved_node_id);
      }
      rt.node_id_resolved = false;
      rt.resolved_node_id = 0U;
    }
  }
}

bool MagneticAngleSensorCore::should_query_getinfo(const uint8_t source_node_id) const
{
  if (node_id_to_joint_index_.count(source_node_id) > 0) {
    return false;
  }
  if (getinfo_queried_nodes_.count(source_node_id) > 0) {
    return false;
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    if (joint_configs_[i].has_unique_id && !runtime_[i].node_id_resolved) {
      return true;
    }
  }
  return false;
}

void MagneticAngleSensorCore::mark_getinfo_queried(const uint8_t source_node_id)
{
  getinfo_queried_nodes_.insert(source_node_id);
}

bool MagneticAngleSensorCore::on_getinfo_response(
  const uint8_t source_node_id,
  const uint8_t unique_id[16],
  std::string * log_message)
{
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto & cfg = joint_configs_[i];
    auto & rt = runtime_[i];

    if (!cfg.has_unique_id || rt.node_id_resolved) {
      continue;
    }

    if (std::memcmp(cfg.unique_id.data(), unique_id, 16U) != 0) {
      continue;
    }

    const auto existing = node_id_to_joint_index_.find(source_node_id);
    if (existing != node_id_to_joint_index_.end() && existing->second != i) {
      if (log_message != nullptr) {
        std::ostringstream oss;
        oss << "Node " << static_cast<int>(source_node_id)
            << " unique_id matches joint '" << cfg.joint_name
            << "' but is already mapped to joint '"
            << joint_configs_[existing->second].joint_name << "'; ignoring";
        *log_message = oss.str();
      }
      return false;
    }

    rt.node_id_resolved = true;
    rt.resolved_node_id = source_node_id;
    node_id_to_joint_index_.emplace(source_node_id, i);

    if (log_message != nullptr) {
      std::ostringstream oss;
      char uid_hex[33] = {0};
      for (int b = 0; b < 16; ++b) {
        std::snprintf(uid_hex + b * 2, 3, "%02x", unique_id[b]);
      }
      oss << "Node " << static_cast<int>(source_node_id)
          << " matched to joint '" << cfg.joint_name
          << "' via unique_id 0x" << uid_hex;
      *log_message = oss.str();
    }
    return true;
  }
  return false;
}

bool MagneticAngleSensorCore::on_angle_message(
  const uint8_t source_node_id,
  const uint8_t * payload,
  const size_t len,
  const std::chrono::steady_clock::time_point now,
  std::string * diagnostic_message)
{
  const auto it = node_id_to_joint_index_.find(source_node_id);
  if (it == node_id_to_joint_index_.end()) {
    return false;
  }

  double angle_rad = 0.0;
  if (!deserialize_angle(payload, len, &angle_rad)) {
    return false;
  }

  const size_t joint_index = it->second;
  const auto & config = joint_configs_[joint_index];
  auto & state = runtime_[joint_index];

  const double raw_rad = config.invert_position ? -angle_rad : angle_rad;
  state.position_rad = raw_rad + deg_to_rad(config.zero_offset_deg);
  state.last_angle_update = now;
  state.angle_seen = true;

  if (diagnostic_message != nullptr) {
    diagnostic_message->clear();
  }
  return true;
}

void MagneticAngleSensorCore::on_temperature_message(
  const uint8_t source_node_id,
  const uint8_t * payload,
  const size_t len,
  const std::chrono::steady_clock::time_point now)
{
  const auto it = node_id_to_joint_index_.find(source_node_id);
  if (it == node_id_to_joint_index_.end()) {
    return;
  }

  double temp_k = 0.0;
  if (!deserialize_temperature(payload, len, &temp_k)) {
    return;
  }

  auto & state = runtime_[it->second];
  state.temperature_k = temp_k;
  state.temperature_seen = true;
  (void)now;
}

bool MagneticAngleSensorCore::compute_states(
  const std::chrono::steady_clock::time_point now,
  std::vector<double> * positions_rad,
  std::vector<double> * temperatures_k,
  std::string * error_message) const
{
  if (positions_rad == nullptr || temperatures_k == nullptr) {
    if (error_message != nullptr) {
      *error_message = "Output containers cannot be null";
    }
    return false;
  }

  positions_rad->resize(runtime_.size());
  temperatures_k->resize(runtime_.size());

  for (size_t i = 0; i < runtime_.size(); ++i) {
    const auto & config = joint_configs_[i];
    const auto & state = runtime_[i];

    const int node_id_for_log = state.node_id_resolved ?
      static_cast<int>(state.resolved_node_id) : -1;

    if (!state.angle_seen) {
      const auto startup_elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - started_at_);
      if (startup_elapsed > std::chrono::milliseconds(startup_timeout_ms_)) {
        if (error_message != nullptr) {
          std::ostringstream oss;
          oss << "No angle message received for joint '" << config.joint_name
              << "' (node_id " << node_id_for_log
              << ") within startup timeout " << startup_timeout_ms_ << " ms";
          if (!state.node_id_resolved) {
            oss << " [node_id not yet discovered from unique_id]";
          }
          *error_message = oss.str();
        }
        return false;
      }

      (*positions_rad)[i] = state.position_rad;
      (*temperatures_k)[i] = state.temperature_k;
      continue;
    }

    const auto age =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - state.last_angle_update);
    if (age > std::chrono::milliseconds(stale_timeout_ms_)) {
      if (error_message != nullptr) {
        std::ostringstream oss;
        oss << "Stale angle data for joint '" << config.joint_name
            << "' (node_id " << node_id_for_log
            << "): age " << age.count()
            << " ms exceeds timeout " << stale_timeout_ms_ << " ms";
        *error_message = oss.str();
      }
      return false;
    }

    (*positions_rad)[i] = state.position_rad;
    (*temperatures_k)[i] = state.temperature_k;
  }

  return true;
}

size_t MagneticAngleSensorCore::joint_count() const
{
  return joint_configs_.size();
}

bool MagneticAngleSensorCore::deserialize_angle(
  const uint8_t * payload, const size_t len, double * angle_rad)
{
  if (len < 4U || payload == nullptr || angle_rad == nullptr) {
    return false;
  }
  float val;
  std::memcpy(&val, payload, sizeof(val));
  if (!std::isfinite(val)) {
    return false;
  }
  *angle_rad = static_cast<double>(val);
  return true;
}

bool MagneticAngleSensorCore::deserialize_temperature(
  const uint8_t * payload, const size_t len, double * temp_k)
{
  if (len < 4U || payload == nullptr || temp_k == nullptr) {
    return false;
  }
  float val;
  std::memcpy(&val, payload, sizeof(val));
  if (!std::isfinite(val)) {
    return false;
  }
  *temp_k = static_cast<double>(val);
  return true;
}

bool MagneticAngleSensorCore::extract_getinfo_unique_id(
  const uint8_t * payload, const size_t len, uint8_t unique_id_out[16])
{
  constexpr size_t kUniqueIdOffset = 14U;
  constexpr size_t kMinLen = kUniqueIdOffset + 16U;

  if (payload == nullptr || unique_id_out == nullptr || len < kMinLen) {
    return false;
  }
  std::memcpy(unique_id_out, payload + kUniqueIdOffset, 16U);
  return true;
}

double MagneticAngleSensorCore::deg_to_rad(const double angle_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  return angle_deg * kPi / 180.0;
}

}  // namespace ros2_shoulder_sensor
