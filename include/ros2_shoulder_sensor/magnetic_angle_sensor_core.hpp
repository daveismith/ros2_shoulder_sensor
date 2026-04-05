#ifndef ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_
#define ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <unordered_set>

namespace ros2_shoulder_sensor
{

/// Configuration for one joint/sensor attached to the Cyphal bus.
/// Exactly one of has_node_id or has_unique_id must be true (or both, in which case
/// node_id takes effect immediately and unique_id is not used for discovery).
struct MagneticAngleSensorJointConfig
{
  std::string joint_name;

  /// When true, node_id is used directly without any GetInfo discovery.
  bool has_node_id{false};
  uint8_t node_id{0U};

  /// When true (and has_node_id is false), the driver listens for previously-unseen
  /// Cyphal nodes broadcasting angle measurements and sends a GetInfo request to each.
  /// The response unique_id is matched against this value to resolve the node_id.
  bool has_unique_id{false};
  std::array<uint8_t, 16> unique_id{};  ///< 128-bit unique-ID from GetInfo response.

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

  /// Reset timeouts and discovery state.  Call each time the hardware interface is activated.
  void mark_started(const std::chrono::steady_clock::time_point now);

  /// Returns true if source_node_id has not yet been matched to any joint AND at least one
  /// joint is still awaiting unique_id resolution AND we have not already sent a GetInfo
  /// request to source_node_id.
  bool should_query_getinfo(uint8_t source_node_id) const;

  /// Record that a GetInfo request has been dispatched to source_node_id so we do not
  /// send duplicate queries.
  void mark_getinfo_queried(uint8_t source_node_id);

  /// Called when a uavcan.node.GetInfo.1.0 response is received.
  /// Matches unique_id against all joints configured with has_unique_id=true that are
  /// still unresolved.  On a successful match, the joint's runtime node_id is set to
  /// source_node_id and angle/temperature messages from that node will be accepted.
  /// Returns true if a new match was made.
  bool on_getinfo_response(
    uint8_t source_node_id,
    const uint8_t unique_id[16],
    std::string * log_message);

  /// Process a uavcan.si.unit.angle.Scalar.1.0 message payload (4-byte IEEE-754 LE float32,
  /// radians).  Applies per-joint inversion and zero-offset before storing.
  /// Returns false when source_node_id is not matched to any joint or the payload is invalid.
  bool on_angle_message(
    uint8_t source_node_id,
    const uint8_t * payload,
    size_t len,
    const std::chrono::steady_clock::time_point now,
    std::string * diagnostic_message);

  /// Process a uavcan.si.unit.temperature.Scalar.1.0 message payload (4-byte IEEE-754 LE
  /// float32, Kelvin).  Silently ignored if source_node_id is not matched to any joint.
  void on_temperature_message(
    uint8_t source_node_id,
    const uint8_t * payload,
    size_t len,
    const std::chrono::steady_clock::time_point now);

  /// Output current joint states.
  /// temperatures_k values are in Kelvin (Cyphal SI unit; no conversion applied).
  /// Returns false if any angle data is missing or stale.
  bool compute_states(
    const std::chrono::steady_clock::time_point now,
    std::vector<double> * positions_rad,
    std::vector<double> * temperatures_k,
    std::string * error_message) const;

  size_t joint_count() const;

  // ---- Serialization helpers (public for unit testing) ----

  /// Deserialize a uavcan.si.unit.angle.Scalar.1.0 payload into radians.
  /// Payload must be exactly 4 bytes (IEEE-754 float32, little-endian).
  static bool deserialize_angle(const uint8_t * payload, size_t len, double * angle_rad);

  /// Deserialize a uavcan.si.unit.temperature.Scalar.1.0 payload into Kelvin.
  /// Payload must be exactly 4 bytes (IEEE-754 float32, little-endian).
  static bool deserialize_temperature(const uint8_t * payload, size_t len, double * temp_k);

  /// Extract the 16-byte unique_id from a uavcan.node.GetInfo.1.0 response payload.
  /// The unique_id field starts at byte offset 14 after three Version.1.0 structs and a
  /// uint64 vcs_revision_id.  Requires payload length >= 30.
  static bool extract_getinfo_unique_id(
    const uint8_t * payload, size_t len, uint8_t unique_id_out[16]);

private:
  struct JointRuntime
  {
    bool node_id_resolved{false};
    uint8_t resolved_node_id{0U};

    double position_rad{0.0};
    double temperature_k{0.0};

    bool angle_seen{false};
    bool temperature_seen{false};

    std::chrono::steady_clock::time_point last_angle_update{};
  };

  static double deg_to_rad(double angle_deg);

  std::vector<MagneticAngleSensorJointConfig> joint_configs_{};
  std::vector<JointRuntime> runtime_{};

  /// Maps resolved Cyphal source node_id → joint index for O(1) lookup in the hot path.
  std::unordered_map<uint8_t, size_t> node_id_to_joint_index_{};

  /// Tracks source node_ids already sent a GetInfo request to prevent duplicate queries.
  std::unordered_set<uint8_t> getinfo_queried_nodes_{};

  uint32_t stale_timeout_ms_{0U};
  uint32_t startup_timeout_ms_{0U};
  std::chrono::steady_clock::time_point started_at_{};
};

}  // namespace ros2_shoulder_sensor

#endif  // ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_CORE_HPP_
