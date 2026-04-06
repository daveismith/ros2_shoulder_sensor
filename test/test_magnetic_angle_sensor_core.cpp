// Unit tests for MagneticAngleSensorCore — Cyphal protocol implementation.
//
// The core is tested in isolation with synthetic message payloads, no hardware required.
// Temperature values are in Kelvin (Cyphal SI standard).

#include "ros2_shoulder_sensor/magnetic_angle_sensor_core.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace
{
using ros2_shoulder_sensor::MagneticAngleSensorCore;
using ros2_shoulder_sensor::MagneticAngleSensorJointConfig;

// ---- payload helpers ---------------------------------------------------------

/// Serialise a float32 into a 4-byte Cyphal angle/temperature payload.
std::array<uint8_t, 4> make_float32_payload(const float value)
{
  std::array<uint8_t, 4> buf{};
  std::memcpy(buf.data(), &value, 4U);
  return buf;
}

/// Build a minimal uavcan.node.GetInfo.1.0 response payload containing the given unique_id.
/// The first 14 bytes are the fixed header (3 × Version.1.0 + uint64 vcs_id), then 16 bytes uid.
std::array<uint8_t, 30> make_getinfo_payload(const uint8_t unique_id[16])
{
  std::array<uint8_t, 30> buf{};
  // Bytes 0-1:  protocol_version  (Version.1.0)
  // Bytes 2-3:  hardware_version  (Version.1.0)
  // Bytes 4-5:  software_version  (Version.1.0)
  // Bytes 6-13: software_vcs_revision_id (uint64 LE)
  // Bytes 14-29: unique_id (uint8[16])
  std::memcpy(buf.data() + 14U, unique_id, 16U);
  return buf;
}

// ---- joint config builders ---------------------------------------------------

/// Joint with a statically-configured node_id (no discovery needed).
MagneticAngleSensorJointConfig static_joint(
  const std::string & name, uint8_t node_id,
  bool invert = false, double offset_deg = 0.0)
{
  MagneticAngleSensorJointConfig cfg;
  cfg.joint_name = name;
  cfg.has_node_id = true;
  cfg.node_id = node_id;
  cfg.invert_position = invert;
  cfg.zero_offset_deg = offset_deg;
  return cfg;
}

/// Joint that must discover its node_id via unique_id / GetInfo.
MagneticAngleSensorJointConfig discovery_joint(
  const std::string & name,
  const uint8_t unique_id[16],
  bool invert = false, double offset_deg = 0.0)
{
  MagneticAngleSensorJointConfig cfg;
  cfg.joint_name = name;
  cfg.has_unique_id = true;
  std::memcpy(cfg.unique_id.data(), unique_id, 16U);
  cfg.invert_position = invert;
  cfg.zero_offset_deg = offset_deg;
  return cfg;
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Reference unique_ids for tests.
constexpr uint8_t kUid1[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10};
constexpr uint8_t kUid2[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22,
                                0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00};

}  // namespace

// =============================================================================
// Serialization helpers
// =============================================================================

TEST(DeserializeAngle, ValidPositiveValue)
{
  const auto buf = make_float32_payload(1.5707963f);  // pi/2
  double angle = 0.0;
  EXPECT_TRUE(MagneticAngleSensorCore::deserialize_angle(buf.data(), buf.size(), &angle));
  EXPECT_NEAR(angle, 1.5707963, 1e-6);
}

TEST(DeserializeAngle, ValidNegativeValue)
{
  const auto buf = make_float32_payload(-3.14159f);
  double angle = 0.0;
  EXPECT_TRUE(MagneticAngleSensorCore::deserialize_angle(buf.data(), buf.size(), &angle));
  EXPECT_NEAR(angle, -3.14159, 1e-5);
}

TEST(DeserializeAngle, ZeroAngle)
{
  const auto buf = make_float32_payload(0.0f);
  double angle = 999.0;
  EXPECT_TRUE(MagneticAngleSensorCore::deserialize_angle(buf.data(), buf.size(), &angle));
  EXPECT_DOUBLE_EQ(angle, 0.0);
}

TEST(DeserializeAngle, RejectNaN)
{
  const auto buf = make_float32_payload(std::numeric_limits<float>::quiet_NaN());
  double angle = 0.0;
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_angle(buf.data(), buf.size(), &angle));
}

TEST(DeserializeAngle, RejectInfinity)
{
  const auto buf = make_float32_payload(std::numeric_limits<float>::infinity());
  double angle = 0.0;
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_angle(buf.data(), buf.size(), &angle));
}

TEST(DeserializeAngle, RejectShortPayload)
{
  const uint8_t buf[3] = {0, 0, 0};
  double angle = 0.0;
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_angle(buf, 3, &angle));
}

TEST(DeserializeAngle, RejectNullPointers)
{
  const auto buf = make_float32_payload(1.0f);
  double angle = 0.0;
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_angle(nullptr, 4, &angle));
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_angle(buf.data(), 4, nullptr));
}

TEST(DeserializeTemperature, ValidKelvin)
{
  // 293.15 K = 20 °C (body temperature not relevant here, just a valid float)
  const auto buf = make_float32_payload(293.15f);
  double temp = 0.0;
  EXPECT_TRUE(MagneticAngleSensorCore::deserialize_temperature(buf.data(), buf.size(), &temp));
  EXPECT_NEAR(temp, 293.15, 1e-3);
}

TEST(DeserializeTemperature, RejectNaN)
{
  const auto buf = make_float32_payload(std::numeric_limits<float>::quiet_NaN());
  double temp = 0.0;
  EXPECT_FALSE(MagneticAngleSensorCore::deserialize_temperature(buf.data(), buf.size(), &temp));
}

TEST(ExtractGetInfoUniqueId, CorrectOffset)
{
  const auto payload = make_getinfo_payload(kUid1);
  uint8_t out[16]{};
  EXPECT_TRUE(MagneticAngleSensorCore::extract_getinfo_unique_id(
      payload.data(), payload.size(), out));
  EXPECT_EQ(std::memcmp(out, kUid1, 16U), 0);
}

TEST(ExtractGetInfoUniqueId, RejectShortPayload)
{
  std::array<uint8_t, 29> short_buf{};
  uint8_t out[16]{};
  EXPECT_FALSE(
    MagneticAngleSensorCore::extract_getinfo_unique_id(short_buf.data(), short_buf.size(), out));
}

TEST(ExtractGetInfoUniqueId, AcceptsLongerPayload)
{
  // GetInfo responses can be longer (variable-length fields after unique_id).
  std::array<uint8_t, 100> long_buf{};
  std::memcpy(long_buf.data() + 14U, kUid2, 16U);
  uint8_t out[16]{};
  EXPECT_TRUE(
    MagneticAngleSensorCore::extract_getinfo_unique_id(long_buf.data(), long_buf.size(), out));
  EXPECT_EQ(std::memcmp(out, kUid2, 16U), 0);
}

// =============================================================================
// configure()
// =============================================================================

TEST(Configure, RejectNoIdentificationOnJoint)
{
  MagneticAngleSensorCore core;
  std::string err;
  MagneticAngleSensorJointConfig bad;
  bad.joint_name = "bare";
  // neither has_node_id nor has_unique_id set
  EXPECT_FALSE(core.configure({bad}, 200U, 500U, &err));
  EXPECT_FALSE(err.empty());
}

TEST(Configure, RejectDuplicateNodeId)
{
  MagneticAngleSensorCore core;
  std::string err;
  EXPECT_FALSE(core.configure(
      {static_joint("j1", 1U), static_joint("j2", 1U)}, 200U, 500U, &err));
  EXPECT_NE(err.find("Duplicate"), std::string::npos);
}

TEST(Configure, AcceptStaticNodeIds)
{
  MagneticAngleSensorCore core;
  std::string err;
  EXPECT_TRUE(core.configure(
      {static_joint("left", 1U), static_joint("right", 2U)}, 200U, 5000U, &err)) << err;
  EXPECT_EQ(core.joint_count(), 2U);
}

TEST(Configure, AcceptDiscoveryJoint)
{
  MagneticAngleSensorCore core;
  std::string err;
  EXPECT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;
}

// =============================================================================
// on_angle_message() — static node_id joints
// =============================================================================

TEST(AngleMessage, DecodesAndStoresWithInversionAndOffset)
{
  MagneticAngleSensorCore core;
  std::string err;
  // right: invert=true, offset=+5 deg
  ASSERT_TRUE(core.configure(
      {static_joint("left", 1U), static_joint("right", 2U, true, 5.0)},
      200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  const float angle_left_rad = 0.5f;
  const auto left_payload = make_float32_payload(angle_left_rad);
  std::string diag;
  EXPECT_TRUE(core.on_angle_message(
      1U, left_payload.data(), left_payload.size(),
      t0 + std::chrono::milliseconds(5), &diag));

  const float angle_right_rad = 1.0f;
  const auto right_payload = make_float32_payload(angle_right_rad);
  EXPECT_TRUE(core.on_angle_message(
      2U, right_payload.data(), right_payload.size(),
      t0 + std::chrono::milliseconds(6), &diag));

  std::vector<double> pos, temp;
  ASSERT_TRUE(core.compute_states(
      t0 + std::chrono::milliseconds(10), &pos, &temp, &err)) << err;

  ASSERT_EQ(pos.size(), 2U);
  // left: no inversion, no offset
  EXPECT_NEAR(pos[0], static_cast<double>(angle_left_rad), 1e-6);
  // right: inverted (-1.0 rad) + 5° offset
  EXPECT_NEAR(pos[1], -1.0 + 5.0 * kDegToRad, 1e-6);
}

TEST(AngleMessage, IgnoresUnknownNodeId)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  const auto pay = make_float32_payload(1.0f);
  std::string diag;
  EXPECT_FALSE(core.on_angle_message(99U, pay.data(), pay.size(), t0, &diag));
}

TEST(AngleMessage, RejectsInvalidPayload)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // 3 bytes — too short for float32
  const uint8_t short_pay[3] = {0, 0, 0};
  std::string diag;
  EXPECT_FALSE(core.on_angle_message(1U, short_pay, 3, t0, &diag));
}

TEST(AngleMessage, RejectsNaNPayload)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  const auto nan_pay = make_float32_payload(std::numeric_limits<float>::quiet_NaN());
  std::string diag;
  EXPECT_FALSE(core.on_angle_message(1U, nan_pay.data(), nan_pay.size(), t0, &diag));
}

// =============================================================================
// on_temperature_message()
// =============================================================================

TEST(TemperatureMessage, StoresKelvinValue)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Provide an angle first so compute_states passes
  const auto angle_pay = make_float32_payload(0.0f);
  std::string diag;
  EXPECT_TRUE(core.on_angle_message(
      1U, angle_pay.data(), angle_pay.size(), t0 + std::chrono::milliseconds(1), &diag));

  const auto temp_pay = make_float32_payload(295.0f);
  core.on_temperature_message(1U, temp_pay.data(), temp_pay.size(),
    t0 + std::chrono::milliseconds(2));

  std::vector<double> pos, temp;
  ASSERT_TRUE(core.compute_states(
      t0 + std::chrono::milliseconds(5), &pos, &temp, &err)) << err;
  ASSERT_EQ(temp.size(), 1U);
  EXPECT_NEAR(temp[0], 295.0, 1e-3);
}

TEST(TemperatureMessage, IgnoresUnknownNode)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Should not crash but also not update state
  const auto pay = make_float32_payload(300.0f);
  core.on_temperature_message(99U, pay.data(), pay.size(), t0);
}

// =============================================================================
// Node discovery via unique_id / GetInfo
// =============================================================================

TEST(Discovery, ShouldQueryGetinfoForUnknownAngleBroadcaster)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Node 7 is broadcasting but not matched yet — should trigger GetInfo
  EXPECT_TRUE(core.should_query_getinfo(7U));
}

TEST(Discovery, ShouldNotQueryIfAlreadyMatchedByNodeId)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Node 1 is statically configured — no GetInfo needed
  EXPECT_FALSE(core.should_query_getinfo(1U));
}

TEST(Discovery, ShouldNotQueryIfAlreadyQueried)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  EXPECT_TRUE(core.should_query_getinfo(5U));
  core.mark_getinfo_queried(5U);
  EXPECT_FALSE(core.should_query_getinfo(5U));
}

TEST(Discovery, ShouldNotQueryIfNoDiscoveryJoints)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U), static_joint("right", 2U)},
      200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // All joints are statically configured; no discovery needed for any node
  EXPECT_FALSE(core.should_query_getinfo(99U));
}

TEST(Discovery, MatchesUniqueIdFromGetInfoResponse)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::string log;
  EXPECT_TRUE(core.on_getinfo_response(3U, kUid1, &log));
  EXPECT_NE(log.find("matched"), std::string::npos);
  EXPECT_NE(log.find("left"), std::string::npos);

  // Node 3 is now accepted for angle messages
  const auto angle_pay = make_float32_payload(1.0f);
  std::string diag;
  EXPECT_TRUE(core.on_angle_message(
      3U, angle_pay.data(), angle_pay.size(),
      t0 + std::chrono::milliseconds(5), &diag));
}

TEST(Discovery, DoesNotMatchWrongUniqueId)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // kUid2 does not match kUid1
  std::string log;
  EXPECT_FALSE(core.on_getinfo_response(4U, kUid2, &log));
}

TEST(Discovery, TwoDiscoveryJointsMatchIndependently)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(
      {discovery_joint("left", kUid1), discovery_joint("right", kUid2)},
      200U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::string log;
  EXPECT_TRUE(core.on_getinfo_response(1U, kUid1, &log));
  EXPECT_TRUE(core.on_getinfo_response(2U, kUid2, &log));

  // Both joints now receive data
  const auto ap = make_float32_payload(0.5f);
  std::string diag;
  EXPECT_TRUE(core.on_angle_message(1U, ap.data(), ap.size(),
      t0 + std::chrono::milliseconds(5), &diag));
  EXPECT_TRUE(core.on_angle_message(2U, ap.data(), ap.size(),
      t0 + std::chrono::milliseconds(5), &diag));

  std::vector<double> pos, temp;
  ASSERT_TRUE(core.compute_states(
      t0 + std::chrono::milliseconds(10), &pos, &temp, &err)) << err;
  EXPECT_EQ(pos.size(), 2U);
}

TEST(Discovery, ResetOnMarkStarted)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 5000U, &err)) << err;

  auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Discover and match node 5
  core.mark_getinfo_queried(5U);
  std::string log;
  core.on_getinfo_response(5U, kUid1, &log);

  // Re-activate (mark_started again) — discovery state should reset
  t0 = t0 + std::chrono::seconds(10);
  core.mark_started(t0);

  // Node 5 should be queryable again (queried-set was cleared)
  EXPECT_TRUE(core.should_query_getinfo(5U));
}

// =============================================================================
// compute_states() — timeout behaviour
// =============================================================================

TEST(ComputeStates, StartupTimeoutWhenNoData)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 100U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::vector<double> pos, temp;
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(90), &pos, &temp, &err));
  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(110), &pos, &temp, &err));
  EXPECT_NE(err.find("startup timeout"), std::string::npos);
}

TEST(ComputeStates, StaleAngleDataCausesError)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 50U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // Provide fresh angle data
  const auto pay = make_float32_payload(1.0f);
  std::string diag;
  EXPECT_TRUE(core.on_angle_message(
      1U, pay.data(), pay.size(), t0 + std::chrono::milliseconds(5), &diag));

  // Within stale window — OK
  std::vector<double> pos, temp;
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(50), &pos, &temp, &err));

  // Past stale window — ERROR
  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(70), &pos, &temp, &err));
  EXPECT_NE(err.find("Stale"), std::string::npos);
}

TEST(ComputeStates, StartupGracePeriodReturnsDefaultValues)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({static_joint("left", 1U)}, 200U, 500U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::vector<double> pos, temp;
  // Within startup window, no data yet — should succeed with defaults
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(100), &pos, &temp, &err)) << err;
  ASSERT_EQ(pos.size(), 1U);
  EXPECT_DOUBLE_EQ(pos[0], 0.0);
}

TEST(ComputeStates, MultiJointFailsIfAnyStale)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(
      {static_joint("left", 1U), static_joint("right", 2U)},
      50U, 5000U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  const auto pay = make_float32_payload(0.0f);
  std::string diag;
  core.on_angle_message(1U, pay.data(), pay.size(), t0 + std::chrono::milliseconds(5), &diag);
  core.on_angle_message(2U, pay.data(), pay.size(), t0 + std::chrono::milliseconds(5), &diag);

  std::vector<double> pos, temp;
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(40), &pos, &temp, &err));
  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(65), &pos, &temp, &err));
}

TEST(ComputeStates, DiscoveryJointStartupTimeoutMentionsDiscovery)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure({discovery_joint("left", kUid1)}, 200U, 100U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  // No GetInfo response, no angle data — startup timeout with discovery hint
  std::vector<double> pos, temp;
  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(200), &pos, &temp, &err));
  EXPECT_NE(err.find("unique_id"), std::string::npos);
}
