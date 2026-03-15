#include "ros2_shoulder_sensor/magnetic_angle_sensor_core.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace
{
using ros2_shoulder_sensor::MagneticAngleSensorCore;
using ros2_shoulder_sensor::MagneticAngleSensorFrame;
using ros2_shoulder_sensor::MagneticAngleSensorJointConfig;

MagneticAngleSensorFrame make_frame(
  const uint32_t can_id, const uint8_t seq,
  const int16_t angle_hundredths, const int16_t temp_hundredths)
{
  MagneticAngleSensorFrame frame;
  frame.can_id = can_id;
  frame.is_extended = true;
  frame.dlc = 5U;
  frame.data[0] = seq;
  frame.data[1] = static_cast<uint8_t>((static_cast<uint16_t>(angle_hundredths) >> 8U) & 0xFFU);
  frame.data[2] = static_cast<uint8_t>(static_cast<uint16_t>(angle_hundredths) & 0xFFU);
  frame.data[3] = static_cast<uint8_t>((static_cast<uint16_t>(temp_hundredths) >> 8U) & 0xFFU);
  frame.data[4] = static_cast<uint8_t>(static_cast<uint16_t>(temp_hundredths) & 0xFFU);
  return frame;
}

std::vector<MagneticAngleSensorJointConfig> two_joint_config()
{
  return {
    MagneticAngleSensorJointConfig{"left", 3U, false, 0.0},
    MagneticAngleSensorJointConfig{"right", 4U, true, 1.5},
  };
}

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

TEST(MagneticAngleSensorCoreTest, RejectsDuplicateNodeId)
{
  MagneticAngleSensorCore core;
  std::string err;

  const std::vector<MagneticAngleSensorJointConfig> cfg = {
    MagneticAngleSensorJointConfig{"j1", 2U, false, 0.0},
    MagneticAngleSensorJointConfig{"j2", 2U, false, 0.0},
  };

  EXPECT_FALSE(core.configure(cfg, 200U, 500U, &err));
  EXPECT_NE(err.find("Duplicate node_id"), std::string::npos);
}

TEST(MagneticAngleSensorCoreTest, IgnoresMalformedFrames)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(two_joint_config(), 200U, 500U, &err)) << err;

  const auto now = std::chrono::steady_clock::time_point{};
  core.mark_started(now);

  auto frame = make_frame(0x60U, 0U, 750, 1952);
  frame.is_extended = false;
  std::string diag;
  EXPECT_FALSE(core.process_frame(frame, now, &diag));

  frame.is_extended = true;
  frame.dlc = 8U;
  EXPECT_FALSE(core.process_frame(frame, now, &diag));

  frame.dlc = 5U;
  frame.can_id = 0x1234U;
  EXPECT_FALSE(core.process_frame(frame, now, &diag));
}

TEST(MagneticAngleSensorCoreTest, DecodesSignedPayload)
{
  const auto frame = make_frame(0x80U, 7U, static_cast<int16_t>(-123), static_cast<int16_t>(3210));
  uint8_t seq = 0U;
  double angle_deg = 0.0;
  double temp_c = 0.0;

  ASSERT_TRUE(MagneticAngleSensorCore::decode_payload(frame, &seq, &angle_deg, &temp_c));
  EXPECT_EQ(seq, 7U);
  EXPECT_DOUBLE_EQ(angle_deg, -1.23);
  EXPECT_DOUBLE_EQ(temp_c, 32.10);
}

TEST(MagneticAngleSensorCoreTest, DemuxesInterleavedNodesAndComputesStates)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(two_joint_config(), 200U, 500U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::string diag;
  EXPECT_TRUE(core.process_frame(make_frame(0x80U, 252U, 1035, 3212),
    t0 + std::chrono::milliseconds(10), &diag));
  EXPECT_TRUE(core.process_frame(make_frame(0x60U, 0U, 750, 1952),
    t0 + std::chrono::milliseconds(11), &diag));

  std::vector<double> pos;
  std::vector<double> temp;
  ASSERT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(20), &pos, &temp, &err)) << err;

  ASSERT_EQ(pos.size(), 2U);
  ASSERT_EQ(temp.size(), 2U);

  // left: node 3 (0x60) with no inversion/offset
  EXPECT_NEAR(pos[0], 7.5 * kDegToRad, 1e-12);
  EXPECT_DOUBLE_EQ(temp[0], 19.52);

  // right: node 4 (0x80) with inversion and +1.5 deg offset
  EXPECT_NEAR(pos[1], (-10.35 + 1.5) * kDegToRad, 1e-12);
  EXPECT_DOUBLE_EQ(temp[1], 32.12);
}

TEST(MagneticAngleSensorCoreTest, ReportsSequenceDiscontinuity)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(two_joint_config(), 200U, 500U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::string diag;
  EXPECT_TRUE(core.process_frame(make_frame(0x60U, 1U, 747, 1947),
    t0 + std::chrono::milliseconds(1), &diag));
  EXPECT_TRUE(diag.empty());

  EXPECT_TRUE(core.process_frame(make_frame(0x60U, 3U, 745, 1947),
    t0 + std::chrono::milliseconds(2), &diag));
  EXPECT_NE(diag.find("Sequence discontinuity"), std::string::npos);
}

TEST(MagneticAngleSensorCoreTest, FailsAfterStartupTimeoutWhenNoData)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(two_joint_config(), 200U, 100U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::vector<double> pos;
  std::vector<double> temp;
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(90), &pos, &temp, &err));

  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(120), &pos, &temp, &err));
  EXPECT_NE(err.find("startup timeout"), std::string::npos);
}

TEST(MagneticAngleSensorCoreTest, FailsWhenDataBecomesStale)
{
  MagneticAngleSensorCore core;
  std::string err;
  ASSERT_TRUE(core.configure(two_joint_config(), 50U, 500U, &err)) << err;

  const auto t0 = std::chrono::steady_clock::time_point{};
  core.mark_started(t0);

  std::string diag;
  EXPECT_TRUE(core.process_frame(make_frame(0x60U, 0U, 750, 1952),
    t0 + std::chrono::milliseconds(10), &diag));
  EXPECT_TRUE(core.process_frame(make_frame(0x80U, 252U, 1035, 3212),
    t0 + std::chrono::milliseconds(11), &diag));

  std::vector<double> pos;
  std::vector<double> temp;
  EXPECT_TRUE(core.compute_states(t0 + std::chrono::milliseconds(55), &pos, &temp, &err));

  EXPECT_FALSE(core.compute_states(t0 + std::chrono::milliseconds(70), &pos, &temp, &err));
  EXPECT_NE(err.find("Stale data"), std::string::npos);
}
