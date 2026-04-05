#ifndef ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
#define ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// Vendored Cyphal/CAN transport library (MIT) — https://github.com/OpenCyphal/libcanard
#include "libcanard/canard.h"
// Deterministic O(1) heap allocator for libcanard (MIT) — https://github.com/pavel-kirienko/o1heap
#include "o1heap/o1heap.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

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
  // ---- state interfaces exported to ros2_control ----
  std::vector<double> positions_{};
  std::vector<double> temperatures_{};
  hardware_interface::HardwareInfo info_{};

  // ---- configuration parameters ----
  std::string can_interface_{};
  uint16_t angle_subject_id_{6144U};
  uint16_t temperature_subject_id_{6145U};
  uint8_t local_node_id_{100U};
  uint32_t stale_timeout_ms_{200U};
  uint32_t startup_timeout_ms_{5000U};

  // ---- CAN socket ----
  int can_socket_fd_{-1};

  // ---- libcanard instance and deterministic memory pool ----
  static constexpr size_t kHeapArenaSize = 16384U;
  alignas(O1HEAP_ALIGNMENT) uint8_t heap_arena_[kHeapArenaSize]{};
  O1HeapInstance * heap_{nullptr};
  CanardMemoryResource memory_resource_{};
  CanardInstance canard_{};

  // ---- TX queue (one per CAN interface; initialised in on_activate) ----
  CanardTxQueue tx_queue_{};
  bool tx_queue_initialized_{false};

  // ---- RX subscriptions (registered in on_init, persist across activate cycles) ----
  CanardRxSubscription angle_subscription_{};
  CanardRxSubscription temperature_subscription_{};
  CanardRxSubscription getinfo_response_subscription_{};

  // ---- GetInfo TX transfer-ID counters, one counter per target node_id ----
  std::unordered_map<uint8_t, CanardTransferID> getinfo_tx_ids_{};

  // ---- sensor core (pure logic, no I/O) ----
  MagneticAngleSensorCore core_{};

  // ---- helpers ----
  bool open_can_socket();
  void close_can_socket();
  void drain_tx_queue();

  std::chrono::steady_clock::time_point now_steady() const;
  CanardMicrosecond now_usec() const;

  void send_getinfo_request(uint8_t target_node_id);
  void transmit_pending_frames();

  void handle_angle_transfer(const CanardRxTransfer & transfer);
  void handle_temperature_transfer(const CanardRxTransfer & transfer);
  void handle_getinfo_response(const CanardRxTransfer & transfer);

  static void * cyphal_allocate(void * user_reference, size_t size);
  static void cyphal_deallocate(void * user_reference, size_t size, void * pointer);
};

}  // namespace ros2_shoulder_sensor

#endif  // ROS2_SHOULDER_SENSOR__MAGNETIC_ANGLE_SENSOR_HPP_
