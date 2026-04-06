#include "ros2_shoulder_sensor/magnetic_angle_sensor.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_shoulder_sensor
{

// Maximum CAN frames processed per read() call to bound execution time.
constexpr std::size_t kMaxFramesPerRead = 200U;

// GetInfo service ID per Cyphal specification.
constexpr CanardPortID kGetInfoServiceId = 430U;

// Deadline for GetInfo request TX frames (1 second in the future).
constexpr CanardMicrosecond kGetInfoTxDeadlineUsec = 1'000'000UL;

// ---- static memory callbacks -------------------------------------------------

void * MagneticAngleSensor::cyphal_allocate(void * const user_reference, const size_t size)
{
  return o1heapAllocate(static_cast<O1HeapInstance *>(user_reference), size);
}

void MagneticAngleSensor::cyphal_deallocate(
  void * const user_reference, const size_t /*size*/, void * const pointer)
{
  o1heapFree(static_cast<O1HeapInstance *>(user_reference), pointer);
}

// ---- constructor -------------------------------------------------------------

MagneticAngleSensor::MagneticAngleSensor() = default;

// ---- on_init -----------------------------------------------------------------

hardware_interface::CallbackReturn MagneticAngleSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;

  if (info.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "No joints specified in hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- required: can_interface ----
  const auto iface_it = info.hardware_parameters.find("can_interface");
  if (iface_it == info.hardware_parameters.end() || iface_it->second.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Missing required hardware parameter 'can_interface'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = iface_it->second;

  // ---- optional: timeouts ----
  auto parse_uint32 =
    [&](const std::string & name, uint32_t & out, uint32_t default_val) -> bool {
      const auto it = info.hardware_parameters.find(name);
      if (it == info.hardware_parameters.end()) {
        out = default_val;
        return true;
      }
      try {
        size_t pos = 0;
        out = static_cast<uint32_t>(std::stoul(it->second, &pos));
        if (pos != it->second.size()) {throw std::invalid_argument("trailing chars");}
        return true;
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"),
          "Invalid parameter '%s': must be an unsigned integer", name.c_str());
        return false;
      }
    };

  if (!parse_uint32("stale_timeout_ms", stale_timeout_ms_, 200U)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_uint32("startup_timeout_ms", startup_timeout_ms_, 5000U)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- optional: Cyphal subject / node IDs ----
  auto parse_uint16 =
    [&](const std::string & name, uint16_t & out, uint16_t default_val) -> bool {
      const auto it = info.hardware_parameters.find(name);
      if (it == info.hardware_parameters.end()) {out = default_val; return true;}
      try {
        size_t pos = 0;
        const unsigned long v = std::stoul(it->second, &pos);
        if (pos != it->second.size()) {throw std::invalid_argument("trailing chars");}
        if (v > 0xFFFFU) {throw std::out_of_range("value too large");}
        out = static_cast<uint16_t>(v);
        return true;
      } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"),
          "Invalid parameter '%s'", name.c_str());
        return false;
      }
    };

  if (!parse_uint16("angle_subject_id", angle_subject_id_, 6144U)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!parse_uint16("temperature_subject_id", temperature_subject_id_, 6145U)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    uint32_t tmp = local_node_id_;
    if (!parse_uint32("local_node_id", tmp, 100U) || tmp > CANARD_NODE_ID_MAX) {
      RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"),
        "Parameter 'local_node_id' must be in range 0..%u", CANARD_NODE_ID_MAX);
      return hardware_interface::CallbackReturn::ERROR;
    }
    local_node_id_ = static_cast<uint8_t>(tmp);
  }

  // ---- parse per-joint configurations ----
  std::vector<MagneticAngleSensorJointConfig> joint_configs;
  joint_configs.reserve(info.joints.size());
  positions_.assign(info.joints.size(), 0.0);
  temperatures_.assign(info.joints.size(), 0.0);

  for (const auto & joint : info.joints) {
    MagneticAngleSensorJointConfig cfg;
    cfg.joint_name = joint.name;

    // Optional: node_id
    const auto nid_it = joint.parameters.find("node_id");
    if (nid_it != joint.parameters.end()) {
      try {
        size_t pos = 0;
        const unsigned long v = std::stoul(nid_it->second, &pos);
        if (pos != nid_it->second.size()) {throw std::invalid_argument("trailing");}
        if (v == 0 || v > CANARD_NODE_ID_MAX) {throw std::out_of_range("range");}
        cfg.has_node_id = true;
        cfg.node_id = static_cast<uint8_t>(v);
      } catch (...) {
        RCLCPP_ERROR(
          rclcpp::get_logger("MagneticAngleSensor"),
          "Joint '%s': 'node_id' must be 1..%u", joint.name.c_str(), CANARD_NODE_ID_MAX);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Optional: unique_id (32 lowercase hex characters = 16 byte 128-bit ID)
    const auto uid_it = joint.parameters.find("unique_id");
    if (uid_it != joint.parameters.end()) {
      const std::string & hex = uid_it->second;
      if (hex.size() != 32U) {
        RCLCPP_ERROR(
          rclcpp::get_logger("MagneticAngleSensor"),
          "Joint '%s': 'unique_id' must be exactly 32 hex characters (got %zu)",
          joint.name.c_str(), hex.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      bool ok = true;
      for (size_t b = 0; b < 16U && ok; ++b) {
        try {
          size_t pos = 0;
          const unsigned long byte_val = std::stoul(hex.substr(b * 2, 2), &pos, 16);
          if (pos != 2 || byte_val > 0xFFU) {ok = false; break;}
          cfg.unique_id[b] = static_cast<uint8_t>(byte_val);
        } catch (...) {ok = false;}
      }
      if (!ok) {
        RCLCPP_ERROR(
          rclcpp::get_logger("MagneticAngleSensor"),
          "Joint '%s': 'unique_id' contains invalid hex characters", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      cfg.has_unique_id = true;
    }

    if (!cfg.has_node_id && !cfg.has_unique_id) {
      RCLCPP_ERROR(
        rclcpp::get_logger("MagneticAngleSensor"),
        "Joint '%s': at least one of 'node_id' or 'unique_id' must be specified",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Optional: invert_position
    const auto inv_it = joint.parameters.find("invert_position");
    cfg.invert_position = (inv_it != joint.parameters.end() && inv_it->second == "true");

    // Optional: zero_offset_deg
    const auto off_it = joint.parameters.find("zero_offset_deg");
    if (off_it != joint.parameters.end()) {
      try {
        size_t pos = 0;
        cfg.zero_offset_deg = std::stod(off_it->second, &pos);
        if (pos != off_it->second.size()) {throw std::invalid_argument("trailing");}
      } catch (...) {
        RCLCPP_ERROR(
          rclcpp::get_logger("MagneticAngleSensor"),
          "Joint '%s': invalid 'zero_offset_deg'", joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    joint_configs.push_back(cfg);
  }

  std::string core_error;
  if (!core_.configure(joint_configs, stale_timeout_ms_, startup_timeout_ms_, &core_error)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Sensor core configuration failed: %s", core_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- initialise libcanard ----
  heap_ = o1heapInit(heap_arena_, kHeapArenaSize);
  if (heap_ == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"), "Failed to initialise o1heap memory pool");
    return hardware_interface::CallbackReturn::ERROR;
  }

  memory_resource_ = CanardMemoryResource{
    .user_reference = heap_,
    .deallocate = cyphal_deallocate,
    .allocate = cyphal_allocate,
  };

  canard_ = canardInit(memory_resource_);
  canard_.node_id = local_node_id_;

  // Register subscriptions — these persist across activate/deactivate cycles.
  // uavcan.si.unit.angle.Scalar.1.0  — 4-byte float32
  canardRxSubscribe(
    &canard_, CanardTransferKindMessage,
    angle_subject_id_, 4U,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &angle_subscription_);

  // uavcan.si.unit.temperature.Scalar.1.0 — 4-byte float32
  canardRxSubscribe(
    &canard_, CanardTransferKindMessage,
    temperature_subject_id_, 4U,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &temperature_subscription_);

  // uavcan.node.GetInfo.1.0 response — up to 448 bytes per @extent in the DSDL
  canardRxSubscribe(
    &canard_, CanardTransferKindResponse,
    kGetInfoServiceId, 448U,
    CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, &getinfo_response_subscription_);

  RCLCPP_INFO(
    rclcpp::get_logger("MagneticAngleSensor"),
    "Initialised Cyphal driver on '%s': angle subject %u, temperature subject %u, "
    "local node_id %u, %zu joint(s)",
    can_interface_.c_str(), angle_subject_id_, temperature_subject_id_,
    local_node_id_, info.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- on_configure ------------------------------------------------------------

hardware_interface::CallbackReturn MagneticAngleSensor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- on_activate -------------------------------------------------------------

hardware_interface::CallbackReturn MagneticAngleSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_can_socket()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialise (or reinitialise) the TX queue.
  if (tx_queue_initialized_) {
    drain_tx_queue();
  }
  tx_queue_ = canardTxInit(32U, CANARD_MTU_CAN_CLASSIC, memory_resource_);
  tx_queue_initialized_ = true;

  getinfo_tx_ids_.clear();
  core_.mark_started(now_steady());

  RCLCPP_INFO(
    rclcpp::get_logger("MagneticAngleSensor"),
    "Activated on '%s'", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- on_deactivate -----------------------------------------------------------

hardware_interface::CallbackReturn MagneticAngleSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  drain_tx_queue();
  close_can_socket();
  RCLCPP_INFO(rclcpp::get_logger("MagneticAngleSensor"), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---- export_state_interfaces -------------------------------------------------

std::vector<hardware_interface::StateInterface>
MagneticAngleSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.reserve(positions_.size() * 2U);
  for (size_t i = 0; i < positions_.size(); ++i) {
    const auto & name = info_.joints[i].name;
    ifaces.emplace_back(name, "position", &positions_[i]);
    ifaces.emplace_back(name, "temperature", &temperatures_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
MagneticAngleSensor::export_command_interfaces()
{
  return {};
}

// ---- read --------------------------------------------------------------------

hardware_interface::return_type MagneticAngleSensor::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (can_socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "CAN socket is not open");
    return hardware_interface::return_type::ERROR;
  }

  std::size_t frames_processed = 0;
  while (frames_processed < kMaxFramesPerRead) {
    struct can_frame raw {};
    const ssize_t n = ::recv(can_socket_fd_, &raw, sizeof(raw), 0);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      if (errno == EAGAIN || errno == EWOULDBLOCK) {break;}
      RCLCPP_ERROR(
        rclcpp::get_logger("MagneticAngleSensor"),
        "CAN recv error: %s", std::strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    if (n != static_cast<ssize_t>(sizeof(raw))) {continue;}
    // Cyphal/CAN requires extended 29-bit frames; skip RTR, error and standard frames.
    if ((raw.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {continue;}
    if ((raw.can_id & CAN_EFF_FLAG) == 0) {continue;}

    const CanardFrame frame{
      .extended_can_id = raw.can_id & CAN_EFF_MASK,
      .payload = {.size = raw.can_dlc, .data = raw.data},
    };

    RCLCPP_DEBUG(
      rclcpp::get_logger("MagneticAngleSensor"),
      "CAN RX  id=0x%08x  dlc=%u",
      frame.extended_can_id, raw.can_dlc);

    CanardRxTransfer transfer{};
    CanardRxSubscription * sub = nullptr;
    const int8_t result =
      canardRxAccept(&canard_, now_usec(), &frame, 0U, &transfer, &sub);

    if (result == 1) {
      if (sub == &angle_subscription_) {
        handle_angle_transfer(transfer);
      } else if (sub == &temperature_subscription_) {
        handle_temperature_transfer(transfer);
      } else if (sub == &getinfo_response_subscription_) {
        handle_getinfo_response(transfer);
      }
      // Transfer payload ownership is ours; free it.
      memory_resource_.deallocate(
        memory_resource_.user_reference,
        transfer.payload.allocated_size,
        transfer.payload.data);
    }
    ++frames_processed;
  }

  // Flush any GetInfo request frames queued this cycle.
  transmit_pending_frames();

  std::string state_error;
  if (!core_.compute_states(now_steady(), &positions_, &temperatures_, &state_error)) {
    RCLCPP_ERROR(rclcpp::get_logger("MagneticAngleSensor"), "%s", state_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ---- write -------------------------------------------------------------------

hardware_interface::return_type MagneticAngleSensor::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

// ---- transfer handlers -------------------------------------------------------

void MagneticAngleSensor::handle_angle_transfer(const CanardRxTransfer & transfer)
{
  const uint8_t src = transfer.metadata.remote_node_id;
  const uint8_t * payload = static_cast<const uint8_t *>(transfer.payload.data);
  const size_t len = transfer.payload.size;

  RCLCPP_DEBUG(
    rclcpp::get_logger("MagneticAngleSensor"),
    "Angle transfer from node %u: %zu bytes", src, len);

  // If the source node_id is unknown and discovery is needed, query it.
  if (core_.should_query_getinfo(src)) {
    RCLCPP_INFO(
      rclcpp::get_logger("MagneticAngleSensor"),
      "New node %u broadcasting angles — sending GetInfo request for unique_id discovery",
      src);
    core_.mark_getinfo_queried(src);
    send_getinfo_request(src);
  }

  std::string diag;
  if (core_.on_angle_message(src, payload, len, now_steady(), &diag)) {
    double angle_rad = 0.0;
    if (len >= 4) {
      float v;
      std::memcpy(&v, payload, 4);
      angle_rad = static_cast<double>(v);
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Angle  node=%u  raw=%.6f rad", src, angle_rad);
  }
}

void MagneticAngleSensor::handle_temperature_transfer(const CanardRxTransfer & transfer)
{
  const uint8_t src = transfer.metadata.remote_node_id;
  const uint8_t * payload = static_cast<const uint8_t *>(transfer.payload.data);
  const size_t len = transfer.payload.size;

  RCLCPP_DEBUG(
    rclcpp::get_logger("MagneticAngleSensor"),
    "Temperature transfer from node %u: %zu bytes", src, len);

  core_.on_temperature_message(src, payload, len, now_steady());

  if (len >= 4) {
    float v;
    std::memcpy(&v, payload, 4);
    RCLCPP_DEBUG(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Temperature  node=%u  raw=%.2f K", src, static_cast<double>(v));
  }
}

void MagneticAngleSensor::handle_getinfo_response(const CanardRxTransfer & transfer)
{
  const uint8_t src = transfer.metadata.remote_node_id;
  const uint8_t * payload = static_cast<const uint8_t *>(transfer.payload.data);
  const size_t len = transfer.payload.size;

  RCLCPP_DEBUG(
    rclcpp::get_logger("MagneticAngleSensor"),
    "GetInfo response from node %u: %zu bytes", src, len);

  uint8_t uid[16]{};
  if (!MagneticAngleSensorCore::extract_getinfo_unique_id(payload, len, uid)) {
    RCLCPP_WARN(
      rclcpp::get_logger("MagneticAngleSensor"),
      "GetInfo response from node %u too short to extract unique_id (%zu bytes)", src, len);
    return;
  }

  char uid_hex[33];
  for (int b = 0; b < 16; ++b) {
    std::snprintf(uid_hex + b * 2, 3, "%02x", uid[b]);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("MagneticAngleSensor"),
    "GetInfo response: node %u  unique_id=0x%s", src, uid_hex);

  std::string match_log;
  if (core_.on_getinfo_response(src, uid, &match_log)) {
    RCLCPP_INFO(rclcpp::get_logger("MagneticAngleSensor"), "%s", match_log.c_str());
  } else if (!match_log.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("MagneticAngleSensor"), "%s", match_log.c_str());
  }
}

// ---- GetInfo TX --------------------------------------------------------------

void MagneticAngleSensor::send_getinfo_request(const uint8_t target_node_id)
{
  if (!tx_queue_initialized_ || can_socket_fd_ < 0) {
    return;
  }

  CanardTransferID & tid = getinfo_tx_ids_[target_node_id];
  const CanardTransferMetadata meta{
    .priority = CanardPriorityNominal,
    .transfer_kind = CanardTransferKindRequest,
    .port_id = kGetInfoServiceId,
    .remote_node_id = target_node_id,
    .transfer_id = tid,
  };
  const CanardPayload empty_payload{.size = 0U, .data = nullptr};
  const CanardMicrosecond deadline = now_usec() + kGetInfoTxDeadlineUsec;

  const int32_t pushed = canardTxPush(
    &tx_queue_, &canard_, deadline, &meta, empty_payload, now_usec(), nullptr);

  if (pushed > 0) {
    ++tid;
    RCLCPP_DEBUG(
      rclcpp::get_logger("MagneticAngleSensor"),
      "GetInfo request queued for node %u (transfer_id %u)",
      target_node_id, static_cast<unsigned>(tid - 1U));
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to enqueue GetInfo request for node %u (error %d)", target_node_id, pushed);
  }
}

void MagneticAngleSensor::transmit_pending_frames()
{
  if (!tx_queue_initialized_ || can_socket_fd_ < 0) {
    return;
  }

  while (true) {
    CanardTxQueueItem * const item = canardTxPeek(&tx_queue_);
    if (item == nullptr) {break;}

    struct can_frame raw {};
    raw.can_id = item->frame.extended_can_id | CAN_EFF_FLAG;
    raw.can_dlc = static_cast<uint8_t>(item->frame.payload.size);
    if (item->frame.payload.data != nullptr && item->frame.payload.size > 0) {
      std::memcpy(raw.data, item->frame.payload.data, item->frame.payload.size);
    }

    const ssize_t sent = ::send(can_socket_fd_, &raw, sizeof(raw), MSG_DONTWAIT);
    if (sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // TX buffer full — leave remaining frames in the queue for the next cycle.
        break;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("MagneticAngleSensor"),
        "CAN TX error: %s", std::strerror(errno));
    }

    CanardTxQueueItem * popped = canardTxPop(&tx_queue_, item);
    canardTxFree(&tx_queue_, &canard_, popped);
  }
}

// ---- socket management -------------------------------------------------------

bool MagneticAngleSensor::open_can_socket()
{
  close_can_socket();

  can_socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to create CAN socket: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr {};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());
  if (::ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to lookup CAN interface '%s': %s",
      can_interface_.c_str(), std::strerror(errno));
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
      can_interface_.c_str(), std::strerror(errno));
    close_can_socket();
    return false;
  }

  const int flags = ::fcntl(can_socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(can_socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MagneticAngleSensor"),
      "Failed to set CAN socket non-blocking: %s", std::strerror(errno));
    close_can_socket();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MagneticAngleSensor"),
    "CAN socket opened on '%s'", can_interface_.c_str());
  return true;
}

void MagneticAngleSensor::close_can_socket()
{
  if (can_socket_fd_ >= 0) {
    ::close(can_socket_fd_);
    can_socket_fd_ = -1;
  }
}

void MagneticAngleSensor::drain_tx_queue()
{
  if (!tx_queue_initialized_) {
    return;
  }
  while (true) {
    CanardTxQueueItem * item = canardTxPeek(&tx_queue_);
    if (item == nullptr) {break;}
    canardTxFree(&tx_queue_, &canard_, canardTxPop(&tx_queue_, item));
  }
}

// ---- time helpers ------------------------------------------------------------

std::chrono::steady_clock::time_point MagneticAngleSensor::now_steady() const
{
  return std::chrono::steady_clock::now();
}

CanardMicrosecond MagneticAngleSensor::now_usec() const
{
  using namespace std::chrono;
  return static_cast<CanardMicrosecond>(
    duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace ros2_shoulder_sensor

PLUGINLIB_EXPORT_CLASS(
  ros2_shoulder_sensor::MagneticAngleSensor,
  hardware_interface::SystemInterface)
