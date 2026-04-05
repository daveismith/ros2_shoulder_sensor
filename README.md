MagneticAngleSensor
===================

`ros2_shoulder_sensor/MagneticAngleSensor` is a `hardware_interface::SystemInterface` plugin that reads Cyphal/CAN shoulder sensor data over SocketCAN and exports per-joint:

- `position` state interface (radians)
- `temperature` state interface (Kelvin)

Protocol
--------

This driver uses Cyphal/CAN transport (via vendored `libcanard` and `o1heap`) and subscribes to two message subjects:

- `uavcan.si.unit.angle.Scalar.1.0` on `angle_subject_id` (default `6144`)
- `uavcan.si.unit.temperature.Scalar.1.0` on `temperature_subject_id` (default `6145`)

Message payloads:

- angle: 4-byte little-endian IEEE754 `float32` in radians
- temperature: 4-byte little-endian IEEE754 `float32` in Kelvin

The driver processes only extended CAN data frames accepted by Cyphal framing rules. Incoming transfers are logged at debug/info level, including:

- raw CAN RX frame IDs and DLC
- decoded angle/temperature transfer origin and payload size
- GetInfo discovery events and unique-ID matches

Node Identification and Discovery
-------------------------------

Each joint must provide at least one identification method:

- static `node_id`
- `unique_id` discovery (from `uavcan.node.GetInfo.1.0` response)

Discovery flow:

- if an angle message arrives from an unknown node, the driver can issue `uavcan.node.GetInfo.1.0` request (service ID `430`)
- response `unique_id` is matched against unresolved joint `unique_id` values
- on match, that source node is bound to the joint and subsequent angle/temperature messages are accepted

If both `node_id` and `unique_id` are provided for a joint, `node_id` is used immediately.

Configuration
-------------

Hardware-level parameters:

- `can_interface` (required): SocketCAN interface name, for example `can0`
- `angle_subject_id` (optional, default `6144`): Cyphal subject ID for angle messages
- `temperature_subject_id` (optional, default `6145`): Cyphal subject ID for temperature messages
- `local_node_id` (optional, default `100`): local Cyphal node ID used by this driver (`0..127`)
- `stale_timeout_ms` (optional, default `200`): maximum sample age allowed in `read()`
- `startup_timeout_ms` (optional, default `5000`): max time after activation before first angle message must be received for every joint

Per-joint parameters:

- `node_id` (optional, `1..127`): Cyphal source node ID for static mapping
- `unique_id` (optional): 32 hex chars (128-bit) from GetInfo response for discovery mapping
- `invert_position` (optional, default `false`): invert decoded angle before applying offset
- `zero_offset_deg` (optional, default `0.0`): angle offset in degrees applied after inversion

Constraints:

- at least one of `node_id` or `unique_id` must be provided per joint
- duplicate static `node_id` values across joints are rejected
- on missing angle data (startup timeout or stale timeout), `read()` returns error
- temperature is published as Kelvin (no Celsius conversion)

URDF / ros2_control Example
---------------------------

Static `node_id` mapping:

```xml
<ros2_control name="ShoulderSensors" type="system">
  <hardware>
    <plugin>ros2_shoulder_sensor/MagneticAngleSensor</plugin>
    <param name="can_interface">can0</param>
    <param name="angle_subject_id">6144</param>
    <param name="temperature_subject_id">6145</param>
    <param name="local_node_id">100</param>
    <param name="stale_timeout_ms">200</param>
    <param name="startup_timeout_ms">1000</param>
  </hardware>

  <joint name="left_shoulder_joint">
    <state_interface name="position"/>
    <state_interface name="temperature"/>
    <param name="node_id">3</param>
    <param name="invert_position">false</param>
    <param name="zero_offset_deg">0.0</param>
  </joint>

  <joint name="right_shoulder_joint">
    <state_interface name="position"/>
    <state_interface name="temperature"/>
    <param name="node_id">4</param>
    <param name="invert_position">true</param>
    <param name="zero_offset_deg">1.5</param>
  </joint>
</ros2_control>
```

Unique-ID discovery mapping:

```xml
<ros2_control name="ShoulderSensors" type="system">
  <hardware>
    <plugin>ros2_shoulder_sensor/MagneticAngleSensor</plugin>
    <param name="can_interface">can0</param>
    <param name="local_node_id">100</param>
  </hardware>

  <joint name="left_shoulder_joint">
    <state_interface name="position"/>
    <state_interface name="temperature"/>
    <param name="unique_id">0102030405060708090a0b0c0d0e0f10</param>
  </joint>

  <joint name="right_shoulder_joint">
    <state_interface name="position"/>
    <state_interface name="temperature"/>
    <param name="unique_id">aabbccddeeff11223344556677889900</param>
    <param name="invert_position">true</param>
  </joint>
</ros2_control>
```

Build and Test
--------------

From the workspace root:

```bash
source /home/pi/r2_ws/setup.bash
colcon build --symlink-install --packages-select ros2_shoulder_sensor
colcon test --packages-select ros2_shoulder_sensor
```

Test coverage includes:

- angle/temperature float32 deserialization validation
- GetInfo unique-ID extraction and discovery matching
- static and dynamic node resolution behavior
- inversion and zero-offset transform behavior
- startup-timeout and stale-timeout failures
- runtime reset behavior on reactivation (`mark_started`)
