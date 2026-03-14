MagneticAngleSensor
===================

`ros2_shoulder_sensor/MagneticAngleSensor` is a `hardware_interface::SystemInterface` plugin that reads magnetic angle sensors from SocketCAN and exports per-joint:

- `position` state interface (radians)
- `temperature` state interface (degrees C)

Protocol
--------

Each sensor reports on an extended CAN ID:

- `extended_id = (node_id << 5) | address`
- this implementation currently supports `address = 0` only

Payload format is 5 bytes:

- byte 0: wrapping sequence number (0-255)
- bytes 1-2: signed angle in hundredths of a degree, big-endian
- bytes 3-4: signed temperature in hundredths of a degree C, big-endian

Filtering behavior:

- ignores standard (non-extended) CAN frames
- ignores frames where `DLC != 5`
- ignores frames where ID does not match configured node IDs

Configuration
-------------

Hardware-level parameters:

- `can_interface` (required): SocketCAN interface name, for example `can0`
- `stale_timeout_ms` (optional, default `200`): maximum sample age allowed in `read()`
- `startup_timeout_ms` (optional, default `stale_timeout_ms`): max time after activation before first frame must be received for every joint

Per-joint parameters:

- `node_id` (required, 0..255): simple-can node ID
- `invert_position` (optional, default `false`): invert decoded angle before applying offset
- `zero_offset_deg` (optional, default `0.0`): angle offset in degrees applied after inversion

Constraints:

- duplicate `node_id` values across joints are rejected as configuration errors
- on missing data (startup timeout or stale timeout), `read()` returns error
- sequence discontinuities are logged as diagnostics

URDF / ros2_control Example
---------------------------

```xml
<ros2_control name="ShoulderSensors" type="system">
  <hardware>
    <plugin>ros2_shoulder_sensor/MagneticAngleSensor</plugin>
    <param name="can_interface">can0</param>
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

Build and Test
--------------

From the workspace root:

```bash
source /home/pi/r2_ws/setup.bash
colcon build --symlink-install --packages-select ros2_shoulder_sensor
colcon test --packages-select ros2_shoulder_sensor
```

Test coverage includes:

- ID mapping and demultiplexing for multiple sensors on one CAN bus
- payload decoding including signed values
- malformed frame rejection
- inversion and zero-offset transform behavior
- startup-timeout and stale-timeout failures
- sequence discontinuity diagnostics
