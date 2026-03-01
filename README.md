MagneticAngleSensor
===================

This package provides a minimal `hardware_interface::SystemInterface` plugin `MagneticAngleSensor` which exposes joint `position` state interfaces and returns a configured static value per joint.

Usage
-----

Add the hardware plugin to your ros2_control system in the URDF (example):

```xml
<ros2_control name="MyHardware" type="system">
  <hardware>
    <plugin>ros2_shoulder_sensor::MagneticAngleSensor</plugin>
    <!-- optional hardware-level default -->
    <param name="static_position">0.5</param>
    <joint name="shoulder_joint_left">
      <param name="static_position">0.25</param>
    </joint>
    <joint name="shoulder_joint_right">
      <param name="static_position">-0.25</param>
    </joint>
  </hardware>
</ros2_control>
```

Notes
-----
- The interface reads `static_position` from either `hardware` level parameters or from individual `joint` parameters. The joint parameter overrides the hardware default.
- The plugin is exported in `plugin.xml` and built as a shared library.
