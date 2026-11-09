# titania_extender

ROS 2 driver for the gripper extender assembly of the Titania robot.

The extender has two motors driven by a single Basic Micro RoboClaw motor
controller connected to a Raspberry Pi 5 over USB:

| Motor | Name     | Function                                   |
|-------|----------|--------------------------------------------|
| M1    | `rotate` | Rotates the extender                       |
| M2    | `extend` | Extends/retracts the box tube extender     |

Moves are executed by the RoboClaw in position mode (buffered
speed/accel/deccel/position commands) with velocity and position PIDs
configured from the YAML file at startup.

## Sensors

| Hardware | Purpose | Status |
|----------|---------|--------|
| BNO055 IMU | Orientation and gravity vector | Enabled; publishes quaternion and gravity ([bno055.hpp](include/titania_extender/bno055.hpp)) |
| VL53L4CD ToF | Distance to the end reflector | Enabled; publishes raw range ([vl53l4cd.hpp](include/titania_extender/vl53l4cd.hpp)) |
| Limit switches | Min/max travel stops (optical or hall effect, 3-pin) | Dummy ([limit_switches.hpp](include/titania_extender/limit_switches.hpp)) |
| Homing | Home motors on startup using IMU + limit switches | Dummy (`homeMotors()`, exposed as the `home` service, never called automatically) |

The node publishes `imu/data` (`sensor_msgs/Imu`, including the quaternion),
`imu/gravity` (`geometry_msgs/Vector3Stamped`, m/s^2),
`extender_tof/range` (`sensor_msgs/Range`, raw meters), and
`extender_tof/range_status` (`std_msgs/UInt8`,
mapped VL53L4CD ranging status). IMU messages use the `extender_imu` frame ID;
ToF range messages use `extender_tof`. These frame names are published now;
the corresponding URDF elements can be added later. The raw ToF distance is
published whenever the sensor has a measurement, including measurements with a
nonzero status; check `extender_tof/range_status` when assessing its quality.
`extension_length` is
updated only for status-zero measurements, by projecting the raw distance onto
the tube axis using `tof.beam_angle_degrees` and subtracting
`tof.mount_offset_meters`. The beam angle is initially configured to the
approximate 9-degree mounting angle and the offset remains uncalibrated.
The node also publishes `rotation_angle` relative to the configured IMU zero
orientation. Until an IMU reference is configured,
the quaternion at node startup is captured as zero; place the extender at its
zero-angle position before starting the node. Copy the reported orientation
into `imu.zero_orientation` and set `imu.zero_orientation_set: true` to preserve
the reference across restarts. Raw IMU values remain available to verify the
installation. The current zero-angle calculation uses the relative
orientation quaternion; gravity is separately published so the mounting and
angle convention can be checked.

The VL53L4CD raw measurement is useful for calibration before setting
`tof.mount_offset_meters`; the current reflector setup means that offset should
be measured rather than assumed. The node initializes the
configured sensors and logs initialization errors. It needs permission to
open `/dev/i2c-<bus>`.

To manually scan every standard address on a bus, run the separate utility:

```bash
ros2 run titania_extender i2c_scan 1
```

Replace `1` with the bus number. Like `i2cdetect`, a full scan probes devices
using SMBus quick writes or receive-byte reads, depending on address and
adapter capabilities; probing can affect some devices on the bus.

## Interfaces

All positions/speeds are in encoder counts (quadrature pulses) and QPPS.

### Actions

* `rotate/move_to_position`, `extend/move_to_position`
  ([MoveToPosition](action/MoveToPosition.action)) — move to an absolute
  position, or a relative offset when `relative: true`. Optional non-zero
  `speed`, `accel`, `decel` override the YAML defaults. Feedback streams
  position/speed/remaining counts. A new goal preempts the active one.
  Targets are clamped to the configured min/max limits.

### Topics

Published:

* `rotate/status`, `extend/status`
  ([MotorStatus](msg/MotorStatus.msg)) — encoder, speed, current,
  temperature, voltage, error flags, target, limits. Rate: `sensor_rate_hz`.
  `error_status` contains errors in its low 16 bits and warnings in its high
  16 bits; `error_string` decodes both.
* `rotation_angle` (`std_msgs/Float32`) — degrees.
* `extension_length` (`std_msgs/Float32`) — meters.

Subscribed (fire-and-forget moves using YAML default speed/accel/decel):

* `rotate/cmd_position`, `extend/cmd_position` (`std_msgs/Int32`) — absolute.
* `rotate/cmd_position_relative`, `extend/cmd_position_relative`
  (`std_msgs/Int32`) — offset from the current encoder value.

### Services

* `home` (`std_srvs/Trigger`) — homing procedure (dummy; not implemented).
* `reset_encoders` (`std_srvs/Trigger`) — zero both encoders.

## Configuration

All tunables live in
[config/titania_extender.yaml](config/titania_extender.yaml): serial device,
per-motor velocity and position PIDs, QPPS, position limits, default
speed/accel/decel, current limits, calibration values, and dummy-sensor
settings.

Note: at startup the driver resets both encoders, i.e. **the power-on
physical position is position 0**. This will be replaced by a homing
procedure once the limit switches and IMU are supported.

## Device permissions

Install the included [udev rule](udev/99-titania-extender.rules) on the
Raspberry Pi to create `/dev/roboclaw` and give members of `dialout` read/write
access to the RoboClaw and `/dev/i2c-1`:

```bash
cd ~/titania_extender
sudo install -m 0644 src/titania_extender/udev/99-titania-extender.rules /etc/udev/rules.d/
sudo usermod -aG dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add /sys/class/tty/ttyACM0 /sys/class/i2c-dev/i2c-1
sudo udevadm settle
```

Log out and back in so the user's new group membership takes effect. If the
RoboClaw is not currently `ttyACM0`, use its actual sysfs path in the trigger
command, or reconnect it. Verify with `id`, `ls -l /dev/roboclaw /dev/i2c-1`
and `test -r /dev/i2c-1 && test -w /dev/i2c-1`.

The USB rule matches the connected Basicmicro RoboClaw 2x15A by vendor,
product and device strings rather than its changing `ttyACM` number. It has
no unique serial number, so multiple matching controllers require a
device-specific rule to avoid a symlink conflict. The I2C rule applies only
to bus 1; adjust it if the configured sensor buses differ. Other existing
udev rules or ACLs may also grant access.

## Building

```bash
cd ~/titania_extender
colcon build --symlink-install
source install/setup.bash
```

## Running

```bash
ros2 launch titania_extender titania_extender.launch.py
# or with a custom config:
ros2 launch titania_extender titania_extender.launch.py config_file:=/path/to/config.yaml
```

Examples:

```bash
# Rotate the extender to position 1500 counts.
ros2 action send_goal /rotate/move_to_position \
  titania_extender/action/MoveToPosition "{position: 1500}" --feedback

# Extend by 1000 counts from the current position at reduced speed.
ros2 action send_goal /extend/move_to_position \
  titania_extender/action/MoveToPosition \
  "{position: 1000, relative: true, speed: 2000}"

# Fire-and-forget move.
ros2 topic pub --once /extend/cmd_position std_msgs/msg/Int32 "{data: -5000}"

# Watch a motor.
ros2 topic echo /extend/status
```

## Testing without hardware

[scripts/roboclaw_emulator.py](scripts/roboclaw_emulator.py) emulates the
RoboClaw protocol subset used by the driver, including a simple motion
model:

```bash
python3 src/titania_extender/scripts/roboclaw_emulator.py &
ros2 run titania_extender titania_extender_node --ros-args \
  --params-file src/titania_extender/config/titania_extender.yaml \
  -p device_name:=/tmp/roboclaw_emu
```

## License

Project code is licensed under the Apache License 2.0; see [LICENSE](LICENSE).
The VL53L4CD initialization values from STMicroelectronics retain their
BSD-3-Clause notice in [vl53l4cd.cpp](src/vl53l4cd.cpp).
