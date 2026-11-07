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

## Status of future hardware (dummy implementations)

| Hardware | Purpose | Status |
|----------|---------|--------|
| BNO055 IMU | Rotation angle from the gravity vector | Dummy ([bno055.hpp](include/titania_extender/bno055.hpp)) |
| VL53L4CD ToF | Absolute extension length to a reflector | Dummy ([vl53l4cd.hpp](include/titania_extender/vl53l4cd.hpp)) |
| Limit switches | Min/max travel stops (optical or hall effect, 3-pin) | Dummy ([limit_switches.hpp](include/titania_extender/limit_switches.hpp)) |
| Homing | Home motors on startup using IMU + limit switches | Dummy (`homeMotors()`, exposed as the `home` service, never called automatically) |

Until the sensors exist, `rotation_angle` and `extension_length` fall back
to encoder counts scaled by the `counts_per_degree` / `counts_per_meter`
calibration values in the YAML file (and publish `NaN` while those are 0.0).

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
procedure once the limit switches and IMU are installed.

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

MIT — see [LICENSE](LICENSE).
