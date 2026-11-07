// VL53L4CD time-of-flight distance sensor interface for measuring the
// absolute extension length of the box tube extender.
//
// DUMMY IMPLEMENTATION: the sensor is not yet installed. The plan is to
// measure the distance from a fixed point on the extender to a reflector
// on the end of the extender. The mounting position will be specified
// later; mount_offset_meters converts the raw range to extension length.
//
// TODO(wimblerobotics): implement I2C communication with the VL53L4CD
// (default address 0x29) using the ST ULD API register sequence: sensor
// init, start ranging, poll data-ready, read range, clear interrupt.

#pragma once

#include <cstdint>
#include <optional>

namespace titania_extender {

class Vl53l4cd {
 public:
  Vl53l4cd(int i2c_bus, uint8_t i2c_address, double mount_offset_meters,
           bool enabled)
      : i2c_bus_(i2c_bus),
        i2c_address_(i2c_address),
        mount_offset_meters_(mount_offset_meters),
        enabled_(enabled) {}

  bool initialize() {
    // TODO: open /dev/i2c-<bus>, verify model ID (0xEBAA), load default
    // configuration, start ranging.
    available_ = false;
    return available_;
  }

  bool isAvailable() const { return enabled_ && available_; }

  // Extension length in meters (raw range adjusted by the mounting
  // offset). Returns std::nullopt until the hardware is implemented.
  std::optional<double> readExtensionMeters() {
    // TODO: poll data-ready, read result register (mm), clear interrupt,
    // return range / 1000.0 - mount_offset_meters_.
    return std::nullopt;
  }

 private:
  int i2c_bus_;
  uint8_t i2c_address_;
  double mount_offset_meters_;
  bool enabled_;
  bool available_ = false;
};

}  // namespace titania_extender
