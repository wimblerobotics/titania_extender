// BNO055 IMU interface for measuring the extender rotation angle.
//
// DUMMY IMPLEMENTATION: the BNO055 is not yet connected. The plan is to
// mount it on the rotating part of the extender and derive the rotation
// angle from the gravity vector. The mounting position relative to the
// rotation axis point and orientation will be specified later.
//
// TODO(wimblerobotics): implement I2C communication with the BNO055
// (default address 0x28), configure NDOF or ACCONLY mode, read the gravity
// vector registers, and compute the rotation angle from the projection of
// gravity onto the rotation plane given the mounting pose.

#pragma once

#include <cstdint>
#include <optional>

namespace titania_extender {

class Bno055 {
 public:
  Bno055(int i2c_bus, uint8_t i2c_address, bool enabled)
      : i2c_bus_(i2c_bus), i2c_address_(i2c_address), enabled_(enabled) {}

  // Returns true if the device was initialized and is usable.
  bool initialize() {
    // TODO: open /dev/i2c-<bus>, verify chip ID (0xA0), configure mode.
    available_ = false;
    return available_;
  }

  bool isAvailable() const { return enabled_ && available_; }

  // Rotation angle of the extender in degrees, derived from the gravity
  // vector. Returns std::nullopt until the hardware is implemented.
  std::optional<double> readRotationAngleDegrees() {
    // TODO: read gravity vector (registers 0x2E..0x33, 100 LSB = 1 m/s^2),
    // transform by the mounting orientation, and compute
    // atan2 of the in-plane components.
    return std::nullopt;
  }

 private:
  int i2c_bus_;
  uint8_t i2c_address_;
  bool enabled_;
  bool available_ = false;
};

}  // namespace titania_extender
