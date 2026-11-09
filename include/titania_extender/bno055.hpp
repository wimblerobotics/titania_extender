#pragma once

// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace titania_extender {

class Bno055 {
 public:
  struct Sample {
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double gravity_x;
    double gravity_y;
    double gravity_z;
  };

  Bno055(int i2c_bus, uint8_t i2c_address, bool enabled);
  ~Bno055();

  Bno055(const Bno055&) = delete;
  Bno055& operator=(const Bno055&) = delete;

  bool initialize();
  bool isAvailable() const { return enabled_ && available_; }
  const std::string& lastError() const { return last_error_; }
  std::optional<Sample> readSample();

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* values, size_t count);

  int i2c_bus_;
  uint8_t i2c_address_;
  bool enabled_;
  bool available_ = false;
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace titania_extender
