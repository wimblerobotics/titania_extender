#pragma once

// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <optional>
#include <string>

namespace titania_extender {

class Vl53l4cd {
 public:
  struct Sample {
    double distance_meters;
    uint8_t range_status;
  };

  Vl53l4cd(int i2c_bus, uint8_t i2c_address, bool enabled);
  ~Vl53l4cd();

  Vl53l4cd(const Vl53l4cd&) = delete;
  Vl53l4cd& operator=(const Vl53l4cd&) = delete;

  bool initialize();
  bool isAvailable() const { return enabled_ && available_; }
  const std::string& lastError() const { return last_error_; }
  std::optional<Sample> readSample();

 private:
  bool writeByte(uint16_t reg, uint8_t value);
  bool writeWord(uint16_t reg, uint16_t value);
  bool writeDword(uint16_t reg, uint32_t value);
  bool readByte(uint16_t reg, uint8_t& value);
  bool readWord(uint16_t reg, uint16_t& value);
  bool readDword(uint16_t reg, uint32_t& value);
  bool waitForDataReady();

  int i2c_bus_;
  uint8_t i2c_address_;
  bool enabled_;
  bool available_ = false;
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace titania_extender
