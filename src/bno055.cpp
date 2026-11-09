// Copyright 2026 Wimblerobotics
// SPDX-License-Identifier: Apache-2.0

#include "titania_extender/bno055.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace titania_extender {
namespace {

constexpr uint8_t kChipIdRegister = 0x00;
constexpr uint8_t kExpectedChipId = 0xA0;
constexpr uint8_t kPageIdRegister = 0x07;
constexpr uint8_t kUnitSelectionRegister = 0x3B;
constexpr uint8_t kOperationModeRegister = 0x3D;
constexpr uint8_t kPowerModeRegister = 0x3E;
constexpr uint8_t kConfigMode = 0x00;
constexpr uint8_t kImuPlusMode = 0x08;

int16_t decodeSignedWord(uint8_t low, uint8_t high) {
  const uint16_t bits =
      static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return static_cast<int16_t>(bits);
}

}  // namespace

Bno055::Bno055(int i2c_bus, uint8_t i2c_address, bool enabled)
    : i2c_bus_(i2c_bus), i2c_address_(i2c_address), enabled_(enabled) {}

Bno055::~Bno055() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool Bno055::writeRegister(uint8_t reg, uint8_t value) {
  const std::array<uint8_t, 2> data{reg, value};
  if (write(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Bno055::readRegisters(uint8_t reg, uint8_t* values, size_t count) {
  if (write(fd_, &reg, sizeof(reg)) != static_cast<ssize_t>(sizeof(reg))) {
    last_error_ = std::strerror(errno);
    return false;
  }
  if (read(fd_, values, count) != static_cast<ssize_t>(count)) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Bno055::initialize() {
  available_ = false;
  if (!enabled_) {
    last_error_ = "disabled by configuration";
    return false;
  }

  const std::string path = "/dev/i2c-" + std::to_string(i2c_bus_);
  fd_ = open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    last_error_ = "cannot open " + path + ": " + std::strerror(errno);
    return false;
  }
  if (ioctl(fd_, I2C_SLAVE, static_cast<int>(i2c_address_)) < 0) {
    last_error_ =
        "cannot select I2C address: " + std::string(std::strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }

  uint8_t chip_id = 0;
  bool chip_ready = false;
  const auto boot_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < boot_deadline) {
    if (!readRegisters(kChipIdRegister, &chip_id, sizeof(chip_id))) {
      close(fd_);
      fd_ = -1;
      return false;
    }
    if (chip_id == kExpectedChipId) {
      chip_ready = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!chip_ready) {
    last_error_ = "unexpected chip ID " + std::to_string(chip_id);
    close(fd_);
    fd_ = -1;
    return false;
  }

  if (!writeRegister(kOperationModeRegister, kConfigMode)) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  if (!writeRegister(kPageIdRegister, 0) ||
      !writeRegister(kPowerModeRegister, 0) ||
      !writeRegister(kUnitSelectionRegister, 0) ||
      !writeRegister(kOperationModeRegister, kImuPlusMode)) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  available_ = true;
  last_error_.clear();
  return true;
}

std::optional<Bno055::Sample> Bno055::readSample() {
  if (!isAvailable()) {
    return std::nullopt;
  }

  std::array<uint8_t, 8> quaternion_data{};
  std::array<uint8_t, 6> gravity_data{};
  if (!readRegisters(0x20, quaternion_data.data(), quaternion_data.size()) ||
      !readRegisters(0x2E, gravity_data.data(), gravity_data.size())) {
    return std::nullopt;
  }

  constexpr double kQuaternionScale = 1.0 / 16384.0;
  constexpr double kGravityScale = 1.0 / 100.0;
  const int16_t w = decodeSignedWord(quaternion_data[0], quaternion_data[1]);
  const int16_t x = decodeSignedWord(quaternion_data[2], quaternion_data[3]);
  const int16_t y = decodeSignedWord(quaternion_data[4], quaternion_data[5]);
  const int16_t z = decodeSignedWord(quaternion_data[6], quaternion_data[7]);
  const int16_t gx = decodeSignedWord(gravity_data[0], gravity_data[1]);
  const int16_t gy = decodeSignedWord(gravity_data[2], gravity_data[3]);
  const int16_t gz = decodeSignedWord(gravity_data[4], gravity_data[5]);

  const double quaternion_norm =
      std::sqrt(w * w + x * x + y * y + z * z) * kQuaternionScale;
  if (quaternion_norm < 0.5) {
    last_error_ = "orientation quaternion is not ready";
    return std::nullopt;
  }

  last_error_.clear();
  return Sample{x * kQuaternionScale, y * kQuaternionScale,
                z * kQuaternionScale, w * kQuaternionScale,
                gx * kGravityScale,   gy * kGravityScale,
                gz * kGravityScale};
}

}  // namespace titania_extender
