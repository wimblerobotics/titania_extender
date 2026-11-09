// Copyright 2026 Wimblerobotics
// SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
// Includes VL53L4CD initialization values from STMicroelectronics' ULD.
// Copyright (c) 2021 STMicroelectronics. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of STMicroelectronics nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "titania_extender/vl53l4cd.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>
#include <utility>

namespace titania_extender {
namespace {

constexpr uint16_t kModelIdRegister = 0x010F;
constexpr uint16_t kFirmwareStatusRegister = 0x00E5;
constexpr uint16_t kSystemStartRegister = 0x0087;
constexpr uint16_t kSystemInterruptClearRegister = 0x0086;
constexpr uint16_t kGpioMuxControlRegister = 0x0030;
constexpr uint16_t kGpioStatusRegister = 0x0031;
constexpr uint16_t kRangeStatusRegister = 0x0089;
constexpr uint16_t kSpadCountRegister = 0x008C;
constexpr uint16_t kDistanceRegister = 0x0096;

constexpr std::array<uint8_t, 0x87 - 0x2D + 1> kDefaultConfiguration = {
    0x12, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x07, 0x01, 0xF1, 0x05, 0x00, 0xA0, 0x00, 0x80, 0x08,
    0x38, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

uint16_t decodeWord(const uint8_t* data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

uint8_t mapRangeStatus(uint8_t status) {
  constexpr std::array<uint8_t, 24> kStatusMap = {
      255, 255, 255, 5,   2,   4,   1,  7, 3,   0,   255, 255,
      9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11,  12};
  status &= 0x1F;
  return status < kStatusMap.size() ? kStatusMap[status] : status;
}

}  // namespace

Vl53l4cd::Vl53l4cd(int i2c_bus, uint8_t i2c_address, bool enabled)
    : i2c_bus_(i2c_bus), i2c_address_(i2c_address), enabled_(enabled) {}

Vl53l4cd::~Vl53l4cd() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool Vl53l4cd::writeByte(uint16_t reg, uint8_t value) {
  const std::array<uint8_t, 3> data{static_cast<uint8_t>(reg >> 8),
                                    static_cast<uint8_t>(reg), value};
  if (write(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Vl53l4cd::writeWord(uint16_t reg, uint16_t value) {
  const std::array<uint8_t, 4> data{
      static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  if (write(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Vl53l4cd::writeDword(uint16_t reg, uint32_t value) {
  const std::array<uint8_t, 6> data{
      static_cast<uint8_t>(reg >> 8),    static_cast<uint8_t>(reg),
      static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
      static_cast<uint8_t>(value >> 8),  static_cast<uint8_t>(value)};
  if (write(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Vl53l4cd::readByte(uint16_t reg, uint8_t& value) {
  const std::array<uint8_t, 2> address{static_cast<uint8_t>(reg >> 8),
                                       static_cast<uint8_t>(reg)};
  if (write(fd_, address.data(), address.size()) !=
      static_cast<ssize_t>(address.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  if (read(fd_, &value, sizeof(value)) != static_cast<ssize_t>(sizeof(value))) {
    last_error_ = std::strerror(errno);
    return false;
  }
  return true;
}

bool Vl53l4cd::readWord(uint16_t reg, uint16_t& value) {
  std::array<uint8_t, 2> data{};
  const std::array<uint8_t, 2> address{static_cast<uint8_t>(reg >> 8),
                                       static_cast<uint8_t>(reg)};
  if (write(fd_, address.data(), address.size()) !=
      static_cast<ssize_t>(address.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  if (read(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  value = decodeWord(data.data());
  return true;
}

bool Vl53l4cd::readDword(uint16_t reg, uint32_t& value) {
  std::array<uint8_t, 4> data{};
  const std::array<uint8_t, 2> address{static_cast<uint8_t>(reg >> 8),
                                       static_cast<uint8_t>(reg)};
  if (write(fd_, address.data(), address.size()) !=
      static_cast<ssize_t>(address.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  if (read(fd_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    last_error_ = std::strerror(errno);
    return false;
  }
  value = (static_cast<uint32_t>(data[0]) << 24) |
          (static_cast<uint32_t>(data[1]) << 16) |
          (static_cast<uint32_t>(data[2]) << 8) | data[3];
  return true;
}

bool Vl53l4cd::waitForDataReady() {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    uint8_t mux = 0;
    uint8_t status = 0;
    if (!readByte(kGpioMuxControlRegister, mux) ||
        !readByte(kGpioStatusRegister, status)) {
      return false;
    }
    const uint8_t interrupt_polarity = (mux & 0x10) ? 0 : 1;
    if ((status & 1) == interrupt_polarity) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  last_error_ = "timed out waiting for sensor data";
  return false;
}

bool Vl53l4cd::initialize() {
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

  uint16_t model_id = 0;
  if (!readWord(kModelIdRegister, model_id)) {
    close(fd_);
    fd_ = -1;
    return false;
  }
  if (model_id != 0xEBAA) {
    last_error_ = "unexpected model ID";
    close(fd_);
    fd_ = -1;
    return false;
  }

  uint8_t firmware_status = 0;
  const auto boot_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  do {
    if (!readByte(kFirmwareStatusRegister, firmware_status)) {
      close(fd_);
      fd_ = -1;
      return false;
    }
    if (firmware_status == 0x03) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < boot_deadline);
  if (firmware_status != 0x03) {
    last_error_ = "sensor firmware did not boot";
    close(fd_);
    fd_ = -1;
    return false;
  }

  for (size_t offset = 0; offset < kDefaultConfiguration.size(); offset += 32) {
    const size_t count =
        std::min<size_t>(32, kDefaultConfiguration.size() - offset);
    std::array<uint8_t, 34> data{};
    const uint16_t reg = static_cast<uint16_t>(0x002D + offset);
    data[0] = static_cast<uint8_t>(reg >> 8);
    data[1] = static_cast<uint8_t>(reg);
    std::copy_n(kDefaultConfiguration.data() + offset, count, data.data() + 2);
    if (write(fd_, data.data(), count + 2) != static_cast<ssize_t>(count + 2)) {
      last_error_ = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      return false;
    }
  }

  if (!writeByte(kSystemStartRegister, 0x40) || !waitForDataReady() ||
      !writeByte(kSystemInterruptClearRegister, 0x01) ||
      !writeByte(kSystemStartRegister, 0x00) || !writeByte(0x0008, 0x09) ||
      !writeByte(0x000B, 0x00) || !writeWord(0x0024, 0x0500)) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  uint16_t oscillator_frequency = 0;
  if (!readWord(0x0006, oscillator_frequency) || oscillator_frequency == 0) {
    if (last_error_.empty()) {
      last_error_ = "invalid oscillator frequency";
    }
    close(fd_);
    fd_ = -1;
    return false;
  }

  const uint64_t macro_period_us =
      (uint64_t{2304} * (uint64_t{0x40000000} / oscillator_frequency)) >> 6;
  const uint64_t timing_budget = uint64_t{47500} << 12;
  for (const auto [reg, factor] :
       {std::pair<uint16_t, uint64_t>{0x005E, 16}, {0x0061, 12}}) {
    const uint64_t divisor = (macro_period_us * factor) >> 6;
    uint64_t mantissa = (timing_budget + divisor / 2) / divisor - 1;
    uint16_t exponent = 0;
    while (mantissa > 0xFF) {
      mantissa >>= 1;
      ++exponent;
    }
    const uint16_t timing_value =
        static_cast<uint16_t>((exponent << 8) | (mantissa & 0xFF));
    if (!writeWord(reg, timing_value)) {
      close(fd_);
      fd_ = -1;
      return false;
    }
  }
  if (!writeDword(0x006C, 0) || !writeByte(kSystemStartRegister, 0x21) ||
      !waitForDataReady() || !writeByte(kSystemInterruptClearRegister, 0x01)) {
    close(fd_);
    fd_ = -1;
    return false;
  }

  available_ = true;
  last_error_.clear();
  return true;
}

std::optional<Vl53l4cd::Sample> Vl53l4cd::readSample() {
  if (!isAvailable()) {
    return std::nullopt;
  }

  uint8_t mux = 0;
  uint8_t interrupt_status = 0;
  if (!readByte(kGpioMuxControlRegister, mux) ||
      !readByte(kGpioStatusRegister, interrupt_status)) {
    return std::nullopt;
  }
  const uint8_t interrupt_polarity = (mux & 0x10) ? 0 : 1;
  if ((interrupt_status & 1) != interrupt_polarity) {
    return std::nullopt;
  }

  uint8_t range_status = 0;
  uint16_t spad_count = 0;
  uint16_t distance_mm = 0;
  if (!readByte(kRangeStatusRegister, range_status) ||
      !readWord(kSpadCountRegister, spad_count) ||
      !readWord(kDistanceRegister, distance_mm) ||
      !writeByte(kSystemInterruptClearRegister, 0x01)) {
    return std::nullopt;
  }
  if (spad_count == 0) {
    last_error_.clear();
    return std::nullopt;
  }

  last_error_.clear();
  return Sample{static_cast<double>(distance_mm) / 1000.0,
                mapRangeStatus(range_status)};
}

}  // namespace titania_extender
