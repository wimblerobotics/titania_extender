// Copyright 2026 titania_extender contributors
// SPDX-License-Identifier: Apache-2.0
// Only linked into homing_test_node, never the hardware driver.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

#include "titania_extender/bno055.hpp"
#include "titania_extender/vl53l4cd.hpp"

namespace titania_extender {
namespace {
struct State {
  double length;
  double angle;
  int quality;
  bool imu_valid;
  double length_noise;
  double angle_noise;
};

State readState() {
  const char* path = std::getenv("TITANIA_EXTENDER_TEST_SENSOR_STATE");
  if (!path) {
    throw std::runtime_error("test sensor state path is required");
  }
  std::ifstream input(path);
  State state{};
  if (!(input >> state.length >> state.angle >> state.quality >>
        state.imu_valid >> state.length_noise >> state.angle_noise)) {
    throw std::runtime_error("cannot read test sensor state");
  }
  return state;
}
}  // namespace

Bno055::Bno055(int bus, uint8_t address, bool enabled)
    : i2c_bus_(bus), i2c_address_(address), enabled_(enabled) {}
Bno055::~Bno055() = default;
bool Bno055::initialize() {
  available_ = enabled_;
  return available_;
}
std::optional<Bno055::Sample> Bno055::readSample() {
  const auto state = readState();
  if (!state.imu_valid) {
    return std::nullopt;
  }
  constexpr double kRadians = 3.14159265358979323846 / 180.0;
  const double angle = state.angle * kRadians;
  static unsigned sample_index = 0;
  const double noise = (++sample_index % 2 ? 1 : -1) * state.angle_noise;
  const double gravity_angle =
      std::atan2(0.22, 9.80) + angle + noise * kRadians;
  const double magnitude = std::hypot(0.22, 9.80);
  return Sample{0,
                0,
                std::sin(angle / 2),
                std::cos(angle / 2),
                magnitude * std::sin(gravity_angle),
                magnitude * std::cos(gravity_angle),
                -0.18};
}

Vl53l4cd::Vl53l4cd(int bus, uint8_t address, bool enabled)
    : i2c_bus_(bus), i2c_address_(address), enabled_(enabled) {}
Vl53l4cd::~Vl53l4cd() = default;
bool Vl53l4cd::initialize() {
  available_ = enabled_;
  return available_;
}
std::optional<Vl53l4cd::Sample> Vl53l4cd::readSample() {
  const auto state = readState();
  if (state.quality < 0) {
    return std::nullopt;
  }
  constexpr double kRadians = 3.14159265358979323846 / 180.0;
  static unsigned sample_index = 0;
  const double noise = (++sample_index % 2 ? 1 : -1) * state.length_noise;
  return Sample{(state.length + noise) / std::cos(9.0 * kRadians),
                static_cast<uint8_t>(state.quality)};
}
}  // namespace titania_extender
