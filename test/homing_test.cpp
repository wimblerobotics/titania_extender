// Copyright 2026 titania_extender contributors
// SPDX-License-Identifier: Apache-2.0
#include "titania_extender/homing.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition) {
  if (!condition) {
    throw std::runtime_error("homing test failed");
  }
}

template <typename Function>
void checkThrows(Function function) {
  bool threw = false;
  try {
    function();
  } catch (const std::runtime_error&) {
    threw = true;
  }
  check(threw);
}
}  // namespace

int main() {
  using titania_extender::averageHomingReading;
  using titania_extender::extensionLength;
  using titania_extender::gravityAngleDegrees;
  using titania_extender::homingDirection;
  using titania_extender::homingSpeed;
  const auto angle = [](double x, double y, double z) {
    return gravityAngleDegrees(x, y, z, -0.25, 9.79, -0.32);
  };
  check(std::abs(angle(-0.25, 9.79, -0.32)) < 1e-10);
  check(std::abs(angle(3.98, 8.95, -0.34) - 25.438) < 0.01);
  check(angle(-3.98, 8.95, -0.34) < 0.0);
  checkThrows([&]() { angle(0.0, 0.0, 0.0); });
  checkThrows([&]() { angle(3.98, 8.95, 2.0); });
  checkThrows(
      [&]() { angle(std::numeric_limits<double>::quiet_NaN(), 9.79, 0); });
  check(std::abs(extensionLength(0.2, 0, 0.1) - 0.1) < 1e-10);
  check(std::abs(extensionLength(0.2, 9, 0.01) - 0.187537668) < 1e-8);
  check(homingDirection(0.1 - 0.2, 0.002, 1) == -1);
  check(homingDirection(0.1 - 0.05, 0.002, 1) == 1);
  check(homingDirection(-25.438, 1.0, -1) == 1);
  check(homingDirection(25.438, 1.0, -1) == -1);
  check(homingDirection(0.002, 0.002, 1) == 0);
  check(homingDirection(-1.0, 1.0, -1) == 0);
  checkThrows(
      []() { homingDirection(std::numeric_limits<double>::infinity(), 1, 1); });
  check(std::abs(averageHomingReading(0.1, 0.11, 0.35, false) - 0.1035) <
        1e-10);
  check(averageHomingReading(0.1, 0.11, 1.0, false) == 0.11);
  check(std::abs(averageHomingReading(179, -179, 0.5, true)) == 180);
  checkThrows([]() { averageHomingReading(0.1, 0.1, 0.0, false); });
  checkThrows([]() { averageHomingReading(0.1, 0.1, 1.1, false); });
  checkThrows([]() {
    averageHomingReading(0.1, std::numeric_limits<double>::quiet_NaN(), 0.35,
                         false);
  });
  bool settling = false;
  check(homingSpeed(-0.2, 0.005, 0.03, 600, 60, 1, settling) == -600);
  check(homingSpeed(-0.015, 0.005, 0.03, 600, 60, 1, settling) == -300);
  check(homingSpeed(-0.0045, 0.005, 0.03, 600, 60, 1, settling) == -90);
  check(homingSpeed(-0.0039, 0.005, 0.03, 600, 60, 1, settling) == 0);
  check(settling);
  check(homingSpeed(-0.0049, 0.005, 0.03, 600, 60, 1, settling) == 0);
  check(homingSpeed(0.0049, 0.005, 0.03, 600, 60, 1, settling) == 0);
  check(homingSpeed(0.0051, 0.005, 0.03, 600, 60, 1, settling) == 102);
  check(!settling);
  check(homingSpeed(-0.81, 1.0, 8.0, 300, 40, -1, settling) == 40);
  check(homingSpeed(25.0, 1.0, 8.0, 300, 30, -1, settling) == -300);
  check(homingSpeed(-0.7, 1.0, 8.0, 300, 30, -1, settling) == 0);
  double filtered = 0.1;
  for (int i = 0; i < 20; ++i) {
    filtered =
        averageHomingReading(filtered, i % 2 ? 0.104 : 0.096, 0.35, false);
    check(std::abs(filtered - 0.1) <= 0.0015);
  }
  std::cout << "Homing geometry and direction tests passed\n";
}
