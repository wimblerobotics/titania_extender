// Copyright 2026 titania_extender contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace titania_extender {

inline double gravityAngleDegrees(double x, double y, double z, double home_x,
                                  double home_y, double home_z) {
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm < 8.0 || norm > 11.0 ||
      std::hypot(x, y) < 1.0 || std::abs(z - home_z) > 0.5) {
    throw std::runtime_error(
        "invalid gravity or rotation outside IMU XY plane");
  }
  constexpr double kDegrees = 180.0 / 3.14159265358979323846;
  return std::remainder(
      (std::atan2(x, y) - std::atan2(home_x, home_y)) * kDegrees, 360.0);
}

inline double extensionLength(double range, double beam_degrees,
                              double offset) {
  constexpr double kRadians = 3.14159265358979323846 / 180.0;
  return range * std::cos(beam_degrees * kRadians) - offset;
}

inline int homingDirection(double error, double tolerance,
                           int positive_encoder_direction) {
  if (!std::isfinite(error)) {
    throw std::runtime_error("non-finite homing sensor reading");
  }
  if (std::abs(error) <= tolerance) {
    return 0;
  }
  return error > 0.0 ? positive_encoder_direction : -positive_encoder_direction;
}

inline double averageHomingReading(double previous, double current,
                                   double alpha, bool rotation) {
  if (!std::isfinite(current) || !std::isfinite(previous) ||
      !std::isfinite(alpha) || alpha <= 0.0 || alpha > 1.0) {
    throw std::runtime_error("invalid homing reading or smoothing alpha");
  }
  const double delta =
      rotation ? std::remainder(current - previous, 360.0) : current - previous;
  const double filtered = previous + alpha * delta;
  return rotation ? std::remainder(filtered, 360.0) : filtered;
}

inline int homingSpeed(double error, double tolerance, double approach_distance,
                       int max_speed, int min_speed, int direction,
                       bool& settling) {
  if (!std::isfinite(error)) {
    throw std::runtime_error("non-finite homing sensor reading");
  }
  // Enter the stop band slightly inside the accepted tolerance; only resume
  // movement if the filtered error leaves the full tolerance.
  settling = std::abs(error) <= tolerance * (settling ? 1.0 : 0.8);
  if (settling) {
    return 0;
  }
  const int speed = static_cast<int>(std::lround(
      max_speed * std::min(1.0, std::abs(error) / approach_distance)));
  return homingDirection(error, 0.0, direction) *
         std::clamp(speed, min_speed, max_speed);
}

}  // namespace titania_extender
