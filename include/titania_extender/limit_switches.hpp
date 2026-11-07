// Motion limit stop sensors (optical or hall-effect, 3-pin devices) for
// the extender rotation and extension axes.
//
// DUMMY IMPLEMENTATION: the sensors are not yet installed and the GPIO pin
// assignments will be specified later. Until then every query returns
// "not triggered" so motion is bounded only by the RoboClaw's configured
// min/max position limits.
//
// TODO(wimblerobotics): implement GPIO reads (libgpiod on the Raspberry
// Pi 5 — note the Pi 5 uses gpiochip4) with debouncing, and decide
// active-high vs active-low once the sensor part is chosen.

#pragma once

namespace titania_extender {

class LimitSwitches {
 public:
  enum Axis { kRotate = 0, kExtend = 1 };

  LimitSwitches(int rotate_min_pin, int rotate_max_pin, int extend_min_pin,
                int extend_max_pin, bool enabled)
      : rotate_min_pin_(rotate_min_pin),
        rotate_max_pin_(rotate_max_pin),
        extend_min_pin_(extend_min_pin),
        extend_max_pin_(extend_max_pin),
        enabled_(enabled) {}

  bool initialize() {
    // TODO: request GPIO lines as inputs with pull-ups via libgpiod.
    available_ = false;
    return available_;
  }

  bool isAvailable() const { return enabled_ && available_; }

  // True when the axis is at its minimum travel limit.
  bool isMinTriggered(Axis) const {
    // TODO: read the GPIO line and apply debounce.
    return false;
  }

  // True when the axis is at its maximum travel limit.
  bool isMaxTriggered(Axis) const {
    // TODO: read the GPIO line and apply debounce.
    return false;
  }

 private:
  int rotate_min_pin_;
  int rotate_max_pin_;
  int extend_min_pin_;
  int extend_max_pin_;
  bool enabled_;
  bool available_ = false;
};

}  // namespace titania_extender
