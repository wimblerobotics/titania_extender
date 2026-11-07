// titania_extender_node: ROS 2 driver for the gripper extender assembly of
// the Titania robot.
//
// Drives two motors through a Basic Micro RoboClaw over USB:
//   M1 "rotate": rotates the extender.
//   M2 "extend": extends/retracts the box tube extender.
//
// Interfaces (all names relative to the node):
//   Actions:
//     rotate/move_to_position  (titania_extender/action/MoveToPosition)
//     extend/move_to_position  (titania_extender/action/MoveToPosition)
//   Topics (subscribed, fire-and-forget moves with configured defaults):
//     rotate/cmd_position, extend/cmd_position              (std_msgs/Int32)
//     rotate/cmd_position_relative, extend/cmd_position_relative
//   Topics (published):
//     rotate/status, extend/status   (titania_extender/msg/MotorStatus)
//     rotation_angle                 (std_msgs/Float32, degrees)
//     extension_length               (std_msgs/Float32, meters)
//   Services:
//     home            (std_srvs/Trigger)  -- dummy, not yet implemented
//     reset_encoders  (std_srvs/Trigger)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "titania_extender/action/move_to_position.hpp"
#include "titania_extender/bno055.hpp"
#include "titania_extender/limit_switches.hpp"
#include "titania_extender/msg/motor_status.hpp"
#include "titania_extender/roboclaw.hpp"
#include "titania_extender/vl53l4cd.hpp"

using namespace std::chrono_literals;

namespace titania_extender {

using MoveToPosition = titania_extender::action::MoveToPosition;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveToPosition>;

struct MotorConfig {
  std::string name;
  RoboClaw::Motor motor;

  double velocity_p;
  double velocity_i;
  double velocity_d;
  int qpps;

  double position_p;
  double position_i;
  double position_d;
  int position_max_i;
  int position_deadzone;

  int min_position;
  int max_position;

  int speed;
  int accel;
  int decel;

  double max_current;

  // Encoder counts per engineering unit (degree for rotate, meter for
  // extend). 0.0 => calibration unknown, derived topic publishes NaN.
  double counts_per_unit;

  LimitSwitches::Axis axis;
};

class MotorContext {
 public:
  MotorConfig config;

  rclcpp_action::Server<MoveToPosition>::SharedPtr action_server;
  rclcpp::Publisher<titania_extender::msg::MotorStatus>::SharedPtr status_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_relative_sub;

  std::atomic<int32_t> target_position{0};
  std::atomic<bool> move_active{false};

  // The goal currently being executed, protected by goal_mutex. A newly
  // accepted goal preempts (aborts) the previous one.
  std::mutex goal_mutex;
  std::shared_ptr<GoalHandleMove> active_goal;

  // Latest values sensed by the status timer, for cheap reuse.
  std::atomic<int32_t> last_encoder{0};
  std::atomic<bool> over_current{false};
};

class TitaniaExtenderNode : public rclcpp::Node {
 public:
  TitaniaExtenderNode() : rclcpp::Node("titania_extender_node") {
    declareParameters();
    loadMotorConfig(rotate_, "rotate", RoboClaw::kM1, LimitSwitches::kRotate);
    loadMotorConfig(extend_, "extend", RoboClaw::kM2, LimitSwitches::kExtend);

    connectRoboClaw();
    initializeSensors();

    rotation_angle_pub_ =
        create_publisher<std_msgs::msg::Float32>("rotation_angle", 10);
    extension_length_pub_ =
        create_publisher<std_msgs::msg::Float32>("extension_length", 10);

    setupMotorInterfaces(rotate_);
    setupMotorInterfaces(extend_);

    home_service_ = create_service<std_srvs::srv::Trigger>(
        "home", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response) {
          homeMotors(*response);
        });
    reset_encoders_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_encoders",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          resetEncoders(*response);
        });

    const double sensor_rate_hz = get_parameter("sensor_rate_hz").as_double();
    status_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / sensor_rate_hz),
        [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "titania_extender_node ready");
  }

  ~TitaniaExtenderNode() override {
    if (roboclaw_) {
      try {
        roboclaw_->stopAll();
      } catch (const RoboClawError &e) {
        RCLCPP_WARN(get_logger(), "stopAll on shutdown failed: %s", e.what());
      }
    }
  }

 private:
  // ----- Parameter handling ----------------------------------------------

  void declareParameters() {
    declare_parameter("device_name", "/dev/ttyACM0");
    declare_parameter("baud_rate", 38400);
    declare_parameter("device_port", 128);
    declare_parameter("sensor_rate_hz", 10.0);
    declare_parameter("goal_tolerance", 10);
    declare_parameter("goal_timeout_seconds", 30.0);
    declare_parameter("do_debug", false);

    for (const std::string prefix : {"rotate", "extend"}) {
      declare_parameter(prefix + ".velocity_p", 0.0);
      declare_parameter(prefix + ".velocity_i", 0.0);
      declare_parameter(prefix + ".velocity_d", 0.0);
      declare_parameter(prefix + ".qpps", 0);
      declare_parameter(prefix + ".position_p", 0.0);
      declare_parameter(prefix + ".position_i", 0.0);
      declare_parameter(prefix + ".position_d", 0.0);
      declare_parameter(prefix + ".position_max_i", 0);
      declare_parameter(prefix + ".position_deadzone", 10);
      declare_parameter(prefix + ".min_position", 0);
      declare_parameter(prefix + ".max_position", 0);
      declare_parameter(prefix + ".speed", 0);
      declare_parameter(prefix + ".accel", 0);
      declare_parameter(prefix + ".decel", 0);
      declare_parameter(prefix + ".max_current", 6.0);
    }
    declare_parameter("rotate.counts_per_degree", 0.0);
    declare_parameter("extend.counts_per_meter", 0.0);

    declare_parameter("imu.enabled", false);
    declare_parameter("imu.i2c_bus", 1);
    declare_parameter("imu.i2c_address", 0x28);

    declare_parameter("tof.enabled", false);
    declare_parameter("tof.i2c_bus", 1);
    declare_parameter("tof.i2c_address", 0x29);
    declare_parameter("tof.mount_offset_meters", 0.0);

    declare_parameter("limit_switches.enabled", false);
    declare_parameter("limit_switches.rotate_min_pin", -1);
    declare_parameter("limit_switches.rotate_max_pin", -1);
    declare_parameter("limit_switches.extend_min_pin", -1);
    declare_parameter("limit_switches.extend_max_pin", -1);
  }

  void loadMotorConfig(MotorContext &context, const std::string &name,
                       RoboClaw::Motor motor, LimitSwitches::Axis axis) {
    MotorConfig &config = context.config;
    config.name = name;
    config.motor = motor;
    config.axis = axis;
    config.velocity_p = get_parameter(name + ".velocity_p").as_double();
    config.velocity_i = get_parameter(name + ".velocity_i").as_double();
    config.velocity_d = get_parameter(name + ".velocity_d").as_double();
    config.qpps = get_parameter(name + ".qpps").as_int();
    config.position_p = get_parameter(name + ".position_p").as_double();
    config.position_i = get_parameter(name + ".position_i").as_double();
    config.position_d = get_parameter(name + ".position_d").as_double();
    config.position_max_i = get_parameter(name + ".position_max_i").as_int();
    config.position_deadzone =
        get_parameter(name + ".position_deadzone").as_int();
    config.min_position = get_parameter(name + ".min_position").as_int();
    config.max_position = get_parameter(name + ".max_position").as_int();
    config.speed = get_parameter(name + ".speed").as_int();
    config.accel = get_parameter(name + ".accel").as_int();
    config.decel = get_parameter(name + ".decel").as_int();
    config.max_current = get_parameter(name + ".max_current").as_double();
    config.counts_per_unit =
        name == "rotate" ? get_parameter("rotate.counts_per_degree").as_double()
                         : get_parameter("extend.counts_per_meter").as_double();
  }

  // ----- Hardware setup ----------------------------------------------------

  void connectRoboClaw() {
    const std::string device_name = get_parameter("device_name").as_string();
    const uint32_t baud_rate =
        static_cast<uint32_t>(get_parameter("baud_rate").as_int());
    const uint8_t address =
        static_cast<uint8_t>(get_parameter("device_port").as_int());

    roboclaw_ = std::make_unique<RoboClaw>(device_name, baud_rate, address);
    RCLCPP_INFO(get_logger(), "Connected to RoboClaw: %s",
                roboclaw_->getVersion().c_str());

    for (MotorContext *context : {&rotate_, &extend_}) {
      const MotorConfig &config = context->config;
      roboclaw_->setVelocityPid(config.motor,
                                static_cast<float>(config.velocity_p),
                                static_cast<float>(config.velocity_i),
                                static_cast<float>(config.velocity_d),
                                static_cast<uint32_t>(config.qpps));
      roboclaw_->setPositionPid(
          config.motor, static_cast<float>(config.position_p),
          static_cast<float>(config.position_i),
          static_cast<float>(config.position_d),
          static_cast<uint32_t>(config.position_max_i),
          static_cast<uint32_t>(config.position_deadzone),
          config.min_position, config.max_position);
      RCLCPP_INFO(get_logger(),
                  "%s (M%d): vel PID %.5f/%.5f/%.5f qpps=%d, pos PID "
                  "%.5f/%.5f/%.5f range [%d, %d]",
                  config.name.c_str(), config.motor + 1, config.velocity_p,
                  config.velocity_i, config.velocity_d, config.qpps,
                  config.position_p, config.position_i, config.position_d,
                  config.min_position, config.max_position);
    }

    // Startup policy: assume the current physical position is zero.
    // TODO: replace with a homing procedure using the IMU and limit
    // switches (see homeMotors()).
    roboclaw_->resetEncoders();
    roboclaw_->stopAll();
    RCLCPP_INFO(get_logger(), "Encoders reset; current position is zero");
  }

  void initializeSensors() {
    imu_ = std::make_unique<Bno055>(
        get_parameter("imu.i2c_bus").as_int(),
        static_cast<uint8_t>(get_parameter("imu.i2c_address").as_int()),
        get_parameter("imu.enabled").as_bool());
    if (!imu_->initialize()) {
      RCLCPP_WARN(get_logger(),
                  "BNO055 IMU not available (dummy implementation); "
                  "rotation_angle will use encoder calibration or NaN");
    }

    tof_ = std::make_unique<Vl53l4cd>(
        get_parameter("tof.i2c_bus").as_int(),
        static_cast<uint8_t>(get_parameter("tof.i2c_address").as_int()),
        get_parameter("tof.mount_offset_meters").as_double(),
        get_parameter("tof.enabled").as_bool());
    if (!tof_->initialize()) {
      RCLCPP_WARN(get_logger(),
                  "VL53L4CD not available (dummy implementation); "
                  "extension_length will use encoder calibration or NaN");
    }

    limit_switches_ = std::make_unique<LimitSwitches>(
        get_parameter("limit_switches.rotate_min_pin").as_int(),
        get_parameter("limit_switches.rotate_max_pin").as_int(),
        get_parameter("limit_switches.extend_min_pin").as_int(),
        get_parameter("limit_switches.extend_max_pin").as_int(),
        get_parameter("limit_switches.enabled").as_bool());
    if (!limit_switches_->initialize()) {
      RCLCPP_WARN(get_logger(),
                  "Limit switches not available (dummy implementation); "
                  "motion is bounded only by RoboClaw position limits");
    }
  }

  // ----- Per-motor ROS interfaces ------------------------------------------

  void setupMotorInterfaces(MotorContext &context) {
    const std::string &name = context.config.name;

    context.status_pub = create_publisher<titania_extender::msg::MotorStatus>(
        name + "/status", 10);

    context.cmd_sub = create_subscription<std_msgs::msg::Int32>(
        name + "/cmd_position", 10,
        [this, &context](std_msgs::msg::Int32::ConstSharedPtr msg) {
          startSimpleMove(context, msg->data, false);
        });
    context.cmd_relative_sub = create_subscription<std_msgs::msg::Int32>(
        name + "/cmd_position_relative", 10,
        [this, &context](std_msgs::msg::Int32::ConstSharedPtr msg) {
          startSimpleMove(context, msg->data, true);
        });

    context.action_server = rclcpp_action::create_server<MoveToPosition>(
        this, name + "/move_to_position",
        [this, &context](const rclcpp_action::GoalUUID &,
                         MoveToPosition::Goal::ConstSharedPtr goal) {
          return handleGoal(context, *goal);
        },
        [this, &context](const std::shared_ptr<GoalHandleMove> &) {
          RCLCPP_INFO(get_logger(), "%s: cancel requested",
                      context.config.name.c_str());
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, &context](const std::shared_ptr<GoalHandleMove> &goal_handle) {
          handleAccepted(context, goal_handle);
        });
  }

  rclcpp_action::GoalResponse handleGoal(MotorContext &context,
                                         const MoveToPosition::Goal &goal) {
    if (!roboclaw_) {
      RCLCPP_ERROR(get_logger(), "%s: rejecting goal, RoboClaw not connected",
                   context.config.name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (context.over_current.load()) {
      RCLCPP_ERROR(get_logger(),
                   "%s: rejecting goal, motor is in over-current state",
                   context.config.name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handleAccepted(MotorContext &context,
                      const std::shared_ptr<GoalHandleMove> &goal_handle) {
    {
      // Preempt any goal already running on this motor. The executing
      // thread notices active_goal changed and finishes as aborted.
      std::lock_guard<std::mutex> lock(context.goal_mutex);
      if (context.active_goal && context.active_goal->is_active()) {
        RCLCPP_WARN(get_logger(), "%s: preempting active goal",
                    context.config.name.c_str());
      }
      context.active_goal = goal_handle;
    }
    std::thread([this, &context, goal_handle]() {
      executeGoal(context, goal_handle);
    }).detach();
  }

  // Clamps a target to the configured limits, warning when clamped.
  int32_t clampTarget(const MotorConfig &config, int64_t target) {
    const int64_t clamped =
        std::clamp<int64_t>(target, config.min_position, config.max_position);
    if (clamped != target) {
      RCLCPP_WARN(get_logger(),
                  "%s: target %ld clamped to %ld (limits [%d, %d])",
                  config.name.c_str(), static_cast<long>(target),
                  static_cast<long>(clamped), config.min_position,
                  config.max_position);
    }
    return static_cast<int32_t>(clamped);
  }

  void startSimpleMove(MotorContext &context, int32_t position,
                       bool relative) {
    if (!roboclaw_) {
      return;
    }
    const MotorConfig &config = context.config;
    try {
      int64_t target = position;
      if (relative) {
        target += roboclaw_->readEncoder(config.motor).value;
      }
      const int32_t clamped = clampTarget(config, target);
      roboclaw_->moveToPosition(config.motor,
                                static_cast<uint32_t>(config.accel),
                                static_cast<uint32_t>(config.speed),
                                static_cast<uint32_t>(config.decel), clamped);
      context.target_position.store(clamped);
      RCLCPP_INFO(get_logger(), "%s: cmd_position%s -> %d",
                  config.name.c_str(), relative ? "_relative" : "", clamped);
    } catch (const RoboClawError &e) {
      RCLCPP_ERROR(get_logger(), "%s: cmd_position failed: %s",
                   config.name.c_str(), e.what());
    }
  }

  void executeGoal(MotorContext &context,
                   const std::shared_ptr<GoalHandleMove> &goal_handle) {
    const MotorConfig &config = context.config;
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveToPosition::Result>();

    const int32_t tolerance =
        static_cast<int32_t>(get_parameter("goal_tolerance").as_int());
    const double timeout_seconds =
        get_parameter("goal_timeout_seconds").as_double();

    const uint32_t speed =
        goal->speed != 0 ? goal->speed : static_cast<uint32_t>(config.speed);
    const uint32_t accel =
        goal->accel != 0 ? goal->accel : static_cast<uint32_t>(config.accel);
    const uint32_t decel =
        goal->decel != 0 ? goal->decel : static_cast<uint32_t>(config.decel);

    int32_t target = 0;
    try {
      int64_t requested = goal->position;
      if (goal->relative) {
        requested += roboclaw_->readEncoder(config.motor).value;
      }
      target = clampTarget(config, requested);
      roboclaw_->moveToPosition(config.motor, accel, speed, decel, target);
    } catch (const RoboClawError &e) {
      result->success = false;
      result->message = std::string("failed to start move: ") + e.what();
      goal_handle->abort(result);
      return;
    }

    context.target_position.store(target);
    context.move_active.store(true);
    RCLCPP_INFO(get_logger(), "%s: moving to %d (speed=%u accel=%u decel=%u)",
                config.name.c_str(), target, speed, accel, decel);

    auto feedback = std::make_shared<MoveToPosition::Feedback>();
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_seconds));
    const auto finish = [&](bool stop_motor) {
      if (stop_motor) {
        try {
          roboclaw_->stop(config.motor);
        } catch (const RoboClawError &e) {
          RCLCPP_ERROR(get_logger(), "%s: stop failed: %s",
                       config.name.c_str(), e.what());
        }
      }
      std::lock_guard<std::mutex> lock(context.goal_mutex);
      if (context.active_goal == goal_handle) {
        context.active_goal.reset();
        context.move_active.store(false);
      }
    };

    rclcpp::Rate rate(20.0);
    while (rclcpp::ok()) {
      // Preempted by a newer goal?
      {
        std::lock_guard<std::mutex> lock(context.goal_mutex);
        if (context.active_goal != goal_handle) {
          result->success = false;
          result->message = "preempted by a newer goal";
          result->final_position = context.last_encoder.load();
          goal_handle->abort(result);
          return;  // The new goal owns the motor; do not stop it.
        }
      }

      int32_t encoder = 0;
      int32_t speed_now = 0;
      try {
        encoder = roboclaw_->readEncoder(config.motor).value;
        speed_now = roboclaw_->readSpeed(config.motor);
      } catch (const RoboClawError &e) {
        result->success = false;
        result->message = std::string("communication error: ") + e.what();
        result->final_position = context.last_encoder.load();
        finish(true);
        goal_handle->abort(result);
        return;
      }
      context.last_encoder.store(encoder);

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "canceled";
        result->final_position = encoder;
        finish(true);
        goal_handle->canceled(result);
        return;
      }

      // Safety aborts: limit switches (dummy, always false for now) and
      // over-current (flag set by the status timer).
      const bool limit_hit =
          limit_switches_->isAvailable() &&
          ((target < encoder &&
            limit_switches_->isMinTriggered(config.axis)) ||
           (target > encoder &&
            limit_switches_->isMaxTriggered(config.axis)));
      if (limit_hit || context.over_current.load()) {
        result->success = false;
        result->message = limit_hit ? "limit switch triggered" : "over-current";
        result->final_position = encoder;
        finish(true);
        goal_handle->abort(result);
        return;
      }

      const int32_t remaining = target - encoder;
      feedback->current_position = encoder;
      feedback->current_speed = speed_now;
      feedback->remaining_counts = remaining;
      goal_handle->publish_feedback(feedback);

      if (std::abs(remaining) <= tolerance) {
        result->success = true;
        result->message = "goal reached";
        result->final_position = encoder;
        // Leave the RoboClaw in position mode so it holds the target.
        finish(false);
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "%s: reached %d (target %d)",
                    config.name.c_str(), encoder, target);
        return;
      }

      if (std::chrono::steady_clock::now() > deadline) {
        result->success = false;
        result->message = "timed out";
        result->final_position = encoder;
        finish(true);
        goal_handle->abort(result);
        return;
      }

      rate.sleep();
    }

    result->success = false;
    result->message = "node shutting down";
    finish(true);
    goal_handle->abort(result);
  }

  // ----- Services ------------------------------------------------------------

  // DUMMY homing procedure. Planned behavior: drive each axis slowly toward
  // its minimum limit switch, use the BNO055 gravity vector to confirm the
  // rotation angle, zero the encoders at the reference position, then move
  // to a configured ready position. Not called automatically yet.
  void homeMotors(std_srvs::srv::Trigger::Response &response) {
    // TODO(wimblerobotics): implement once limit switches and IMU exist.
    RCLCPP_WARN(get_logger(), "homeMotors: not implemented (dummy)");
    response.success = false;
    response.message =
        "homing not implemented yet; waiting on limit switch and IMU hardware";
  }

  void resetEncoders(std_srvs::srv::Trigger::Response &response) {
    try {
      roboclaw_->resetEncoders();
      response.success = true;
      response.message = "encoders reset to zero";
      RCLCPP_INFO(get_logger(), "Encoders reset to zero");
    } catch (const RoboClawError &e) {
      response.success = false;
      response.message = e.what();
    }
  }

  // ----- Periodic status -------------------------------------------------------

  void publishStatus() {
    if (!roboclaw_) {
      return;
    }
    float m1_current = 0.0f;
    float m2_current = 0.0f;
    float temperature = 0.0f;
    float main_battery = 0.0f;
    uint32_t error_status = 0;
    try {
      std::tie(m1_current, m2_current) = roboclaw_->readCurrents();
      temperature = roboclaw_->readTemperature();
      main_battery = roboclaw_->readMainBatteryVoltage();
      error_status = roboclaw_->readErrorStatus();
    } catch (const RoboClawError &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "status read failed: %s", e.what());
      return;
    }

    const auto stamp = now();
    publishMotorStatus(rotate_, stamp, m1_current, temperature, main_battery,
                       error_status);
    publishMotorStatus(extend_, stamp, m2_current, temperature, main_battery,
                       error_status);
    publishDerivedState();
  }

  void publishMotorStatus(MotorContext &context, const rclcpp::Time &stamp,
                          float current, float temperature,
                          float main_battery, uint32_t error_status) {
    const MotorConfig &config = context.config;
    titania_extender::msg::MotorStatus status;
    status.header.stamp = stamp;
    status.name = config.name;
    try {
      const RoboClaw::EncoderResult encoder =
          roboclaw_->readEncoder(config.motor);
      status.encoder = encoder.value;
      status.encoder_status = encoder.status;
      status.speed = roboclaw_->readSpeed(config.motor);
      context.last_encoder.store(encoder.value);
    } catch (const RoboClawError &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%s: encoder read failed: %s", config.name.c_str(),
                           e.what());
      return;
    }
    status.current = current;
    status.temperature = temperature;
    status.main_battery_voltage = main_battery;
    status.error_status = error_status;
    status.error_string = RoboClaw::errorString(error_status);
    status.target_position = context.target_position.load();
    status.move_active = context.move_active.load();
    status.min_limit_active = limit_switches_->isAvailable() &&
                              limit_switches_->isMinTriggered(config.axis);
    status.max_limit_active = limit_switches_->isAvailable() &&
                              limit_switches_->isMaxTriggered(config.axis);
    context.status_pub->publish(status);

    // Over-current monitoring: latch a warning flag that aborts the active
    // goal; cleared when the current drops back below the limit.
    if (current > config.max_current) {
      if (!context.over_current.exchange(true)) {
        RCLCPP_ERROR(get_logger(), "%s: over-current: %.2f A (limit %.2f A)",
                     config.name.c_str(), current, config.max_current);
      }
    } else {
      context.over_current.store(false);
    }
  }

  // Publishes rotation_angle (degrees) and extension_length (meters).
  // Prefers real sensors (IMU gravity vector, ToF range); falls back to
  // encoder counts with YAML calibration; publishes NaN when unknown.
  void publishDerivedState() {
    std_msgs::msg::Float32 angle;
    angle.data = std::numeric_limits<float>::quiet_NaN();
    if (imu_->isAvailable()) {
      if (auto degrees = imu_->readRotationAngleDegrees()) {
        angle.data = static_cast<float>(*degrees);
      }
    } else if (rotate_.config.counts_per_unit != 0.0) {
      angle.data = static_cast<float>(rotate_.last_encoder.load() /
                                      rotate_.config.counts_per_unit);
    }
    rotation_angle_pub_->publish(angle);

    std_msgs::msg::Float32 length;
    length.data = std::numeric_limits<float>::quiet_NaN();
    if (tof_->isAvailable()) {
      if (auto meters = tof_->readExtensionMeters()) {
        length.data = static_cast<float>(*meters);
      }
    } else if (extend_.config.counts_per_unit != 0.0) {
      length.data = static_cast<float>(extend_.last_encoder.load() /
                                       extend_.config.counts_per_unit);
    }
    extension_length_pub_->publish(length);
  }

  // ----- Members ---------------------------------------------------------------

  std::unique_ptr<RoboClaw> roboclaw_;
  std::unique_ptr<Bno055> imu_;
  std::unique_ptr<Vl53l4cd> tof_;
  std::unique_ptr<LimitSwitches> limit_switches_;

  MotorContext rotate_;
  MotorContext extend_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr extension_length_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoders_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace titania_extender

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<titania_extender::TitaniaExtenderNode>();
    rclcpp::spin(node);
  } catch (const titania_extender::RoboClawError &e) {
    RCLCPP_FATAL(rclcpp::get_logger("titania_extender_node"),
                 "RoboClaw error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
