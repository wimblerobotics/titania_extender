// SPDX-License-Identifier: Apache-2.0
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
//   imu/data, imu/gravity, extender_tof/range, extender_tof/range_status
//   Services:
//     home, cancel_home (std_srvs/Trigger)
//     reset_encoders  (std_srvs/Trigger)

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

#include "titania_extender/action/move_to_position.hpp"
#include "titania_extender/bno055.hpp"
#include "titania_extender/homing.hpp"
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
  static constexpr double kPi = 3.14159265358979323846;

 public:
  TitaniaExtenderNode() : rclcpp::Node("titania_extender_node") {
    declareParameters();
    loadMotorConfig(rotate_, "rotate", RoboClaw::kM1, LimitSwitches::kRotate);
    loadMotorConfig(extend_, "extend", RoboClaw::kM2, LimitSwitches::kExtend);

    connectRoboClaw();
    initializeSensors();

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    gravity_pub_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/gravity", 10);
    tof_range_pub_ =
        create_publisher<sensor_msgs::msg::Range>("extender_tof/range", 10);
    tof_status_pub_ =
        create_publisher<std_msgs::msg::UInt8>("extender_tof/range_status", 10);
    rotation_angle_pub_ =
        create_publisher<std_msgs::msg::Float32>("rotation_angle", 10);
    extension_length_pub_ =
        create_publisher<std_msgs::msg::Float32>("extension_length", 10);

    setupMotorInterfaces(rotate_);
    setupMotorInterfaces(extend_);

    home_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    home_service_ = create_service<std_srvs::srv::Trigger>(
        "home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          homeMotors(*response);
        },
        rclcpp::ServicesQoS(), home_callback_group_);
    cancel_home_service_ = create_service<std_srvs::srv::Trigger>(
        "cancel_home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          cancel_home_.store(true);
          response->success = homing_.load();
          response->message = response->success
                                  ? "homing cancellation requested"
                                  : "homing is not active";
        });
    reset_encoders_service_ = create_service<std_srvs::srv::Trigger>(
        "reset_encoders",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          resetEncoders(*response);
        });

    const double sensor_rate_hz = get_parameter("sensor_rate_hz").as_double();
    status_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / sensor_rate_hz),
                          [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "titania_extender_node ready");
  }

  ~TitaniaExtenderNode() override {
    if (roboclaw_) {
      try {
        roboclaw_->stopAll();
      } catch (const RoboClawError& e) {
        RCLCPP_WARN(get_logger(), "stopAll on shutdown failed: %s", e.what());
      }
    }
  }

 private:
  // ----- Parameter handling ----------------------------------------------

  void declareParameters() {
    declare_parameter("device_name", "/dev/roboclaw");
    declare_parameter("baud_rate", 38400);
    declare_parameter("device_port", 128);
    declare_parameter("sensor_rate_hz", 10.0);
    declare_parameter("goal_tolerance", 10);
    declare_parameter("goal_timeout_seconds", 30.0);
    declare_parameter("do_debug", false);
    declare_parameter("homing.extension_meters", 0.1);
    declare_parameter("homing.extension_tolerance_meters", 0.005);
    declare_parameter("homing.rotation_tolerance_degrees", 1.0);
    declare_parameter("homing.smoothing_alpha", 0.35);
    declare_parameter("homing.extension_approach_meters", 0.03);
    declare_parameter("homing.rotation_approach_degrees", 8.0);
    declare_parameter("homing.extend_min_speed", 60);
    declare_parameter("homing.rotate_min_speed", 30);
    declare_parameter("homing.gravity.x", 0.22);
    declare_parameter("homing.gravity.y", 9.80);
    declare_parameter("homing.gravity.z", -0.18);
    declare_parameter("homing.rotate_direction", -1);
    declare_parameter("homing.extend_direction", 1);
    declare_parameter("homing.rotate_speed", 300);
    declare_parameter("homing.extend_speed", 600);
    declare_parameter("homing.rotate_max_travel_counts", 6000);
    declare_parameter("homing.extend_max_travel_counts", 16000);
    declare_parameter("homing.timeout_seconds", 60.0);
    declare_parameter("homing.sensor_timeout_seconds", 0.5);
    declare_parameter("homing.settle_samples", 5);

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
    declare_parameter("imu.zero_orientation_set", false);
    declare_parameter("imu.zero_orientation.x", 0.0);
    declare_parameter("imu.zero_orientation.y", 0.0);
    declare_parameter("imu.zero_orientation.z", 0.0);
    declare_parameter("imu.zero_orientation.w", 1.0);

    declare_parameter("tof.enabled", false);
    declare_parameter("tof.i2c_bus", 1);
    declare_parameter("tof.i2c_address", 0x29);
    declare_parameter("tof.beam_angle_degrees", 0.0);
    declare_parameter("tof.mount_offset_meters", 0.0);

    declare_parameter("limit_switches.enabled", false);
    declare_parameter("limit_switches.rotate_min_pin", -1);
    declare_parameter("limit_switches.rotate_max_pin", -1);
    declare_parameter("limit_switches.extend_min_pin", -1);
    declare_parameter("limit_switches.extend_max_pin", -1);
  }

  void loadMotorConfig(MotorContext& context, const std::string& name,
                       RoboClaw::Motor motor, LimitSwitches::Axis axis) {
    MotorConfig& config = context.config;
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

    for (MotorContext* context : {&rotate_, &extend_}) {
      const MotorConfig& config = context->config;
      roboclaw_->setVelocityPid(config.motor,
                                static_cast<float>(config.velocity_p),
                                static_cast<float>(config.velocity_i),
                                static_cast<float>(config.velocity_d),
                                static_cast<uint32_t>(config.qpps));
      roboclaw_->setPositionPid(config.motor,
                                static_cast<float>(config.position_p),
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

    // Retain the startup coordinate policy; this is not sensor homing.
    roboclaw_->resetEncoders();
    roboclaw_->stopAll();
    RCLCPP_INFO(get_logger(), "Encoders reset; current position is zero");
  }

  void initializeSensors() {
    imu_zero_orientation_set_ =
        get_parameter("imu.zero_orientation_set").as_bool();
    if (imu_zero_orientation_set_) {
      imu_zero_orientation_ = {
          get_parameter("imu.zero_orientation.x").as_double(),
          get_parameter("imu.zero_orientation.y").as_double(),
          get_parameter("imu.zero_orientation.z").as_double(),
          get_parameter("imu.zero_orientation.w").as_double()};
      const double norm = quaternionNorm(imu_zero_orientation_);
      if (norm < 1e-6) {
        RCLCPP_WARN(get_logger(),
                    "Configured IMU zero quaternion is invalid; capturing "
                    "the startup orientation instead");
        imu_zero_orientation_set_ = false;
      } else {
        for (double& component : imu_zero_orientation_) {
          component /= norm;
        }
      }
    }

    imu_ = std::make_unique<Bno055>(
        get_parameter("imu.i2c_bus").as_int(),
        static_cast<uint8_t>(get_parameter("imu.i2c_address").as_int()),
        get_parameter("imu.enabled").as_bool());
    if (!imu_->initialize()) {
      RCLCPP_WARN(get_logger(), "BNO055 IMU unavailable: %s",
                  imu_->lastError().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "BNO055 initialized in IMUPLUS mode");
      if (auto sample = imu_->readSample()) {
        logImuSample(*sample);
        if (!imu_zero_orientation_set_) {
          captureImuZero(*sample);
        }
      }
    }

    tof_ = std::make_unique<Vl53l4cd>(
        get_parameter("tof.i2c_bus").as_int(),
        static_cast<uint8_t>(get_parameter("tof.i2c_address").as_int()),
        get_parameter("tof.enabled").as_bool());
    if (!tof_->initialize()) {
      RCLCPP_WARN(get_logger(), "VL53L4CD unavailable: %s",
                  tof_->lastError().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "VL53L4CD initialized and ranging");
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

  static double quaternionNorm(const std::array<double, 4>& quaternion) {
    double sum = 0.0;
    for (double component : quaternion) {
      sum += component * component;
    }
    return std::sqrt(sum);
  }

  void logImuSample(const Bno055::Sample& sample) {
    RCLCPP_INFO(get_logger(),
                "IMU orientation x=%.6f y=%.6f z=%.6f w=%.6f; gravity "
                "x=%.3f y=%.3f z=%.3f m/s^2",
                sample.orientation_x, sample.orientation_y,
                sample.orientation_z, sample.orientation_w, sample.gravity_x,
                sample.gravity_y, sample.gravity_z);
  }

  void captureImuZero(const Bno055::Sample& sample) {
    imu_zero_orientation_ = {sample.orientation_x, sample.orientation_y,
                             sample.orientation_z, sample.orientation_w};
    const double norm = quaternionNorm(imu_zero_orientation_);
    if (norm < 1e-6) {
      return;
    }
    for (double& component : imu_zero_orientation_) {
      component /= norm;
    }
    imu_zero_orientation_set_ = true;
    RCLCPP_INFO(get_logger(),
                "Captured IMU zero quaternion x=%.6f y=%.6f z=%.6f w=%.6f",
                imu_zero_orientation_[0], imu_zero_orientation_[1],
                imu_zero_orientation_[2], imu_zero_orientation_[3]);
  }

  std::optional<double> rotationAngleDegrees(
      const Bno055::Sample& sample) const {
    const std::array<double, 4> current{
        sample.orientation_x, sample.orientation_y, sample.orientation_z,
        sample.orientation_w};
    const double current_norm = quaternionNorm(current);
    if (current_norm < 1e-6 || !imu_zero_orientation_set_) {
      return std::nullopt;
    }

    const double x = current[0] / current_norm;
    const double y = current[1] / current_norm;
    const double z = current[2] / current_norm;
    const double w = current[3] / current_norm;
    const double zx = imu_zero_orientation_[0];
    const double zy = imu_zero_orientation_[1];
    const double zz = imu_zero_orientation_[2];
    const double zw = imu_zero_orientation_[3];

    double dx = -w * zx + x * zw - y * zz + z * zy;
    double dy = -w * zy + x * zz + y * zw - z * zx;
    double dz = -w * zz - x * zy + y * zx + z * zw;
    double dw = w * zw + x * zx + y * zy + z * zz;
    if (dw < 0.0) {
      dx = -dx;
      dy = -dy;
      dz = -dz;
      dw = -dw;
    }

    const double vector_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    double angle = 2.0 * std::atan2(vector_norm, dw) * 180.0 / kPi;
    const double dominant_axis =
        std::abs(dx) >= std::abs(dy) && std::abs(dx) >= std::abs(dz)
            ? dx
            : (std::abs(dy) >= std::abs(dz) ? dy : dz);
    if (dominant_axis < 0.0) {
      angle = -angle;
    }
    return angle;
  }

  // ----- Per-motor ROS interfaces ------------------------------------------

  void setupMotorInterfaces(MotorContext& context) {
    const std::string& name = context.config.name;

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
        [this, &context](const rclcpp_action::GoalUUID&,
                         MoveToPosition::Goal::ConstSharedPtr goal) {
          return handleGoal(context, *goal);
        },
        [this, &context](const std::shared_ptr<GoalHandleMove>&) {
          RCLCPP_INFO(get_logger(), "%s: cancel requested",
                      context.config.name.c_str());
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, &context](const std::shared_ptr<GoalHandleMove>& goal_handle) {
          handleAccepted(context, goal_handle);
        });
  }

  rclcpp_action::GoalResponse handleGoal(MotorContext& context,
                                         const MoveToPosition::Goal& goal) {
    if (homing_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting move while homing");
      return rclcpp_action::GoalResponse::REJECT;
    }
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

  void handleAccepted(MotorContext& context,
                      const std::shared_ptr<GoalHandleMove>& goal_handle) {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    if (homing_.load()) {
      auto result = std::make_shared<MoveToPosition::Result>();
      result->success = false;
      result->message = "homing is active";
      goal_handle->abort(result);
      return;
    }
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
  int32_t clampTarget(const MotorConfig& config, int64_t target) {
    const int64_t clamped =
        std::clamp<int64_t>(target, config.min_position, config.max_position);
    if (clamped != target) {
      RCLCPP_WARN(
          get_logger(), "%s: target %ld clamped to %ld (limits [%d, %d])",
          config.name.c_str(), static_cast<long>(target),
          static_cast<long>(clamped), config.min_position, config.max_position);
    }
    return static_cast<int32_t>(clamped);
  }

  void startSimpleMove(MotorContext& context, int32_t position, bool relative) {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    if (homing_.load()) {
      RCLCPP_WARN(get_logger(), "Ignoring position command while homing");
      return;
    }
    if (!roboclaw_) {
      return;
    }
    const MotorConfig& config = context.config;
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
      RCLCPP_INFO(get_logger(), "%s: cmd_position%s -> %d", config.name.c_str(),
                  relative ? "_relative" : "", clamped);
    } catch (const RoboClawError& e) {
      RCLCPP_ERROR(get_logger(), "%s: cmd_position failed: %s",
                   config.name.c_str(), e.what());
    }
  }

  void executeGoal(MotorContext& context,
                   const std::shared_ptr<GoalHandleMove>& goal_handle) {
    const MotorConfig& config = context.config;
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
    } catch (const RoboClawError& e) {
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
        } catch (const RoboClawError& e) {
          RCLCPP_ERROR(get_logger(), "%s: stop failed: %s", config.name.c_str(),
                       e.what());
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
      } catch (const RoboClawError& e) {
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
          ((target < encoder && limit_switches_->isMinTriggered(config.axis)) ||
           (target > encoder && limit_switches_->isMaxTriggered(config.axis)));
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

  void homeMotors(std_srvs::srv::Trigger::Response& response) {
    response.success = false;
    {
      std::lock_guard<std::mutex> motion_lock(motion_mutex_);
      for (MotorContext* context : {&rotate_, &extend_}) {
        std::lock_guard<std::mutex> goal_lock(context->goal_mutex);
        if (context->active_goal && context->active_goal->is_active()) {
          response.message = "finish or cancel active move goals before homing";
          RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
          return;
        }
      }
      cancel_home_.store(false);
      homing_.store(true);
    }
    try {
      validateHomingParameters();
      if (!imu_->isAvailable() || !tof_->isAvailable()) {
        throw std::runtime_error(
            "homing requires available IMU and ToF sensors");
      }
      // Check both streams before moving either axis.
      homingReading(false);
      homingReading(true);
      roboclaw_->stopAll();
      RCLCPP_INFO(get_logger(), "Homing extension, then rotation");
      homeAxis(extend_, false);
      homeAxis(rotate_, true);
      if (std::abs(homingReading(false).value -
                   get_parameter("homing.extension_meters").as_double()) >
              get_parameter("homing.extension_tolerance_meters").as_double() ||
          std::abs(homingReading(true).value) >
              get_parameter("homing.rotation_tolerance_degrees").as_double()) {
        throw std::runtime_error(
            "home reference drifted while homing other axis");
      }
      response.success = true;
      response.message =
          "homed: extension and rotation at configured references; "
          "both encoders zero";
      RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    } catch (const std::exception& e) {
      response.message = e.what();
      RCLCPP_ERROR(get_logger(), "Homing failed: %s", e.what());
    }
    for (MotorContext* context : {&rotate_, &extend_}) {
      try {
        roboclaw_->stop(context->config.motor);
      } catch (const RoboClawError& e) {
        response.success = false;
        response.message += std::string("; stop failed: ") + e.what();
        RCLCPP_ERROR(get_logger(), "Homing stop failed: %s", e.what());
      }
      context->move_active.store(false);
    }
    homing_.store(false);
  }

  void validateHomingParameters() {
    for (const std::string name :
         {"extension_tolerance_meters", "rotation_tolerance_degrees",
          "extension_approach_meters", "rotation_approach_degrees",
          "timeout_seconds", "sensor_timeout_seconds"}) {
      const double value = get_parameter("homing." + name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error("homing." + name +
                                 " must be finite and positive");
      }
    }
    averageHomingReading(
        0.0, 0.0, get_parameter("homing.smoothing_alpha").as_double(), false);
    if (get_parameter("homing.extension_approach_meters").as_double() <=
            get_parameter("homing.extension_tolerance_meters").as_double() ||
        get_parameter("homing.rotation_approach_degrees").as_double() <=
            get_parameter("homing.rotation_tolerance_degrees").as_double()) {
      throw std::runtime_error(
          "homing approach distances must exceed tolerances");
    }
    const double target = get_parameter("homing.extension_meters").as_double();
    if (!std::isfinite(target) || target <= 0.0 ||
        get_parameter("homing.settle_samples").as_int() < 2) {
      throw std::runtime_error(
          "invalid homing extension target or settle_samples");
    }
    gravityAngleDegrees(get_parameter("homing.gravity.x").as_double(),
                        get_parameter("homing.gravity.y").as_double(),
                        get_parameter("homing.gravity.z").as_double(),
                        get_parameter("homing.gravity.x").as_double(),
                        get_parameter("homing.gravity.y").as_double(),
                        get_parameter("homing.gravity.z").as_double());
    for (const std::string axis : {"rotate", "extend"}) {
      const auto direction =
          get_parameter("homing." + axis + "_direction").as_int();
      const auto speed = get_parameter("homing." + axis + "_speed").as_int();
      const auto min_speed =
          get_parameter("homing." + axis + "_min_speed").as_int();
      const auto travel =
          get_parameter("homing." + axis + "_max_travel_counts").as_int();
      const auto qpps =
          axis == "rotate" ? rotate_.config.qpps : extend_.config.qpps;
      if (std::abs(direction) != 1 || speed <= 0 || speed > qpps ||
          min_speed <= 0 || min_speed > speed || travel <= 0 ||
          travel > std::numeric_limits<int32_t>::max()) {
        throw std::runtime_error(
            "invalid homing direction, maximum/minimum speed or travel for " +
            axis);
      }
    }
  }

  struct HomingReading {
    double value;
    double raw_value;
    std::chrono::steady_clock::time_point stamp;
  };

  // Called under sensor_mutex_. Only fresh, valid physical samples advance
  // the filter; a gap or invalid measurement discards its history.
  void updateHomingReading(std::optional<HomingReading>& reading, double value,
                           bool rotation) {
    try {
      const auto stamp = std::chrono::steady_clock::now();
      const double alpha = get_parameter("homing.smoothing_alpha").as_double();
      const double max_age =
          get_parameter("homing.sensor_timeout_seconds").as_double();
      const bool continuous =
          reading &&
          std::chrono::duration<double>(stamp - reading->stamp).count() <=
              max_age;
      const double previous = continuous ? reading->value : value;
      reading = HomingReading{
          averageHomingReading(previous, value, alpha, rotation), value, stamp};
    } catch (const std::runtime_error& e) {
      reading.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Invalid homing sensor filter: %s", e.what());
    }
  }

  HomingReading homingReading(bool rotation) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    const auto& reading = rotation ? rotation_reading_ : extension_reading_;
    const double max_age =
        get_parameter("homing.sensor_timeout_seconds").as_double();
    if (!reading || !std::isfinite(reading->value) ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      reading->stamp)
                .count() > max_age) {
      throw std::runtime_error(rotation ? "invalid or stale gravity"
                                        : "invalid or stale extension length");
    }
    return *reading;
  }

  void homeAxis(MotorContext& context, bool rotation) {
    const std::string axis = rotation ? "rotate" : "extend";
    const auto motor = context.config.motor;
    const int speed = get_parameter("homing." + axis + "_speed").as_int();
    const int min_speed =
        get_parameter("homing." + axis + "_min_speed").as_int();
    const int direction =
        get_parameter("homing." + axis + "_direction").as_int();
    const auto travel_limit =
        get_parameter("homing." + axis + "_max_travel_counts").as_int();
    const double target =
        rotation ? 0.0 : get_parameter("homing.extension_meters").as_double();
    const double tolerance =
        get_parameter(rotation ? "homing.rotation_tolerance_degrees"
                               : "homing.extension_tolerance_meters")
            .as_double();
    const double approach_distance =
        get_parameter(rotation ? "homing.rotation_approach_degrees"
                               : "homing.extension_approach_meters")
            .as_double();
    const auto started = std::chrono::steady_clock::now();
    const double timeout = get_parameter("homing.timeout_seconds").as_double();
    int64_t previous_encoder = roboclaw_->readEncoder(motor).value;
    int64_t travel = 0;
    int settled = 0;
    double stopped_error_sum = 0.0;
    bool stopped = false;
    // An axis already inside tolerance should not move just to enter the
    // narrower stop band used during an approach.
    bool settling = true;
    auto previous_sample = std::chrono::steady_clock::time_point{};
    context.move_active.store(true);
    while (rclcpp::ok() && !cancel_home_.load()) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        started)
              .count() > timeout) {
        throw std::runtime_error(axis + " homing timed out");
      }
      const auto reading = homingReading(rotation);
      homingReading(!rotation);
      const int64_t encoder = roboclaw_->readEncoder(motor).value;
      travel += std::abs(encoder - previous_encoder);
      previous_encoder = encoder;
      if (travel >= travel_limit) {
        throw std::runtime_error(
            axis + " homing travel limit exceeded (" + std::to_string(travel) +
            "/" + std::to_string(travel_limit) + " counts; filtered reading " +
            std::to_string(reading.value) + (rotation ? " degrees)" : " m)"));
      }
      const auto currents = roboclaw_->readCurrents();
      if (currents.first > rotate_.config.max_current ||
          currents.second > extend_.config.max_current) {
        throw std::runtime_error("over-current while homing");
      }
      if ((roboclaw_->readErrorStatus() & 0xffffu) != 0) {
        throw std::runtime_error("RoboClaw fault while homing");
      }
      const int command =
          homingSpeed(target - reading.value, tolerance, approach_distance,
                      speed, min_speed, direction, settling);
      if (limit_switches_->isAvailable() &&
          ((command < 0 &&
            limit_switches_->isMinTriggered(context.config.axis)) ||
           (command > 0 &&
            limit_switches_->isMaxTriggered(context.config.axis)))) {
        throw std::runtime_error(axis + " homing limit switch triggered");
      }
      if (command == 0) {
        roboclaw_->stop(motor);
        if (stopped && reading.stamp != previous_sample) {
          if (roboclaw_->readSpeed(motor) == 0) {
            ++settled;
            stopped_error_sum +=
                rotation ? std::remainder(target - reading.raw_value, 360.0)
                         : target - reading.raw_value;
          } else {
            settled = 0;
            stopped_error_sum = 0.0;
          }
        }
        stopped = true;
        // Check a stationary average too: the moving EMA can lag behind
        // the mechanism and otherwise accept an overshoot as home.
        if (settled >= get_parameter("homing.settle_samples").as_int() &&
            std::abs(stopped_error_sum / settled) <= tolerance) {
          if (cancel_home_.load() || !rclcpp::ok()) {
            throw std::runtime_error("homing canceled or node shutting down");
          }
          if (rotation) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            if (!latest_imu_sample_) {
              throw std::runtime_error("missing IMU orientation at home");
            }
            captureImuZero(*latest_imu_sample_);
          }
          roboclaw_->setEncoder(motor, 0);
          if (roboclaw_->readEncoder(motor).value != 0) {
            throw std::runtime_error(axis +
                                     " encoder reset verification failed");
          }
          context.last_encoder.store(0);
          context.target_position.store(0);
          context.move_active.store(false);
          RCLCPP_INFO(get_logger(), "%s homed; encoder zeroed", axis.c_str());
          return;
        }
      } else {
        settled = 0;
        stopped_error_sum = 0.0;
        stopped = false;
        roboclaw_->driveSpeed(motor, command);
      }
      previous_sample = reading.stamp;
      std::this_thread::sleep_for(50ms);
    }
    throw std::runtime_error("homing canceled or node shutting down");
  }

  void resetEncoders(std_srvs::srv::Trigger::Response& response) {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    if (homing_.load()) {
      response.success = false;
      response.message = "cannot reset encoders while homing";
      RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
      return;
    }
    try {
      roboclaw_->resetEncoders();
      response.success = true;
      response.message = "encoders reset to zero";
      RCLCPP_INFO(get_logger(), "Encoders reset to zero");
    } catch (const RoboClawError& e) {
      response.success = false;
      response.message = e.what();
    }
  }

  // ----- Periodic status
  // -------------------------------------------------------

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
    } catch (const RoboClawError& e) {
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

  void publishMotorStatus(MotorContext& context, const rclcpp::Time& stamp,
                          float current, float temperature, float main_battery,
                          uint32_t error_status) {
    const MotorConfig& config = context.config;
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
    } catch (const RoboClawError& e) {
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
  // Publishes raw IMU/ToF readings as well as derived scalar values.
  void publishDerivedState() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    std_msgs::msg::Float32 angle;
    angle.data = std::numeric_limits<float>::quiet_NaN();
    if (imu_->isAvailable()) {
      if (auto sample = imu_->readSample()) {
        latest_imu_sample_ = *sample;
        try {
          updateHomingReading(
              rotation_reading_,
              gravityAngleDegrees(
                  sample->gravity_x, sample->gravity_y, sample->gravity_z,
                  get_parameter("homing.gravity.x").as_double(),
                  get_parameter("homing.gravity.y").as_double(),
                  get_parameter("homing.gravity.z").as_double()),
              true);
        } catch (const std::runtime_error& e) {
          rotation_reading_.reset();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Homing gravity invalid: %s", e.what());
        }
        if (!imu_zero_orientation_set_) {
          captureImuZero(*sample);
        }

        sensor_msgs::msg::Imu imu_message;
        imu_message.header.stamp = now();
        imu_message.header.frame_id = "extender_imu";
        imu_message.orientation.x = sample->orientation_x;
        imu_message.orientation.y = sample->orientation_y;
        imu_message.orientation.z = sample->orientation_z;
        imu_message.orientation.w = sample->orientation_w;
        imu_message.angular_velocity_covariance[0] = -1.0;
        imu_message.linear_acceleration_covariance[0] = -1.0;
        imu_pub_->publish(imu_message);

        geometry_msgs::msg::Vector3Stamped gravity_message;
        gravity_message.header = imu_message.header;
        gravity_message.vector.x = sample->gravity_x;
        gravity_message.vector.y = sample->gravity_y;
        gravity_message.vector.z = sample->gravity_z;
        gravity_pub_->publish(gravity_message);

        if (auto degrees = rotationAngleDegrees(*sample)) {
          angle.data = static_cast<float>(*degrees);
        }
      } else if (!imu_->lastError().empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "BNO055 read failed: %s",
                             imu_->lastError().c_str());
      }
    } else if (rotate_.config.counts_per_unit != 0.0) {
      angle.data = static_cast<float>(rotate_.last_encoder.load() /
                                      rotate_.config.counts_per_unit);
    }
    rotation_angle_pub_->publish(angle);

    std_msgs::msg::Float32 length;
    length.data = std::numeric_limits<float>::quiet_NaN();
    if (tof_->isAvailable()) {
      if (auto sample = tof_->readSample()) {
        sensor_msgs::msg::Range range_message;
        range_message.header.stamp = now();
        range_message.header.frame_id = "extender_tof";
        range_message.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_message.field_of_view = 0.314;
        range_message.min_range = 0.01;
        range_message.max_range = 1.3;
        range_message.range = static_cast<float>(sample->distance_meters);
        tof_range_pub_->publish(range_message);

        std_msgs::msg::UInt8 range_status;
        range_status.data = sample->range_status;
        tof_status_pub_->publish(range_status);
        if (sample->range_status == 0) {
          const double meters = extensionLength(
              sample->distance_meters,
              get_parameter("tof.beam_angle_degrees").as_double(),
              get_parameter("tof.mount_offset_meters").as_double());
          length.data = static_cast<float>(meters);
          updateHomingReading(extension_reading_, meters, false);
        } else {
          extension_reading_.reset();
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "VL53L4CD raw range %.3f m has quality status %u",
              sample->distance_meters,
              static_cast<unsigned int>(sample->range_status));
        }
      } else if (!tof_->lastError().empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "VL53L4CD read failed: %s",
                             tof_->lastError().c_str());
      }
    } else if (extend_.config.counts_per_unit != 0.0) {
      length.data = static_cast<float>(
          get_parameter("homing.extension_meters").as_double() +
          extend_.last_encoder.load() / extend_.config.counts_per_unit);
    }
    extension_length_pub_->publish(length);
  }

  // ----- Members
  // ---------------------------------------------------------------

  std::unique_ptr<RoboClaw> roboclaw_;
  std::unique_ptr<Bno055> imu_;
  std::unique_ptr<Vl53l4cd> tof_;
  std::unique_ptr<LimitSwitches> limit_switches_;

  MotorContext rotate_;
  MotorContext extend_;
  std::mutex motion_mutex_;
  std::mutex sensor_mutex_;
  std::atomic<bool> homing_{false};
  std::atomic<bool> cancel_home_{false};
  std::optional<HomingReading> rotation_reading_;
  std::optional<HomingReading> extension_reading_;
  std::optional<Bno055::Sample> latest_imu_sample_;

  std::array<double, 4> imu_zero_orientation_{0.0, 0.0, 0.0, 1.0};
  bool imu_zero_orientation_set_ = false;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_range_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr tof_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rotation_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr extension_length_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_home_service_;
  rclcpp::CallbackGroup::SharedPtr home_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_encoders_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace titania_extender

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<titania_extender::TitaniaExtenderNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(node);
    executor.spin();
  } catch (const titania_extender::RoboClawError& e) {
    RCLCPP_FATAL(rclcpp::get_logger("titania_extender_node"),
                 "RoboClaw error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
