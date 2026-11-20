#include "pure_pursuit.hpp"
#include "launch_ramp.hpp"

#include <cmath>

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


namespace {
inline double planar_distance_sq(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return dx * dx + dy * dy;
}

// 런타임(ros2 param set)에 변경 가능한 파라미터 목록.
// apply_runtime_parameter()의 분기와 1:1로 대응합니다.
constexpr const char *kRuntimeParameterNames[] = {
    "drive_topic",
    "test_mode",
    "drive_test_topic",
    "rviz_runtime_params_x",
    "rviz_runtime_params_y",
    "rviz_runtime_params_z",
    "rf_speed_scale_channel",
    "rf_max_limit_channel",
    "rf_enable_channel",
    "rf_enable_threshold",
    "rf_value_min",
    "rf_value_max",
    "K_p",
    "K_d",
    "K_i",
    "heading_error_gain",
    "path_tangent_gain",
    "pose_preview_s",
    "understeer_rad_per_mps2",
    "velocity_percentage",
    "max_speed_limit_percentage",
    "min_lookahead",
    "max_lookahead",
    "lookahead_ratio",
    "speed_profile_distance_offset",
    "steering_limit",
    "steering_expo_gain",
    "steering_expo_curve",
    "steer_reduction_speed_threshold",
    "steer_reduction_constant_coef",
    "steer_reduction_linear_coef",
    "steer_reduction_min_scale",
    "speed_reduction_steer_angle_deg",
    "max_allowed_steer_drop_deg",
    "speed_reduction_adjust",
    "speed_reduction_prev_scale",
    "drive_output_rate_hz",
    "publish_drive_on_odom",
    "visualization_rate_hz",
    "steer_latest_blend",
    "steer_large_change_blend",
    "steer_blend_change_threshold_deg",
    "steer_speed_filter_start_speed",
    "steer_speed_filter_end_speed",
    "steer_speed_filter_final_blend",
    "speed_latest_blend",
    "min_searching_idx_offset",
    "max_searching_idx_offset",
    "slow_with_obs",
    "obs_slow_th",
    "obs_slow_percentage",
    "speed_to_erpm_gain",
    "speed_to_erpm_offset",
    "wheel_speed_deadband",
    "wheel_speed_scale",
    "wheel_speed_timeout",
    "launch_start_enabled",
    "launch_start_channel",
    "launch_start_channel_threshold",
    "launch_start_channel_2",
    "launch_start_channel_2_threshold",
    "launch_start_engage_diff",
    "launch_start_release_diff",
    "launch_start_accel",
    "speed_hold_default_duration",
    "speed_hold_max_duration",
    "speed_hold_max_speed",
};

// 런타임 변경 시 허용 범위.
// 여기에 없는 숫자 파라미터는 유한값(NaN/inf 불가) 여부만 검사합니다.
// 사용 지점에서 이미 clamp하는 값(각종 blend, min_scale 등)은 등록하지 않습니다.
constexpr double kUnbounded = std::numeric_limits<double>::infinity();

struct RuntimeParameterBound {
  const char *name;
  double min_value;
  double max_value;
};

constexpr RuntimeParameterBound kRuntimeParameterBounds[] = {
    {"velocity_percentage", 0.0, 1.0},
    {"max_speed_limit_percentage", 0.0, 1.0},
    // 음수 게인은 제어 방향이 뒤집혀 즉시 발산합니다.
    {"K_p", 0.0, kUnbounded},
    {"K_i", 0.0, kUnbounded},
    {"K_d", 0.0, kUnbounded},
    {"path_tangent_gain", 0.0, kUnbounded},
    {"pose_preview_s", 0.0, 0.35},
    {"understeer_rad_per_mps2", 0.0, kUnbounded},
    // 0 이하이면 std::clamp(x, -lim, lim)의 lo > hi가 되어 정의되지 않은 동작
    {"steering_limit", 1e-3, 90.0},
    {"min_lookahead", 0.0, kUnbounded},
    {"max_lookahead", 0.0, kUnbounded},
    // lookahead 계산에서 나눗셈 분모로 사용됩니다.
    {"lookahead_ratio", 1e-6, kUnbounded},
    {"steering_expo_curve", 1e-6, kUnbounded},
    {"drive_output_rate_hz", 0.0, kUnbounded},
    {"visualization_rate_hz", 0.0, kUnbounded},
    {"obs_slow_th", 0.0, kUnbounded},
    {"obs_slow_percentage", 0.0, 1.0},
    {"rf_speed_scale_channel", 0.0, kUnbounded},
    {"rf_max_limit_channel", 0.0, kUnbounded},
    {"rf_enable_channel", 0.0, kUnbounded},
    {"launch_start_channel", 0.0, kUnbounded},
    {"launch_start_channel_threshold", 0.0, kUnbounded},
    // 음수는 보조 트리거 끄기이므로 하한을 두지 않습니다.
    {"launch_start_channel_2_threshold", 0.0, kUnbounded},
    {"wheel_speed_deadband", 0.0, kUnbounded},
    // 0이면 휠 속도가 항상 0으로 보여 래치가 계속 걸립니다.
    {"wheel_speed_scale", 1e-6, kUnbounded},
    {"wheel_speed_timeout", 0.0, kUnbounded},
    {"launch_start_engage_diff", 0.0, kUnbounded},
    {"launch_start_release_diff", 0.0, kUnbounded},
    // 0이면 램프가 움직이지 않아 래치가 풀리지 않습니다.
    {"launch_start_accel", 1e-3, kUnbounded},
    {"speed_hold_default_duration", 0.0, 30.0},
    // 상한 자체가 무한이면 상한이 아닙니다. 0은 speed hold 기능 끄기.
    {"speed_hold_max_duration", 0.0, 30.0},
    {"speed_hold_max_speed", 0.0, 20.0},
};
} // namespace

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
  // 파라미터 선언
  this->declare_parameter("odom_topic", "/ego_racecar/odom");
  this->declare_parameter("car_refFrame", "ego_racecar/base_link");
  this->declare_parameter("drive_topic", "/drive");
  this->declare_parameter("test_mode", false);
  this->declare_parameter("drive_test_topic", "/drive_test");
  this->declare_parameter("path_topic", "/Path");
  this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
  this->declare_parameter("rviz_lookahead_waypoint_topic",
                          "/lookahead_waypoint");
  this->declare_parameter("rviz_speed_offset_waypoint_topic",
                          "/speed_offset_waypoint");
  this->declare_parameter("rviz_runtime_params_topic", "/pp_runtime_params");
  this->declare_parameter("rviz_runtime_params_x", 0.0);
  this->declare_parameter("rviz_runtime_params_y", 0.0);
  this->declare_parameter("rviz_runtime_params_z", 1.2);
  this->declare_parameter("rf_topic", "/rf");
  this->declare_parameter("rf_speed_scale_channel", 6);
  this->declare_parameter("rf_max_limit_channel", 7);
  this->declare_parameter("rf_enable_channel", 8);
  this->declare_parameter("rf_enable_threshold", 1500);
  this->declare_parameter("rf_value_min", 1000);
  this->declare_parameter("rf_value_max", 2000);
  // VESC sensors/core 기반 휠 속도
  this->declare_parameter("vesc_state_topic", "sensors/core");
  this->declare_parameter("speed_to_erpm_gain", 3172.47);
  this->declare_parameter("speed_to_erpm_offset", 0.0);
  this->declare_parameter("wheel_speed_deadband", 0.05);
  this->declare_parameter("wheel_speed_scale", 2.6);
  this->declare_parameter("wheel_speed_timeout", 0.5);
  // 런치 스타트
  this->declare_parameter("launch_start_reset_topic", "/launch_start_reset");
  this->declare_parameter("launch_start_enabled", true);
  this->declare_parameter("launch_start_channel", 5);
  this->declare_parameter("launch_start_channel_threshold", 1800);
  this->declare_parameter("launch_start_channel_2", 4);
  this->declare_parameter("launch_start_channel_2_threshold", 1900);
  this->declare_parameter("launch_start_engage_diff", 1.0);
  this->declare_parameter("launch_start_release_diff", 0.5);
  this->declare_parameter("launch_start_accel", 3.0);
  // 외부 요청 고정 속도 유지
  this->declare_parameter("speed_hold_topic", "/launch_speed_hold");
  this->declare_parameter("speed_hold_default_duration", 1.0);
  this->declare_parameter("speed_hold_max_duration", 10.0);
  this->declare_parameter("speed_hold_max_speed", 8.0);
  this->declare_parameter("global_refFrame", "map");
  this->declare_parameter("path_is_circular", true);
  this->declare_parameter("min_lookahead", 0.5);
  this->declare_parameter("max_lookahead", 1.0);
  this->declare_parameter("lookahead_ratio", 8.0);
  this->declare_parameter("speed_profile_distance_offset", 0.0);
  this->declare_parameter("min_searching_idx_offset", 10);
  this->declare_parameter("max_searching_idx_offset", 40);
  this->declare_parameter("K_p", 0.5);
  this->declare_parameter("K_d", 0.1);  // 미분 게인
  this->declare_parameter("K_i", 0.05); // 추가된 적분 게인
  this->declare_parameter("steering_limit", 25.0);
  this->declare_parameter("velocity_percentage", 0.85);
  this->declare_parameter("max_speed_limit_percentage", 1.0);
  this->declare_parameter("heading_error_gain", 0.0);
  this->declare_parameter("path_tangent_gain", 0.0);
  this->declare_parameter("pose_preview_s", 0.0);
  this->declare_parameter("understeer_rad_per_mps2", 0.0);
  this->declare_parameter("steer_reduction_speed_threshold", 5.0);
  this->declare_parameter("steer_reduction_constant_coef", 0.85);
  this->declare_parameter("steer_reduction_linear_coef", 0.03);
  this->declare_parameter("steer_reduction_min_scale", 0.3);
  this->declare_parameter("speed_reduction_steer_angle_deg", 12.0);
  this->declare_parameter("max_allowed_steer_drop_deg", 5.0);
  this->declare_parameter("speed_reduction_adjust", 0.0);
  this->declare_parameter("speed_reduction_prev_scale", 0.0);
  this->declare_parameter("steering_expo_gain", 0.0);
  this->declare_parameter("steering_expo_curve", 2.0);
  this->declare_parameter("drive_output_rate_hz", 50.0);
  this->declare_parameter("publish_drive_on_odom", true);
  this->declare_parameter("visualization_rate_hz", 20.0);
  this->declare_parameter("steer_latest_blend", 0.10);
  this->declare_parameter("steer_large_change_blend", 0.55);
  this->declare_parameter("steer_blend_change_threshold_deg", 10.0);
  this->declare_parameter("steer_speed_filter_start_speed", 0.0);
  this->declare_parameter("steer_speed_filter_end_speed", 0.0);
  this->declare_parameter("steer_speed_filter_final_blend", 0.10);
  this->declare_parameter("speed_latest_blend", 0.90);
  this->declare_parameter("slow_with_obs", true);
  this->declare_parameter("obs_slow_th", 3.0);
  this->declare_parameter("obs_slow_percentage", 0.6);

  // 초기화 전용 파라미터 읽어오기 (런타임 변경 불가)
  odom_topic = this->get_parameter("odom_topic").as_string();
  car_refFrame = this->get_parameter("car_refFrame").as_string();
  path_topic = this->get_parameter("path_topic").as_string();
  rviz_current_waypoint_topic =
      this->get_parameter("rviz_current_waypoint_topic").as_string();
  rviz_lookahead_waypoint_topic =
      this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
  rviz_speed_offset_waypoint_topic =
      this->get_parameter("rviz_speed_offset_waypoint_topic").as_string();
  rviz_runtime_params_topic =
      this->get_parameter("rviz_runtime_params_topic").as_string();
  rf_topic = this->get_parameter("rf_topic").as_string();
  vesc_state_topic = this->get_parameter("vesc_state_topic").as_string();
  launch_start_reset_topic =
      this->get_parameter("launch_start_reset_topic").as_string();
  speed_hold_topic = this->get_parameter("speed_hold_topic").as_string();
  global_refFrame = this->get_parameter("global_refFrame").as_string();
  path_is_circular = this->get_parameter("path_is_circular").as_bool();

  // 런타임 변경 가능 파라미터는 apply_runtime_parameter()로 일괄 반영
  for (const char *name : kRuntimeParameterNames) {
    apply_runtime_parameter(this->get_parameter(name));
  }

  // 초기 적분 오차 초기화
  integral_error = 0.0;
  x_car_world = 0.0;
  y_car_world = 0.0;
  car_heading = 0.0;
  previous_speed_reduction = 0.0;
  target_steer = 0.0;
  target_speed = 0.0;
  target_speed_launch_gain_ = 1.0;
  target_speed_scale_dirty_ = false;
  output_steer = 0.0;
  output_speed = 0.0;
  current_lookahead_distance = 0.0;
  observed_path_max_speed = 0.0;
  rf_runtime_control_active = false;
  wheel_speed_measured_ = 0.0;
  wheel_speed_valid_ = false;
  launch_start_active_ = false;
  launch_start_handoff_active_ = false;
  launch_start_pending_ = false;
  launch_start_pending_source_.clear();
  launch_start_ramp_speed_ = 0.0;
  launch_start_time_valid_ = false;
  launch_start_elapsed_ = 0.0;
  launch_start_prev_raw_ = 0;
  launch_start_prev_raw_valid_ = false;
  speed_hold_active_ = false;
  speed_hold_speed_ = 0.0;
  speed_hold_duration_ = 0.0;
  speed_hold_elapsed_ = 0.0;
  speed_hold_time_valid_ = false;
  wheel_speed_reported_alive_ = false;
  has_target_command_ = false;
  output_command_initialized_ = false;
  num_waypoints = 0;

  // 경로 수신 플래그 초기화
  path_received_ = false;

  // Subscriber, Publisher, Timer 등 초기화
  rclcpp::QoS odom_qos(rclcpp::KeepLast(1));
  odom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, odom_qos, std::bind(&PurePursuit::odom_callback, this, _1));

  subscription_odom_obs = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/static_obstacle", 10, std::bind(&PurePursuit::obs_odom_callback, this, _1));
  obs_status = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/obj_flag", 10, std::bind(&PurePursuit::obs_status_callback, this, _1));

  rclcpp::QoS pathQos(rclcpp::KeepLast(10));
  pathQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  //pathQos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  subscription_path = this->create_subscription<nav_msgs::msg::Path>(
      path_topic, pathQos,
      std::bind(&PurePursuit::path_callback, this, std::placeholders::_1));

  subscription_rf = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      rf_topic, 10, std::bind(&PurePursuit::rf_callback, this, _1));

  // VESC telemetry는 최신값만 필요하므로 얕은 큐 + BestEffort
  rclcpp::QoS vesc_qos(rclcpp::KeepLast(1));
  vesc_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  subscription_vesc_state =
      this->create_subscription<vesc_msgs::msg::VescStateStamped>(
          vesc_state_topic, vesc_qos,
          std::bind(&PurePursuit::vesc_state_callback, this, _1));

  subscription_launch_start_reset =
      this->create_subscription<std_msgs::msg::Bool>(
          launch_start_reset_topic, 10,
          std::bind(&PurePursuit::launch_start_reset_callback, this, _1));

  subscription_speed_hold =
      this->create_subscription<std_msgs::msg::Float64MultiArray>(
          speed_hold_topic, 10,
          std::bind(&PurePursuit::speed_hold_callback, this, _1));

  configure_drive_publisher();
  configure_drive_output_timer();
  vis_current_point_pub =
      this->create_publisher<visualization_msgs::msg::Marker>(
          rviz_current_waypoint_topic, 10);
  vis_lookahead_point_pub =
      this->create_publisher<visualization_msgs::msg::Marker>(
          rviz_lookahead_waypoint_topic, 10);
  vis_speed_point_pub =
      this->create_publisher<visualization_msgs::msg::Marker>(
          rviz_speed_offset_waypoint_topic, 10);
  vis_runtime_params_pub =
      this->create_publisher<visualization_msgs::msg::Marker>(
          rviz_runtime_params_topic, 10);

  runtime_param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // 검증: 숫자 파라미터는 유한값이어야 하고,
        // kRuntimeParameterBounds에 등록된 항목은 그 범위 안이어야 합니다.
        // set_parameters_atomically로 들어온 묶음은 하나만 걸려도 전체가 거부됩니다.
        // (일반 set_parameters는 rclcpp가 파라미터별로 호출하므로 개별 판정)
        for (const auto &parameter : parameters) {
          std::string reason;
          if (!validate_runtime_parameter(parameter, &reason)) {
            result.successful = false;
            result.reason = reason;
            RCLCPP_WARN(this->get_logger(), "Rejected parameter update: %s",
                        reason.c_str());
            return result;
          }
        }

        // 반영: 런타임 변경 가능한 파라미터를 멤버 변수에 적용
        bool drive_output_config_changed = false;
        for (const auto &parameter : parameters) {
          if (!apply_runtime_parameter(parameter)) {
            continue;
          }

          const std::string &name = parameter.get_name();
          if (name == "drive_topic" || name == "test_mode" ||
              name == "drive_test_topic" || name == "drive_output_rate_hz") {
            drive_output_config_changed = true;
          }
        }

        if (drive_output_config_changed) {
          configure_drive_publisher();
          configure_drive_output_timer();
        }

        return result;
      });

  runtime_status_timer_ = this->create_wall_timer(
      200ms, std::bind(&PurePursuit::runtime_status_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

  // RF 실시간 조절 설정 요약 및 임계값 검증
  {
    const int rf_raw_min = std::min(rf_value_min, rf_value_max);
    const int rf_raw_max = std::max(rf_value_min, rf_value_max);
    RCLCPP_INFO(this->get_logger(),
                "RF runtime control: speed ch[%d], max limit ch[%d], "
                "enable ch[%d] < %d, raw range [%d, %d]",
                rf_speed_scale_channel, rf_max_limit_channel, rf_enable_channel,
                rf_enable_threshold, rf_raw_min, rf_raw_max);
    RCLCPP_INFO(this->get_logger(),
                "Launch start: %s, trigger ch[%d] rising past %d, "
                "trigger2 ch[%d] rising past %d, "
                "engage >= %.2f m/s, release <= %.2f m/s (both x speed gain), "
                "accel %.2f m/s^2",
                launch_start_enabled ? "enabled" : "disabled",
                launch_start_channel, launch_start_channel_threshold,
                launch_start_channel_2, launch_start_channel_2_threshold,
                launch_start_engage_diff, launch_start_release_diff,
                launch_start_accel);
    RCLCPP_INFO(this->get_logger(),
                "Wheel speed: %s, erpm gain %.2f, offset %.2f, "
                "command-domain scale %.2f",
                vesc_topic_display_name(), speed_to_erpm_gain,
                speed_to_erpm_offset, wheel_speed_scale);
    RCLCPP_INFO(this->get_logger(), "Launch start reset topic: %s",
                subscription_launch_start_reset
                    ? subscription_launch_start_reset->get_topic_name()
                    : launch_start_reset_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Speed hold topic: %s (std_msgs/Float64MultiArray "
                "[speed, duration]), default %.2fs, max %.2fs, "
                "max speed %.2f m/s",
                subscription_speed_hold
                    ? subscription_speed_hold->get_topic_name()
                    : speed_hold_topic.c_str(),
                speed_hold_default_duration, speed_hold_max_duration,
                speed_hold_max_speed);
    if (launch_start_channel == rf_speed_scale_channel ||
        launch_start_channel == rf_max_limit_channel ||
        launch_start_channel == rf_enable_channel) {
      RCLCPP_WARN(this->get_logger(),
                  "launch_start_channel(%d) is already used by another RF "
                  "channel: any rising edge past %d on it also fires launch "
                  "start",
                  launch_start_channel, launch_start_channel_threshold);
    }
    if (launch_start_channel_2 >= 0 &&
        (launch_start_channel_2 == rf_speed_scale_channel ||
         launch_start_channel_2 == rf_max_limit_channel ||
         launch_start_channel_2 == rf_enable_channel ||
         launch_start_channel_2 == launch_start_channel)) {
      RCLCPP_WARN(this->get_logger(),
                  "launch_start_channel_2(%d) is already used by another RF "
                  "channel: any rising edge past %d on it also fires launch "
                  "start",
                  launch_start_channel_2, launch_start_channel_2_threshold);
    }
    if (launch_start_release_diff >= launch_start_engage_diff) {
      RCLCPP_WARN(this->get_logger(),
                  "launch_start_release_diff(%.2f) >= "
                  "launch_start_engage_diff(%.2f): the ramp enters its "
                  "acceleration-limited handoff as soon as it engages",
                  launch_start_release_diff, launch_start_engage_diff);
    }
    // enable 판정이 "raw < threshold"라 경고 방향도 반대입니다.
    if (rf_enable_threshold <= rf_raw_min) {
      RCLCPP_WARN(this->get_logger(),
                  "rf_enable_threshold(%d) <= raw min(%d): RF runtime control "
                  "never gets enabled",
                  rf_enable_threshold, rf_raw_min);
    } else if (rf_enable_threshold > rf_raw_max) {
      RCLCPP_WARN(this->get_logger(),
                  "rf_enable_threshold(%d) > raw max(%d): RF runtime control "
                  "stays always enabled",
                  rf_enable_threshold, rf_raw_max);
    }
  }

  // PD 제어를 위한 초기값 설정
  prev_error = 0.0;
  prev_time = this->now();
}

double PurePursuit::to_radians(double degrees) {
  return degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
  return radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(const double &x1, const double &x2,
                            const double &y1, const double &y2) const {
  return std::hypot(x2 - x1, y2 - y1);
}

double PurePursuit::adjacent_segment_length(int idx, int next_idx,
                                            int direction) const {
  if (num_waypoints <= 1) {
    return 0.0;
  }

  int segment_idx = direction >= 0 ? idx : next_idx;
  if (path_is_circular) {
    segment_idx = path_idx_limiter(segment_idx);
  }

  if (segment_idx >= 0 &&
      segment_idx < static_cast<int>(waypoints.segment_lengths.size())) {
    return waypoints.segment_lengths[segment_idx];
  }

  if (idx >= 0 && idx < num_waypoints && next_idx >= 0 &&
      next_idx < num_waypoints) {
    return p2pdist(waypoints.X[idx], waypoints.X[next_idx], waypoints.Y[idx],
                   waypoints.Y[next_idx]);
  }
  return 0.0;
}

double PurePursuit::current_max_speed_limit() const {
  if (observed_path_max_speed <= 0.0) {
    return -1.0;
  }

  const double limit_ratio =
      std::clamp(max_speed_limit_percentage, 0.0, 1.0);
  return observed_path_max_speed * limit_ratio;
}

double PurePursuit::apply_max_speed_limit(double speed) const {
  double limited_speed = std::max(0.0, speed);
  const double speed_limit = current_max_speed_limit();
  if (speed_limit >= 0.0) {
    limited_speed = std::min(limited_speed, speed_limit);
  }
  return limited_speed;
}

double PurePursuit::rf_raw_to_unit(int raw_value) const {
  const int raw_min = std::min(rf_value_min, rf_value_max);
  const int raw_max = std::max(rf_value_min, rf_value_max);
  const int raw_range = raw_max - raw_min;
  if (raw_range <= 0) {
    return 0.0;
  }

  const double normalized =
      static_cast<double>(raw_value - raw_min) / static_cast<double>(raw_range);
  return std::clamp(normalized, 0.0, 1.0);
}

bool PurePursuit::read_rf_channel(
    const std_msgs::msg::UInt16MultiArray &rf_msg, int channel_index,
    int *raw_value) const {
  if (channel_index < 0 ||
      channel_index >= static_cast<int>(rf_msg.data.size())) {
    return false;
  }

  if (raw_value) {
    *raw_value = static_cast<int>(rf_msg.data[channel_index]);
  }
  return true;
}

void PurePursuit::set_runtime_percentages(double velocity_scale,
                                          double max_speed_limit_scale) {
  velocity_scale = std::clamp(velocity_scale, 0.0, 1.0);
  max_speed_limit_scale = std::clamp(max_speed_limit_scale, 0.0, 1.0);

  constexpr double kUpdateDeadband = 0.001;
  if (std::abs(velocity_scale - velocity_percentage) < kUpdateDeadband &&
      std::abs(max_speed_limit_scale - max_speed_limit_percentage) <
          kUpdateDeadband) {
    return;
  }

  const auto result = this->set_parameters_atomically(
      {rclcpp::Parameter("velocity_percentage", velocity_scale),
       rclcpp::Parameter("max_speed_limit_percentage", max_speed_limit_scale)});

  if (!result.successful) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Failed to apply RF runtime percentages: %s",
        result.reason.c_str());
    return;
  }

  velocity_percentage = velocity_scale;
  max_speed_limit_percentage = max_speed_limit_scale;
}

std::string PurePursuit::selected_drive_topic() const {
  const std::string selected_topic = test_mode ? drive_test_topic : drive_topic;
  if (!selected_topic.empty()) {
    return selected_topic;
  }
  return test_mode ? "/drive_test" : "/drive";
}

void PurePursuit::configure_drive_publisher() {
  const std::string selected_topic = selected_drive_topic();
  if (publisher_drive && drive_output_topic == selected_topic) {
    return;
  }

  drive_output_topic = selected_topic;
  rclcpp::QoS drive_qos(rclcpp::KeepLast(1));
  drive_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
  drive_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  publisher_drive =
      this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
          drive_output_topic, drive_qos);
  RCLCPP_INFO(this->get_logger(), "Drive command output topic: %s%s",
              drive_output_topic.c_str(), test_mode ? " (test mode)" : "");
}

void PurePursuit::configure_drive_output_timer() {
  const double selected_rate = std::clamp(drive_output_rate_hz, 1.0, 200.0);
  if (drive_output_timer_ &&
      std::abs(active_drive_output_rate_hz - selected_rate) <= 1e-6) {
    return;
  }

  active_drive_output_rate_hz = selected_rate;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / active_drive_output_rate_hz));
  drive_output_timer_ = this->create_wall_timer(
      period, std::bind(&PurePursuit::drive_output_timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Drive command output rate: %.1f Hz",
              active_drive_output_rate_hz);
}

// nav_msgs::Path 토픽을 통해 전달된 경로를 수신하고 내부 waypoints 구조체를
// 갱신합니다. Path 메시지의 각 PoseStamped에서 position.x, position.y,
// position.z를 각각 X, Y, V로 저장합니다.
void PurePursuit::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
  const auto valid_point = [](const geometry_msgs::msg::PoseStamped &pose) {
    const auto &p = pose.pose.position;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  if (!path_msg || path_msg->poses.size() < 2 ||
      !std::all_of(path_msg->poses.begin(), path_msg->poses.end(), valid_point)) {
    path_received_ = has_target_command_ = output_command_initialized_ = false;
    return;
  }

  // The path selector regularly republishes identical geometry with a new stamp.
  // Preserve the filter, nearest-point search and pending 50 Hz command.
  if (path_received_ && path_msg->poses.size() == waypoints.X.size()) {
    bool unchanged = true;
    for (size_t i = 0; i < waypoints.X.size(); ++i) {
      const auto &p = path_msg->poses[i].pose.position;
      if (p.x != waypoints.X[i] || p.y != waypoints.Y[i] || p.z != waypoints.V[i]) {
        unchanged = false;
        break;
      }
    }
    if (unchanged) {
      return;
    }
  }

  // 기존 waypoint 데이터 초기화
  waypoints.X.clear();
  waypoints.Y.clear();
  waypoints.V.clear();
  waypoints.segment_lengths.clear();
  waypoints.path_length = 0.0;
  waypoints.index = 0;
  waypoints.velocity_index = -1;
  waypoints.speed_index = -1;
  has_target_command_ = false;


  double received_path_max_speed = 0.0;

  // 새 경로의 포즈를 순회하면서 좌표와 속도(v)를 저장
  for (const auto &pose_stamped : path_msg->poses) {
    const auto &pos = pose_stamped.pose.position;
    waypoints.X.push_back(pos.x);
    waypoints.Y.push_back(pos.y);
    // Path 메시지에서 z 값은 속도 정보를 담고 있다고 가정
    waypoints.V.push_back(pos.z);
    if (std::isfinite(pos.z)) {
      received_path_max_speed =
          std::max(received_path_max_speed, std::max(0.0, pos.z));
    }
  }

  num_waypoints = static_cast<int>(waypoints.X.size());
  waypoints.segment_lengths.assign(num_waypoints, 0.0);
  if (num_waypoints > 1) {
    const int segment_count =
        path_is_circular ? num_waypoints : num_waypoints - 1;
    for (int i = 0; i < segment_count; ++i) {
      const int next_idx = path_is_circular ? path_idx_limiter(i + 1) : i + 1;
      const double segment_length =
          p2pdist(waypoints.X[i], waypoints.X[next_idx], waypoints.Y[i],
                  waypoints.Y[next_idx]);
      waypoints.segment_lengths[i] = segment_length;
      waypoints.path_length += segment_length;
    }
  }

  if (received_path_max_speed > observed_path_max_speed) {
    observed_path_max_speed = received_path_max_speed;
    const double speed_limit = current_max_speed_limit();
    RCLCPP_INFO(this->get_logger(),
                "Observed path max speed updated: %.2fm/s, speed limit: %.2fm/s",
                observed_path_max_speed, speed_limit);
  }

  RCLCPP_INFO(this->get_logger(),
              "Received new path with %d waypoints from topic.",
              num_waypoints);

  // 경로를 정상적으로 수신했음을 표시
  path_received_ = waypoints.path_length > 1e-6;
  if (!path_received_) {
    output_command_initialized_ = false;
    return;
  }
  // Refresh immediately from a recent localized pose on genuine replans. A
  // new pose will supply the first command if no recent observation exists.
  const double pose_age = (this->now() - last_odom_receipt_).seconds();
  if (last_odom_receipt_.nanoseconds() > 0 && pose_age >= 0.0 && pose_age <= 0.15) {
    refresh_tracking_target();
  } else {
    output_command_initialized_ = false;
  }
}

void PurePursuit::visualize_lookahead_point(Eigen::Vector3d &point) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = global_refFrame;
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0;
  marker.color.r = 1.0;

  marker.pose.position.x = point(0);
  marker.pose.position.y = point(1);
  marker.id = 1;
  vis_lookahead_point_pub->publish(marker);
}

void PurePursuit::visualize_current_point(Eigen::Vector3d &point) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = global_refFrame;
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  marker.color.a = 1.0;
  marker.color.b = 1.0;

  marker.pose.position.x = point(0);
  marker.pose.position.y = point(1);
  marker.id = 1;
  vis_current_point_pub->publish(marker);
}

void PurePursuit::visualize_speed_point(Eigen::Vector3d &point) {
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = global_refFrame;
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.8;
  marker.pose.position.x = point(0);
  marker.pose.position.y = point(1);
  marker.id = 1;
  vis_speed_point_pub->publish(marker);
}

void PurePursuit::visualize_runtime_params(bool wheel_speed_known,
                                           double wheel_speed) {
  if (!vis_runtime_params_pub) {
    return;
  }

  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = global_refFrame.empty() ? "map" : global_refFrame;
  marker.header.stamp = this->now();
  marker.ns = "pure_pursuit_runtime_params";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = rviz_runtime_params_x;
  marker.pose.position.y = rviz_runtime_params_y;
  marker.pose.position.z = rviz_runtime_params_z;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.35;
  marker.color.a = 1.0;
  marker.color.r = 0.1;
  marker.color.g = 1.0;
  marker.color.b = 0.8;

  std::ostringstream text;
  text << std::fixed << std::setprecision(2)
       << "speed scale: " << velocity_percentage << '\n'
       << "max limit: " << max_speed_limit_percentage << '\n';

  if (wheel_speed_known) {
    text << "wheel: " << wheel_speed;
  } else {
    text << "wheel: n/a";
  }
  if (speed_hold_active_) {
    text << '\n' << "HOLD " << speed_hold_speed_ << " ("
         << std::max(0.0, speed_hold_duration_ - speed_hold_elapsed_) << "s)";
  } else if (launch_start_active_ || launch_start_handoff_active_) {
    text << '\n'
         << (launch_start_active_ ? "LAUNCH " : "LAUNCH HANDOFF ")
         << launch_start_ramp_speed_ << " -> " << target_speed;
  }
  marker.text = text.str();

  vis_runtime_params_pub->publish(marker);
}

bool PurePursuit::should_publish_visualization() {
  if (visualization_rate_hz <= 0.0) {
    return false;
  }

  const auto now = this->now();
  const double period_sec = 1.0 / std::max(visualization_rate_hz, 1e-6);
  if (!visualization_time_initialized_) {
    last_visualization_time_ = now;
    visualization_time_initialized_ = true;
    return true;
  }

  const double elapsed_sec = (now - last_visualization_time_).seconds();
  if (elapsed_sec < 0.0 || elapsed_sec >= period_sec) {
    last_visualization_time_ = now;
    return true;
  }

  return false;
}

int PurePursuit::path_idx_limiter(int idx) const {
  if (num_waypoints <= 0) {
    return 0;
  }

  if (path_is_circular) {
    const int mod = idx % num_waypoints;
    return mod >= 0 ? mod : mod + num_waypoints;
  }

  if (idx < 0) {
    return 0;
  }

  if (idx >= num_waypoints) {
    return num_waypoints - 1;
  }

  return idx;
}

int PurePursuit::advance_index_by_distance(int start_idx, double distance) {
  if (num_waypoints <= 1 || std::abs(distance) <= 1e-6) {
    return path_idx_limiter(start_idx);
  }

  int idx = path_idx_limiter(start_idx);
  const int direction = distance >= 0.0 ? 1 : -1;
  double remaining_distance = std::abs(distance);

  if (path_is_circular && waypoints.path_length > 1e-6) {
    remaining_distance = std::fmod(remaining_distance, waypoints.path_length);
  }

  const int max_segments = path_is_circular ? num_waypoints : num_waypoints - 1;
  int segments_checked = 0;
  while (remaining_distance > 1e-6 && segments_checked < max_segments) {
    const int next_idx =
        path_is_circular ? path_idx_limiter(idx + direction) : idx + direction;
    if (!path_is_circular && (next_idx < 0 || next_idx >= num_waypoints)) {
      break;
    }

    const double segment_distance =
        adjacent_segment_length(idx, next_idx, direction);
    idx = next_idx;
    ++segments_checked;

    if (segment_distance <= 1e-6) {
      continue;
    }
    remaining_distance -= segment_distance;
  }

  return path_idx_limiter(idx);
}

Eigen::Vector3d PurePursuit::sample_path_point_by_distance(
    int start_idx, double distance, int *reached_idx) {
  if (num_waypoints <= 0) {
    if (reached_idx) {
      *reached_idx = 0;
    }
    return Eigen::Vector3d::Zero();
  }

  int idx = path_idx_limiter(start_idx);
  if (num_waypoints == 1 || std::abs(distance) <= 1e-6) {
    if (reached_idx) {
      *reached_idx = idx;
    }
    return Eigen::Vector3d(waypoints.X[idx], waypoints.Y[idx], 0.0);
  }

  const int direction = distance >= 0.0 ? 1 : -1;
  double remaining_distance = std::abs(distance);

  if (path_is_circular && waypoints.path_length > 1e-6) {
    remaining_distance = std::fmod(remaining_distance, waypoints.path_length);
  }

  if (remaining_distance <= 1e-6) {
    if (reached_idx) {
      *reached_idx = idx;
    }
    return Eigen::Vector3d(waypoints.X[idx], waypoints.Y[idx], 0.0);
  }

  const int max_segments = path_is_circular ? num_waypoints : num_waypoints - 1;
  int segments_checked = 0;
  while (segments_checked < max_segments) {
    const int next_idx =
        path_is_circular ? path_idx_limiter(idx + direction) : idx + direction;
    if (!path_is_circular && (next_idx < 0 || next_idx >= num_waypoints)) {
      break;
    }

    const double segment_distance =
        adjacent_segment_length(idx, next_idx, direction);
    ++segments_checked;

    if (segment_distance <= 1e-6) {
      idx = next_idx;
      continue;
    }

    if (remaining_distance <= segment_distance) {
      const double ratio = remaining_distance / segment_distance;
      if (reached_idx) {
        *reached_idx = next_idx;
      }
      return Eigen::Vector3d(
          waypoints.X[idx] + ratio * (waypoints.X[next_idx] - waypoints.X[idx]),
          waypoints.Y[idx] + ratio * (waypoints.Y[next_idx] - waypoints.Y[idx]),
          0.0);
    }

    remaining_distance -= segment_distance;
    idx = next_idx;
  }

  if (reached_idx) {
    *reached_idx = path_idx_limiter(idx);
  }
  return Eigen::Vector3d(waypoints.X[path_idx_limiter(idx)],
                         waypoints.Y[path_idx_limiter(idx)], 0.0);
}

void PurePursuit::get_waypoint_new() {
  if (waypoints.velocity_index < 0) { // 첫 번째 호출 시: 가장 가까운 waypoint 찾기
    int closest_idx = 0;
    double closest_dist_sq =
        planar_distance_sq(waypoints.X[closest_idx], waypoints.Y[closest_idx],
                           x_car_world, y_car_world);
    for (int i = 1; i < num_waypoints; i++) {
      double cur_dist_sq =
          planar_distance_sq(waypoints.X[i], waypoints.Y[i], x_car_world,
                             y_car_world);
      if (cur_dist_sq < closest_dist_sq) {
        closest_idx = i;
        closest_dist_sq = cur_dist_sq;
      }
    }
    waypoints.velocity_index = closest_idx;
  } else {
    // warm start: 현재 velocity_index 부근에서 일정 범위 내에서 가장 가까운 포인트 탐색
    int cur_idx = waypoints.velocity_index;
    const int searching_counter =
        min_searching_idx_offset + max_searching_idx_offset;
    double min_dist_sq = planar_distance_sq(
        waypoints.X[cur_idx], waypoints.Y[cur_idx], x_car_world, y_car_world);

    if (path_is_circular) {
      int backIdx = path_idx_limiter(cur_idx - min_searching_idx_offset);
      for (int i = 0; i < searching_counter; i++) {
        int searching_idx = path_idx_limiter(backIdx + i);
        double searching_dist_sq =
            planar_distance_sq(waypoints.X[searching_idx],
                               waypoints.Y[searching_idx], x_car_world,
                               y_car_world);
        if (searching_dist_sq < min_dist_sq) {
          min_dist_sq = searching_dist_sq;
          cur_idx = searching_idx;
        }
      }
    } else {
      const int start_idx = std::max(cur_idx - min_searching_idx_offset, 0);
      const int end_idx =
          std::min(cur_idx + max_searching_idx_offset, num_waypoints - 1);
      for (int searching_idx = start_idx; searching_idx <= end_idx;
           ++searching_idx) {
        double searching_dist_sq =
            planar_distance_sq(waypoints.X[searching_idx],
                               waypoints.Y[searching_idx], x_car_world,
                               y_car_world);
        if (searching_dist_sq < min_dist_sq) {
          min_dist_sq = searching_dist_sq;
          cur_idx = searching_idx;
        }
      }
    }
    waypoints.velocity_index = cur_idx;
  }

  // lookahead calc
  current_lookahead_distance = std::min(
      std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio),
      max_lookahead);

  // lookahead point는 waypoint 자체가 아니라 segment 위의 정확한 거리 지점을 사용합니다.
  int reached_idx = waypoints.velocity_index;
  lookahead_point_world = sample_path_point_by_distance(
      waypoints.velocity_index, current_lookahead_distance, &reached_idx);
  waypoints.index = reached_idx;

  waypoints.speed_index =
      advance_index_by_distance(waypoints.velocity_index,
                                speed_profile_distance_offset);
}

double PurePursuit::apply_steering_expo(double steering_angle,
                                        double steering_limit_rad) {
  if (steering_limit_rad <= 1e-6) {
    return steering_angle;
  }

  const double limited_angle =
      std::clamp(steering_angle, -steering_limit_rad, steering_limit_rad);
  const double normalized = limited_angle / steering_limit_rad;
  const double abs_normalized = std::abs(normalized);
  const double curve = std::max(steering_expo_curve, 1e-6);
  const double expo_weight = std::pow(abs_normalized, curve);
  const double expo_scale =
      std::max(0.0, 1.0 + steering_expo_gain * expo_weight);

  return std::clamp(normalized * expo_scale, -1.0, 1.0) *
         steering_limit_rad;
}

void PurePursuit::transformandinterp_waypoint() {
  // 현재 추종할 waypoint와 속도 프로파일용 waypoint 업데이트
  // waypoints.index 는 lookahead 인덱스, waypoints.velocity_index 는 차량에 가장
  // 가까운 waypoint 인덱스입니다.
  if (num_waypoints == 0) {
    return;
  }
  // 경계 체크
  int look_idx = path_idx_limiter(waypoints.index);
  int vel_idx = path_idx_limiter(waypoints.velocity_index);
  int speed_idx =
      waypoints.speed_index >= 0 ? path_idx_limiter(waypoints.speed_index)
                                 : vel_idx;
  // lookahead_point_world는 get_waypoint_new()에서 segment 보간으로 갱신됩니다.
  if (look_idx < 0 || look_idx >= num_waypoints) {
    lookahead_point_world << waypoints.X[vel_idx], waypoints.Y[vel_idx], 0.0;
  }
  current_point_world << waypoints.X[vel_idx], waypoints.Y[vel_idx], 0.0;
  speed_point_world << waypoints.X[speed_idx], waypoints.Y[speed_idx], 0.0;

  if (should_publish_visualization()) {
    visualize_lookahead_point(lookahead_point_world);
    visualize_current_point(current_point_world);
    visualize_speed_point(speed_point_world);
  }

  const double dx = lookahead_point_world(0) - x_car_world;
  const double dy = lookahead_point_world(1) - y_car_world;
  const double cos_yaw = std::cos(car_heading);
  const double sin_yaw = std::sin(car_heading);

  lookahead_point_car << cos_yaw * dx + sin_yaw * dy,
      -sin_yaw * dx + cos_yaw * dy, 0.0;
}

double PurePursuit::p_controller() {
  // lookahead point를 이용한 lateral error 계산
  // 차량 좌표계에서의 lookahead 포인트를 이용하여 lateral error 계산
  constexpr double kMinRadiusSq = 1e-6;
  const double y = lookahead_point_car(1);
  const double r_squared = lookahead_point_car.head<2>().squaredNorm();
  double lateral_error = 0.0;
  if (r_squared > kMinRadiusSq) {
    lateral_error = 2.0 * y / r_squared;
  }

  // 차량 주행 방향과 lookahead 방향의 각도 차이
  double heading_error = 0.0;
  if (num_waypoints > 0) {
    const double target_heading = std::atan2(
        lookahead_point_world(1) - y_car_world,
        lookahead_point_world(0) - x_car_world);
    heading_error = normalize_angle(target_heading - car_heading);
  }

  const double combined_error =
      lateral_error + heading_error_gain * heading_error;

  // 현재 시간과 이전 시간의 차이 (dt, 초 단위)
  rclcpp::Time current_time = this->now();
  const double raw_dt = (current_time - prev_time).seconds();

  // use_sim_time으로 /clock이 처음 붙는 순간이나 시계가 되감기는 경우 dt가
  // 수억 초로 튑니다. 그대로 적분하면 integral_error가 한 번에 오염되어
  // 재시작 전에는 복구되지 않으므로, 범위를 벗어난 주기는 건너뜁니다.
  constexpr double kMaxControlDt = 0.2;
  const bool dt_usable =
      std::isfinite(raw_dt) && raw_dt > 0.0 && raw_dt <= kMaxControlDt;
  if (!dt_usable && raw_dt > kMaxControlDt) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Control dt %.3fs is out of range: skipping integral "
                         "and derivative for this cycle",
                         raw_dt);
  }
  const double dt = dt_usable ? raw_dt : 0.0;

  // I제어기: 적분 오차 누적 (적분 windup에 대한 방지 처리는 필요에 따라 추가)
  integral_error += combined_error * dt;

  double derivative = 0.0;
  if (dt > 0.0) {
    derivative = (combined_error - prev_error) / dt;
  }

  // PID 제어: steering angle = K_p * error + K_i * integral_error + K_d *
  // derivative
  double angle =
      K_p * combined_error + K_i * integral_error + K_d * derivative;

  // 이전 오차 및 시간 업데이트
  prev_error = combined_error;
  prev_time = current_time;

  if (path_tangent_gain > 0.0 || understeer_rad_per_mps2 > 0.0) {
    angle += path_tracking_correction();
  }
  return angle;
}

double PurePursuit::path_tracking_correction() {
  // Use the selected reference path alone to estimate its tangent and curvature.
  // Including the displaced vehicle here would multiply the feedback gain by v².
  const double spacing = 0.5 * std::max(current_lookahead_distance, 0.1);
  const int idx = waypoints.velocity_index;
  const Eigen::Vector2d center(waypoints.X[idx], waypoints.Y[idx]);
  const Eigen::Vector2d back = sample_path_point_by_distance(idx, -spacing).head<2>();
  const Eigen::Vector2d front = sample_path_point_by_distance(idx, spacing).head<2>();
  const Eigen::Vector2d u = center - back;
  const Eigen::Vector2d v = front - center;
  const Eigen::Vector2d chord = front - back;
  const double denominator = u.norm() * v.norm() * chord.norm();
  if (denominator < 1e-8) {
    return 0.0;
  }
  const double curvature = 2.0 * (u.x() * v.y() - u.y() * v.x()) / denominator;
  const double heading_error = normalize_angle(std::atan2(chord.y(), chord.x()) - car_heading);
  // Only a fresh measured physical speed contributes to understeer feedforward.
  const double speed = command_domain_wheel_speed(nullptr) ? wheel_speed_measured_ : 0.0;
  return path_tangent_gain * std::sin(heading_error) +
         understeer_rad_per_mps2 * speed * speed * curvature;
}

double PurePursuit::get_velocity(double steering_angle) {
  double velocity = 0.0;
  const int speed_idx =
      waypoints.speed_index >= 0 ? path_idx_limiter(waypoints.speed_index)
                                 : path_idx_limiter(waypoints.velocity_index);
  // waypoint 메시지에서 속도 정보가 제공될 수 있으므로 우선 사용한다.
  if (!waypoints.V.empty() && speed_idx >= 0 &&
      speed_idx < static_cast<int>(waypoints.V.size()) &&
      waypoints.V[speed_idx] > 0.0) {
    velocity = waypoints.V[speed_idx] * velocity_percentage * velocity_reduce_obs;
  } else {
    // 속도 정보가 없으면 steering 각도에 따른 기본 속도를 설정한다.
    double abs_angle = std::abs(steering_angle);
    if (abs_angle < to_radians(10.0)) {
      velocity = 6.0 * velocity_percentage;
    } else if (abs_angle <= to_radians(20.0)) {
      velocity = 2.5 * velocity_percentage;
    } else {
      velocity = 2.0 * velocity_percentage;
    }
  }
  return velocity;
}

void PurePursuit::update_target_command(double steering_angle) {
  const double raw_steering = steering_angle;
  const double steering_limit_rad = to_radians(steering_limit);
  const double steering_for_velocity =
      std::clamp(raw_steering, -steering_limit_rad, steering_limit_rad);
  const double base_desired_speed = get_velocity(steering_for_velocity);
  double desired_speed = base_desired_speed;
  const double min_scale = std::clamp(steer_reduction_min_scale, 0.0, 1.0);
  const double positive_linear =
      std::max(steer_reduction_linear_coef, 0.0);
  const double prev_adjust =
      std::max(0.0, previous_speed_reduction) *
      std::max(0.0, speed_reduction_prev_scale);
  const double total_speed_adjust = speed_reduction_adjust + prev_adjust;
  const double adjusted_drop_speed =
      std::max(0.0, steer_reduction_speed_threshold - total_speed_adjust);

  auto compute_scale = [&](double speed) {
    double scale = 1.0;
    if (speed > steer_reduction_speed_threshold) {
      const double over_speed = speed - steer_reduction_speed_threshold;
      const double candidate =
          steer_reduction_constant_coef - positive_linear * over_speed;
      scale = std::clamp(candidate, min_scale, 1.0);
    }
    return scale;
  };

  double steer_scale = compute_scale(desired_speed);
  double adjusted_steering = raw_steering * steer_scale;
  const double original_abs = std::abs(raw_steering);
  auto calc_drop = [&](double scaled) {
    return std::max(0.0, original_abs - std::abs(scaled));
  };
  double steer_drop = calc_drop(adjusted_steering);

  if (original_abs >= speed_reduction_angle_threshold &&
      steer_drop > max_allowed_steer_drop) {
    const double safe_original =
        std::max(original_abs, 1e-6);
    double required_scale =
        1.0 - max_allowed_steer_drop / safe_original;
    required_scale = std::clamp(required_scale, min_scale, 1.0);

    const double scale_at_threshold =
        std::clamp(steer_reduction_constant_coef, min_scale, 1.0);
    double speed_cap = steer_reduction_speed_threshold;
    if (positive_linear > 1e-6 && required_scale < scale_at_threshold) {
      const double allowed_over_speed =
          (scale_at_threshold - required_scale) / positive_linear;
      speed_cap = steer_reduction_speed_threshold + allowed_over_speed;
    }
    double tuned_speed_cap =
        std::max(0.0, speed_cap - total_speed_adjust);
    desired_speed = std::min(desired_speed, tuned_speed_cap);
    steer_scale = compute_scale(desired_speed);
    adjusted_steering = raw_steering * steer_scale;
    steer_drop = calc_drop(adjusted_steering);

    if (steer_drop > max_allowed_steer_drop &&
        desired_speed > adjusted_drop_speed) {
      desired_speed = adjusted_drop_speed;
      steer_scale = compute_scale(desired_speed);
      adjusted_steering = raw_steering * steer_scale;
      steer_drop = calc_drop(adjusted_steering);
    }

    if (steer_drop > max_allowed_steer_drop) {
      desired_speed =
          std::max(0.0, steer_reduction_speed_threshold - total_speed_adjust);
      steer_scale = compute_scale(desired_speed);
      adjusted_steering = raw_steering * steer_scale;
      steer_drop = calc_drop(adjusted_steering);
    }
  }

  const double expo_steering =
      apply_steering_expo(adjusted_steering, steering_limit_rad);
  target_steer =
      std::clamp(expo_steering, -steering_limit_rad, steering_limit_rad);
  const double expo_scale =
      std::abs(adjusted_steering) > 1e-6
          ? std::abs(target_steer) / std::abs(adjusted_steering)
          : 1.0;

  const double speed_before_max_limit = desired_speed;
  target_speed = apply_max_speed_limit(desired_speed);
  // target_speed와 임계값 게인을 한 쌍으로 갱신합니다. RF 비율 변경과 odom
  // 콜백 사이에 출력 타이머가 끼어도 새 게인과 이전 target을 섞지 않습니다.
  target_speed_launch_gain_ =
      std::clamp(velocity_percentage, 0.0, 1.0);
  const double speed_limit = current_max_speed_limit();
  if (speed_limit >= 0.0 &&
      speed_before_max_limit >= speed_limit - 1e-6) {
    target_speed_launch_gain_ =
        std::min(target_speed_launch_gain_,
                 std::clamp(max_speed_limit_percentage, 0.0, 1.0));
  }
  target_speed_scale_dirty_ = false;
  previous_speed_reduction =
      std::max(0.0, base_desired_speed - speed_before_max_limit);
  has_target_command_ = true;

  RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 200,
      "index: %d ... distance: %.2fm ... TargetSpeed: %.2fm/s ... OutputSpeed: %.2fm/s ... Steering "
      "Angle: %.2f ... Raw Steering: %.2f ... SpeedIdx: %d ... "
      "SpeedOffset: %.2fm ... "
      "MaxPathSpeed: %.2f ... MaxSpeedCap: %.2f ... "
      "Steer Scale: %.2f ... "
      "Expo Scale: %.2f ... "
      "SpeedAdj: %.2f ... PrevReduction: %.2f ... K_p: %.2f "
      "... K_i: %.2f ... velocity_percentage: %.2f",
      waypoints.index,
      p2pdist(lookahead_point_world(0), x_car_world,
              lookahead_point_world(1), y_car_world),
      target_speed, output_speed, to_degrees(target_steer),
      to_degrees(raw_steering), waypoints.speed_index,
      speed_profile_distance_offset, observed_path_max_speed,
      current_max_speed_limit(), steer_scale, expo_scale, total_speed_adjust,
      previous_speed_reduction, K_p, K_i, velocity_percentage);
}

void PurePursuit::drive_output_timer_callback() {
  if (!has_target_command_ || !publisher_drive) {
    return;
  }

  const double speed_alpha = std::clamp(speed_latest_blend, 0.0, 1.0);

  if (!output_command_initialized_) {
    output_steer = target_steer;
    output_speed = target_speed;
    output_command_initialized_ = true;
  } else {
    const double steer_low_alpha = std::clamp(steer_latest_blend, 0.0, 1.0);
    const double steer_high_alpha =
        std::clamp(steer_large_change_blend, steer_low_alpha, 1.0);
    const double steer_threshold_rad =
        std::max(to_radians(steer_blend_change_threshold_deg), 1e-6);
    const double steer_delta_abs = std::abs(target_steer - output_steer);
    const double steer_blend_ratio =
        std::clamp(steer_delta_abs / steer_threshold_rad, 0.0, 1.0);
    const double base_steer_alpha =
        steer_low_alpha +
        steer_blend_ratio * (steer_high_alpha - steer_low_alpha);
    double steer_alpha = base_steer_alpha;

    const double command_speed_abs = std::abs(target_speed);
    const double speed_start =
        std::max(0.0, steer_speed_filter_start_speed);
    const double speed_end = std::max(0.0, steer_speed_filter_end_speed);
    double speed_blend_ratio = 0.0;
    if (speed_end > speed_start) {
      speed_blend_ratio =
          std::clamp((command_speed_abs - speed_start) /
                         (speed_end - speed_start),
                     0.0, 1.0);
    } else if (speed_start > 0.0 && command_speed_abs >= speed_start) {
      speed_blend_ratio = 1.0;
    }

    if (speed_blend_ratio > 0.0) {
      const double speed_final_alpha =
          std::clamp(steer_speed_filter_final_blend, steer_alpha, 1.0);
      steer_alpha += speed_blend_ratio * (speed_final_alpha - steer_alpha);
    }

    output_steer += steer_alpha * (target_steer - output_steer);
    output_speed += speed_alpha * (target_speed - output_speed);
  }

  // 새 런치 요청은 휠 속도와 상승 방향을 확인한 뒤에만 기존 hold/램프를
  // 교체합니다. 검증 실패가 현재의 안전한 출력 상태를 먼저 깨지 않게 합니다.
  process_pending_launch_start();

  // speed hold가 최우선입니다. 유지 시간이 끝나면 update_speed_hold()가
  // 유지 속도에서 램프를 시작시키고, 같은 주기에 아래 램프 갱신이 이어받습니다.
  update_speed_hold();
  if (speed_hold_active_) {
    output_speed = speed_hold_speed_;
  } else {
    // 런치 램프와 release 이후 handoff 동안에는 위 블렌딩 후보의 상승량을
    // 설정 가속도로 제한합니다. 목표 하락은 기존 감속 블렌딩으로 바로 넘깁니다.
    if (update_launch_start()) {
      output_speed = launch_start_ramp_speed_;
    }
  }

  const double steering_limit_rad = to_radians(steering_limit);
  output_steer =
      std::clamp(output_steer, -steering_limit_rad, steering_limit_rad);
  output_speed = apply_max_speed_limit(output_speed);
  // 비동기 최대속도 하향 조정으로 실제 발행값이 램프 상태보다 낮아졌다면,
  // 제한이 다시 올라갈 때 저장된 높은 값으로 복귀하지 않도록 기준도 맞춥니다.
  if ((launch_start_active_ || launch_start_handoff_active_) &&
      output_speed < launch_start_ramp_speed_) {
    launch_start_ramp_speed_ = output_speed;
  }
  curr_velocity = output_speed;

  auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
  drive_msgObj.header.stamp = this->now();
  drive_msgObj.drive.steering_angle = output_steer;
  drive_msgObj.drive.speed = output_speed;

  publisher_drive->publish(drive_msgObj);
}

void PurePursuit::odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
  last_odom_receipt_ = this->now();
  x_car_world = odom_submsgObj->pose.pose.position.x;
  y_car_world = odom_submsgObj->pose.pose.position.y;
  const auto &orientation = odom_submsgObj->pose.pose.orientation;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                    orientation.w);
  car_heading = tf2::getYaw(q);
  // Advance the rear-axle pose to the expected time of steering response.
  // PF twist is a finite difference and may jump on scan corrections.
  if (pose_preview_s > 0.0) {
    const double stamp = rclcpp::Time(odom_submsgObj->header.stamp).seconds();
    const double dt = stamp - last_preview_stamp_;
    const double age = this->now().seconds() - stamp;
    const double yaw_rate = odom_submsgObj->twist.twist.angular.z;
    if (dt < 0.0 || dt > 0.2) {
      filtered_yaw_rate_ = 0.0;
    } else if (dt > 0.0 && std::isfinite(yaw_rate) && std::abs(yaw_rate) < 7.0) {
      filtered_yaw_rate_ += -std::expm1(-dt / 0.045) * (yaw_rate - filtered_yaw_rate_);
    }
    last_preview_stamp_ = stamp;
    if (age >= -0.03 && age <= 0.25 && command_domain_wheel_speed(nullptr)) {
      const double horizon = std::min(0.35, pose_preview_s + std::max(0.0, age));
      const double half_turn = 0.5 * horizon * filtered_yaw_rate_;
      const double sinc = std::abs(half_turn) < 1e-6 ? 1.0 : std::sin(half_turn) / half_turn;
      const double travel = wheel_speed_measured_ * horizon * sinc;
      x_car_world += travel * std::cos(car_heading + half_turn);
      y_car_world += travel * std::sin(car_heading + half_turn);
      car_heading = normalize_angle(car_heading + 2.0 * half_turn);
    }
  }


  if (!global_refFrame.empty() &&
      odom_submsgObj->header.frame_id != global_refFrame) {
    RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Expected odom frame '%s' but received '%s'. Using received frame.",
        global_refFrame.c_str(), odom_submsgObj->header.frame_id.c_str());
  }
  if (!car_refFrame.empty() && !odom_submsgObj->child_frame_id.empty() &&
      odom_submsgObj->child_frame_id != car_refFrame) {
    RCLCPP_WARN_ONCE(
        this->get_logger(),
        "Expected odom child frame '%s' but received '%s'. Using received "
        "frame.",
        car_refFrame.c_str(), odom_submsgObj->child_frame_id.c_str());
  }
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "odom x: %.4f y: %.4f", x_car_world, y_car_world);

  // 경로가 아직 수신되지 않았다면 제어를 수행하지 않고 대기합니다.
  if (!path_received_ || num_waypoints == 0) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No path received yet, waiting for path... (current odom will be ignored)");
    return;
  }

  refresh_tracking_target();
  if (publish_drive_on_odom) {
    drive_output_timer_callback();
  }
}

void PurePursuit::refresh_tracking_target() {
  get_waypoint_new();
  transformandinterp_waypoint();
  update_target_command(p_controller());
}

void PurePursuit::obs_odom_callback(const geometry_msgs::msg::PointStamped msg){
  x_obs = msg.point.x;
  y_obs = msg.point.y;
}

void PurePursuit::obs_status_callback(const geometry_msgs::msg::PointStamped msg){ //장애물 정보 반영 속도 줄이기
  int obsIsValid = int(msg.point.x);
  if(slow_with_obs && obsIsValid){
    if(p2pdist(x_obs, x_car_world, y_obs, y_car_world)<slow_th_dist){
      velocity_reduce_obs = slow_amount;
    }
    else{
      velocity_reduce_obs = 1;
    }
  }
  else{
    velocity_reduce_obs = 1;
  }

}

void PurePursuit::rf_callback(
    const std_msgs::msg::UInt16MultiArray::ConstSharedPtr rf_msg) {
  if (!rf_msg) {
    return;
  }

  // 런치 스타트 트리거는 enable 스위치와 무관하게 판정합니다.
  // 두 채널 중 어느 쪽이 올라가도 같은 래치/램프업이 걸립니다.
  check_launch_start_channel(*rf_msg, launch_start_channel,
                             launch_start_channel_threshold,
                             &launch_start_prev_raw_,
                             &launch_start_prev_raw_valid_, "RF channel");
  check_launch_start_channel(*rf_msg, launch_start_channel_2,
                             launch_start_channel_2_threshold,
                             &launch_start_prev_raw_2_,
                             &launch_start_prev_raw_2_valid_, "RF channel 2");

  int enable_raw = 0;
  if (!read_rf_channel(*rf_msg, rf_enable_channel, &enable_raw)) {
    rf_runtime_control_active = false;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "RF enable channel index %d is unavailable. /rf size: %zu",
        rf_enable_channel, rf_msg->data.size());
    return;
  }

  // 스위치를 내렸을 때(raw가 임계값 미만) 반영되는 반전 로직입니다.
  const bool should_apply_rf = enable_raw < rf_enable_threshold;
  if (!should_apply_rf) {
    if (rf_runtime_control_active) {
      RCLCPP_INFO(this->get_logger(),
                  "RF runtime control disabled by channel index %d: %d >= %d",
                  rf_enable_channel, enable_raw, rf_enable_threshold);
    }
    rf_runtime_control_active = false;
    return;
  }

  int speed_scale_raw = 0;
  int max_limit_raw = 0;
  if (!read_rf_channel(*rf_msg, rf_speed_scale_channel, &speed_scale_raw) ||
      !read_rf_channel(*rf_msg, rf_max_limit_channel, &max_limit_raw)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "RF parameter channel is unavailable. speed index: %d, max index: %d, "
        "/rf size: %zu",
        rf_speed_scale_channel, rf_max_limit_channel, rf_msg->data.size());
    return;
  }

  if (!rf_runtime_control_active) {
    RCLCPP_INFO(this->get_logger(),
                "RF runtime control enabled by channel index %d: %d < %d",
                rf_enable_channel, enable_raw, rf_enable_threshold);
  }
  rf_runtime_control_active = true;

  set_runtime_percentages(rf_raw_to_unit(speed_scale_raw),
                          rf_raw_to_unit(max_limit_raw));
}

// VESC telemetry(sensors/core)의 전기 RPM을 휠 속도(m/s)로 변환해 보관합니다.
//   speed = (erpm - speed_to_erpm_offset) / speed_to_erpm_gain
// vesc_to_odom과 동일한 식이며 저속 데드밴드도 동일하게 적용합니다.
void PurePursuit::vesc_state_callback(
    const vesc_msgs::msg::VescStateStamped::ConstSharedPtr state_msg) {
  if (!state_msg) {
    return;
  }

  if (std::abs(speed_to_erpm_gain) < 1e-6) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "speed_to_erpm_gain(%.6f) is too small to convert "
                         "wheel speed",
                         speed_to_erpm_gain);
    return;
  }

  const double erpm = state_msg->state.speed;
  if (!std::isfinite(erpm)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Ignoring non-finite wheel speed on %s",
                         vesc_topic_display_name());
    return;
  }

  double speed = (erpm - speed_to_erpm_offset) / speed_to_erpm_gain;
  if (!std::isfinite(speed)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Ignoring wheel speed that converted to a non-finite value on %s",
        vesc_topic_display_name());
    return;
  }
  if (std::abs(speed) < std::max(0.0, wheel_speed_deadband)) {
    speed = 0.0;
  }

  if (!wheel_speed_reported_alive_) {
    wheel_speed_reported_alive_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Wheel speed source alive on %s: %.2f erpm -> %.2f m/s "
                "(x%.2f = %.2f in command domain)",
                vesc_topic_display_name(), erpm, speed, wheel_speed_scale,
                speed * wheel_speed_scale);
  }

  wheel_speed_measured_ = speed;
  wheel_speed_stamp_ = this->now();
  wheel_speed_valid_ = true;
}

// 다른 노드가 런치 스타트를 걸거나(true) 해제(false)합니다.
void PurePursuit::launch_start_reset_callback(
    const std_msgs::msg::Bool::ConstSharedPtr reset_msg) {
  if (!reset_msg) {
    return;
  }

  if (reset_msg->data) {
    request_launch_start("reset topic");
  } else {
    cancel_launch_start("reset topic");
  }
}

// 측정 휠 속도를 명령 속도(path speed) 도메인으로 변환해 돌려줍니다.
// 값이 없거나 오래됐으면 false.
// 로그마다 삼항 연산자를 반복하지 않도록, 실제로 해석된 토픽 이름을 돌려줍니다.
const char *PurePursuit::vesc_topic_display_name() const {
  return subscription_vesc_state ? subscription_vesc_state->get_topic_name()
                                 : vesc_state_topic.c_str();
}

bool PurePursuit::command_domain_wheel_speed(double *speed) const {
  if (!wheel_speed_valid_) {
    return false;
  }

  if (wheel_speed_timeout > 0.0) {
    const double age = (this->now() - wheel_speed_stamp_).seconds();
    if (age < 0.0 || age > wheel_speed_timeout) {
      return false;
    }
  }

  if (speed) {
    *speed = wheel_speed_measured_ * wheel_speed_scale;
  }
  return true;
}

// 임계값 이하였다가 초과로 "올라가는" 순간에만 한 번 런치 스타트를 겁니다.
// 첫 메시지는 직전 값이 없으므로 엣지로 보지 않습니다.
void PurePursuit::check_launch_start_channel(
    const std_msgs::msg::UInt16MultiArray &rf_msg, int channel_index,
    int threshold, int *prev_raw, bool *prev_raw_valid, const char *source) {
  // 음수 인덱스는 이 트리거를 쓰지 않겠다는 뜻입니다.
  if (channel_index < 0) {
    return;
  }

  int raw = 0;
  if (!read_rf_channel(rf_msg, channel_index, &raw)) {
    // 인덱스가 배열 밖이면 트리거가 조용히 사라지므로 알립니다.
    // 기능을 꺼둔 장비에서는 굳이 경고하지 않습니다.
    if (launch_start_enabled) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "%s index %d is unavailable. /rf size: %zu", source, channel_index,
          rf_msg.data.size());
    }
    *prev_raw_valid = false;
    return;
  }

  const bool is_above = raw > threshold;
  const bool was_above = *prev_raw > threshold;
  if (*prev_raw_valid && is_above && !was_above) {
    request_launch_start(source);
  }
  *prev_raw = raw;
  *prev_raw_valid = true;
}

void PurePursuit::request_launch_start(const char *source) {
  const char *source_name = source ? source : "unknown source";

  if (!launch_start_enabled) {
    // 기존 계약상 true 트리거는 런치가 비활성이어도 hold 취소로 사용됩니다.
    cancel_speed_hold(source_name);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Launch start request from %s ignored: "
                         "launch_start_enabled is false",
                         source_name);
    return;
  }

  // 실제 전환은 출력 콜백에서 최신 target과 wheel을 함께 확인한 뒤 합니다.
  // 검증 전에 현재 hold/램프를 지우면 실패한 재트리거가 출력 단차를 만듭니다.
  launch_start_pending_ = true;
  launch_start_pending_source_ = source_name;
  RCLCPP_INFO(this->get_logger(), "Launch start requested by %s", source_name);
}

void PurePursuit::process_pending_launch_start() {
  if (!launch_start_pending_) {
    return;
  }

  // 속도 비율 변경은 다음 odom에서 target_speed에 반영됩니다. 그 전에는
  // 요청을 소비하지 않아 이전 target과 새 임계값 게인을 섞지 않습니다.
  if (target_speed_scale_dirty_) {
    return;
  }

  launch_start_pending_ = false;
  const std::string source = launch_start_pending_source_.empty()
                                 ? "trigger"
                                 : launch_start_pending_source_;
  launch_start_pending_source_.clear();

  if (!launch_start_enabled) {
    RCLCPP_WARN(this->get_logger(),
                "Launch start request from %s ignored: "
                "launch_start_enabled is false",
                source.c_str());
    return;
  }

  double wheel_speed = 0.0;
  if (!command_domain_wheel_speed(&wheel_speed)) {
    RCLCPP_WARN(this->get_logger(),
                "Launch start skipped: no fresh wheel speed on %s",
                vesc_topic_display_name());
    return;
  }

  const double gain = target_speed_launch_gain_;
  const double effective_engage_diff = launch_start_engage_diff * gain;
  const double target_gap = target_speed - wheel_speed;
  if (!launch_ramp::should_engage(target_speed, wheel_speed,
                                  effective_engage_diff)) {
    RCLCPP_INFO(this->get_logger(),
                "Launch start skipped: target %.2f - wheel %.2f = %.2f "
                "is below positive engage diff %.2f (%.2f x gain %.2f)",
                target_speed, wheel_speed, target_gap, effective_engage_diff,
                launch_start_engage_diff, gain);
    return;
  }

  // 새 런치가 실제로 시작될 때만 기존 hold/램프를 교체합니다.
  cancel_speed_hold(source.c_str());
  begin_launch_ramp(wheel_speed, source.c_str());
}

void PurePursuit::cancel_launch_start(const char *source) {
  cancel_speed_hold(source);
  if (!launch_start_active_ && !launch_start_handoff_active_ &&
      !launch_start_pending_) {
    return;
  }

  launch_start_active_ = false;
  launch_start_handoff_active_ = false;
  launch_start_pending_ = false;
  launch_start_pending_source_.clear();
  launch_start_time_valid_ = false;
  RCLCPP_INFO(this->get_logger(), "Launch start cancelled by %s", source);
}

// 지정한 속도에서 램프를 시작합니다. 트리거 경로와 speed hold 만료 경로가
// 같은 상태를 쓰도록 여기 한 곳으로 모읍니다.
void PurePursuit::begin_launch_ramp(double from_speed, const char *source) {
  launch_start_pending_ = false;
  launch_start_pending_source_.clear();
  launch_start_active_ = true;
  launch_start_handoff_active_ = false;
  launch_start_ramp_speed_ = from_speed;
  launch_start_time_valid_ = false;
  launch_start_begin_speed_ = from_speed;
  launch_start_elapsed_ = 0.0;
  // release 이후 handoff도 같은 가속도 상한을 유지하므로 목표까지의 이론적인
  // 최소 시간을 표시합니다(출력 blend가 더 느리면 실제 완료는 늦어집니다).
  const double expected_sec =
      launch_start_accel > 1e-6
          ? std::max(0.0, target_speed - from_speed) / launch_start_accel
          : 0.0;
  RCLCPP_INFO(this->get_logger(),
              "Launch ramp starting from %.2f m/s toward target %.2f m/s at "
              "%.2f m/s^2, expected %.2fs (%s)",
              from_speed, target_speed, launch_start_accel, expected_sec,
              source);
}

// 외부 노드가 [speed, duration]으로 "n초 동안 m m/s 유지"를 요청합니다.
// duration을 생략하면 speed_hold_default_duration을 씁니다.
// 요청 즉시 그 속도로 고정되고, 끝나면 곧바로 램프업으로 넘어갑니다.
void PurePursuit::speed_hold_callback(
    const std_msgs::msg::Float64MultiArray::ConstSharedPtr hold_msg) {
  auto reject = [this](const char *reason) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Speed hold request ignored: %s", reason);
  };

  if (!hold_msg || hold_msg->data.empty()) {
    reject("expected [speed] or [speed, duration]");
    return;
  }

  // 상한이 0이면 기능을 끈 것으로 봅니다. 0으로 잘라서 즉시 램프업하면
  // 끄려던 조작이 오히려 가장 공격적인 동작이 됩니다.
  if (speed_hold_max_duration <= 0.0) {
    reject("speed_hold_max_duration is 0 (speed hold disabled)");
    return;
  }

  const double speed = hold_msg->data[0];
  double duration = hold_msg->data.size() > 1 ? hold_msg->data[1]
                                              : speed_hold_default_duration;

  if (!std::isfinite(speed) || !std::isfinite(duration)) {
    reject("speed or duration is not finite");
    return;
  }

  if (speed < 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Speed hold request ignored: speed %.2f is negative",
                         speed);
    return;
  }

  // 이 속도는 경로에서 유도된 값이 아니라 외부가 직접 준 값입니다.
  // observed_path_max_speed가 아직 0이면 apply_max_speed_limit()이 아무것도
  // 막지 않으므로, 경로와 무관한 절대 상한을 여기서 겁니다.
  if (speed > speed_hold_max_speed) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Speed hold request ignored: speed %.2f m/s exceeds "
        "speed_hold_max_speed %.2f m/s",
        speed, speed_hold_max_speed);
    return;
  }

  if (duration < 0.0) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Speed hold request ignored: duration %.2fs is negative", duration);
    return;
  }

  // 잘못된 요청 하나로 무한정 붙잡히지 않도록 상한을 둡니다.
  if (duration > speed_hold_max_duration) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Speed hold duration %.2fs clamped to speed_hold_max_duration %.2fs",
        duration, speed_hold_max_duration);
    duration = speed_hold_max_duration;
  }

  // 가장 최근 요청이 이깁니다: 진행 중인 hold나 램프를 덮어씁니다.
  launch_start_pending_ = false;
  launch_start_pending_source_.clear();
  launch_start_active_ = false;
  launch_start_handoff_active_ = false;
  launch_start_time_valid_ = false;
  speed_hold_active_ = false;
  speed_hold_time_valid_ = false;

  if (duration == 0.0) {
    // 유지 시간을 명시적으로 0으로 준 경우: 그 속도에서 바로 램프업만 합니다.
    if (!launch_start_enabled) {
      RCLCPP_INFO(this->get_logger(),
                  "Speed hold with zero duration ignored: "
                  "launch_start_enabled is false");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Speed hold with zero duration: ramping from %.2f m/s", speed);
    begin_launch_ramp(speed, "speed hold");
    return;
  }

  speed_hold_active_ = true;
  speed_hold_speed_ = speed;
  speed_hold_duration_ = duration;
  speed_hold_elapsed_ = 0.0;
  RCLCPP_INFO(this->get_logger(), "Speed hold: %.2f m/s for %.2fs, then ramp",
              speed, duration);
}

void PurePursuit::cancel_speed_hold(const char *source) {
  if (!speed_hold_active_) {
    return;
  }
  speed_hold_active_ = false;
  speed_hold_time_valid_ = false;
  RCLCPP_INFO(this->get_logger(), "Speed hold cancelled by %s", source);
}

// 유지 시간이 끝나면 유지하던 속도에서 곧바로 램프업으로 인계합니다.
// 램프 시작점을 측정 휠 속도가 아니라 유지 속도로 잡아, 명령이 튀지 않게 합니다.
void PurePursuit::update_speed_hold() {
  if (!speed_hold_active_) {
    return;
  }

  // 벽시계 만료 대신 "실제로 유지한 시간"을 누적합니다. 경로가 늦게 들어와
  // 출력이 시작되기 전이면 시간이 흐르지 않고, 시계가 되감겨도 멈추지 않습니다.
  const rclcpp::Time now = this->now();
  double dt = speed_hold_time_valid_ ? (now - speed_hold_prev_time_).seconds()
                                     : 0.0;
  speed_hold_prev_time_ = now;
  speed_hold_time_valid_ = true;
  speed_hold_elapsed_ += std::clamp(dt, 0.0, 0.2);

  if (speed_hold_elapsed_ < speed_hold_duration_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
                         "Speed hold: %.2f m/s (%.2fs left)", speed_hold_speed_,
                         speed_hold_duration_ - speed_hold_elapsed_);
    return;
  }

  speed_hold_active_ = false;
  speed_hold_time_valid_ = false;
  if (!launch_start_enabled) {
    RCLCPP_INFO(this->get_logger(),
                "Speed hold finished at %.2f m/s: launch_start_enabled is "
                "false, returning to normal control",
                speed_hold_speed_);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Speed hold finished at %.2f m/s",
              speed_hold_speed_);
  begin_launch_ramp(speed_hold_speed_, "speed hold");
}

// 런치 스타트가 켜져 있는데 휠 속도가 안 들어오면 트리거를 당겨도 무시됩니다.
// 경로 수신 여부와 무관하게 돌도록 출력 콜백이 아닌 상태 타이머에서 부릅니다.
void PurePursuit::check_launch_start_health(bool wheel_speed_known) {
  if (!launch_start_enabled || wheel_speed_known) {
    return;
  }

  // 다음에 값이 다시 들어오면 복구 사실을 한 번 알리도록 되돌립니다.
  wheel_speed_reported_alive_ = false;
  RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Launch start is armed but %s has %s: a trigger will be ignored",
      vesc_topic_display_name(),
      wheel_speed_valid_ ? "gone stale" : "never published");
}

// 200ms 상태 타이머. 휠 속도 조회는 여기서 한 번만 하고 아래로 넘깁니다.
void PurePursuit::runtime_status_timer_callback() {
  double wheel_speed = 0.0;
  const bool wheel_speed_known = command_domain_wheel_speed(&wheel_speed);
  check_launch_start_health(wheel_speed_known);
  visualize_runtime_params(wheel_speed_known, wheel_speed);
}

// 런치 램프와 release 이후 handoff 제한을 갱신합니다. 상승 중에는 두 상태 모두
// launch_start_accel을 지키고, 목표가 내려가면 즉시 기존 감속 블렌딩으로
// 돌려줍니다. 반환값은 이번 주기에 ramp 속도를 출력해야 한다는 뜻입니다.
bool PurePursuit::update_launch_start() {
  if (!launch_start_enabled) {
    launch_start_active_ = false;
    launch_start_handoff_active_ = false;
    launch_start_pending_ = false;
    launch_start_pending_source_.clear();
    launch_start_time_valid_ = false;
    return false;
  }

  if (!launch_start_active_ && !launch_start_handoff_active_) {
    return false;
  }

  if (!std::isfinite(target_speed) ||
      !std::isfinite(launch_start_ramp_speed_) ||
      !std::isfinite(output_speed)) {
    launch_start_active_ = false;
    launch_start_handoff_active_ = false;
    launch_start_time_valid_ = false;
    RCLCPP_ERROR(
        this->get_logger(),
        "Launch start cancelled: target, ramp, or output speed is non-finite");
    return false;
  }

  // 트리거 뒤에도 최신 실측 속도를 계속 확인합니다. 차량이 이미 목표에
  // 도달했거나 넘어섰다면 launch가 감속이나 추가 상승 명령을 만들지 않고
  // 즉시 normal control에 ownership을 돌려줍니다.
  double wheel_speed = 0.0;
  const bool wheel_speed_known = command_domain_wheel_speed(&wheel_speed);
  if (wheel_speed_known &&
      !launch_ramp::target_is_above(target_speed, wheel_speed)) {
    // RF 비율 변경 직후라면 현재 launch는 해제하되, 새 target pair가 계산된
    // 뒤 휠 속도에서 다시 진입 여부를 판단해 normal blend 급상승을 막습니다.
    if (target_speed_scale_dirty_ && !launch_start_pending_) {
      launch_start_pending_ = true;
      launch_start_pending_source_ = "target refresh";
    }
    launch_start_active_ = false;
    launch_start_handoff_active_ = false;
    launch_start_time_valid_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Launch control released: wheel %.2f m/s is at or above "
                "target %.2f m/s%s",
                wheel_speed, target_speed,
                target_speed_scale_dirty_
                    ? "; will re-evaluate after target refresh"
                    : "");
    return false;
  }

  // RF/파라미터 속도 비율이 먼저 바뀌고 새 target_speed가 아직 계산되지
  // 않았다면 현재 램프 명령을 유지합니다. 이전 target에서 래치를 풀어 버린 뒤
  // 다음 odom의 더 높은 target으로 일반 블렌딩하는 순서 역전을 막습니다.
  if (target_speed_scale_dirty_) {
    launch_start_time_valid_ = false;
    return true;
  }

  // 목표가 램프 명령까지 내려온 경우에는 휠 telemetry가 없어도 감속을 막으면
  // 안 됩니다. normal control이 이미 계산한 감속 후보로 즉시 넘깁니다.
  if (!launch_ramp::target_is_above(target_speed,
                                    launch_start_ramp_speed_)) {
    launch_start_active_ = false;
    launch_start_handoff_active_ = false;
    launch_start_time_valid_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Launch control released: target %.2f m/s is at or below "
                "ramp %.2f m/s",
                target_speed, launch_start_ramp_speed_);
    return false;
  }

  if (!wheel_speed_known) {
    launch_start_time_valid_ = false;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Launch control paused: no fresh wheel speed on %s",
        vesc_topic_display_name());
    return true;
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = 0.0;
  if (launch_start_time_valid_) {
    dt = std::chrono::duration<double>(now - launch_start_prev_time_).count();
  }
  launch_start_prev_time_ = now;
  launch_start_time_valid_ = true;
  // Timer와 odom이 모두 출력을 만들 수 있습니다. 실제 callback 간격을 쓰되,
  // 지연 뒤 한 발행에서 여러 step을 따라잡지 않도록 nominal 한 주기로 제한합니다.
  const double output_rate = std::clamp(
      active_drive_output_rate_hz > 0.0 ? active_drive_output_rate_hz
                                        : drive_output_rate_hz,
      1.0, 200.0);
  const double nominal_period = 1.0 / output_rate;
  dt = std::clamp(dt, 0.0, nominal_period);
  launch_start_elapsed_ += dt;

  const double max_step = launch_ramp::bounded_max_step(
      launch_start_accel, dt, nominal_period);
  const double gain = target_speed_launch_gain_;
  const double effective_release_diff = launch_start_release_diff * gain;

  if (launch_start_handoff_active_) {
    const auto handoff_step = launch_ramp::advance_handoff(
        launch_start_ramp_speed_, target_speed, output_speed, max_step);

    if (handoff_step.target_not_above_ramp) {
      launch_start_handoff_active_ = false;
      launch_start_time_valid_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Launch handoff released after %.2fs: target %.2f m/s is "
                  "no longer above limited speed %.2f m/s",
                  launch_start_elapsed_, target_speed,
                  launch_start_ramp_speed_);
      return false;
    }

    launch_start_ramp_speed_ = handoff_step.speed;
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 250,
        "Launch handoff: %.2f -> target %.2f m/s (step %.3f, dt %.4f)",
        launch_start_ramp_speed_, target_speed, max_step, dt);

    if (!handoff_step.active) {
      launch_start_handoff_active_ = false;
      launch_start_time_valid_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Launch handoff completed after %.2fs: %.2f -> %.2f m/s",
                  launch_start_elapsed_, launch_start_begin_speed_,
                  launch_start_ramp_speed_);
    }
    return handoff_step.command_valid;
  }

  const auto ramp_step = launch_ramp::advance(
      launch_start_ramp_speed_, target_speed, max_step,
      effective_release_diff);

  if (ramp_step.target_not_above_ramp) {
    launch_start_active_ = false;
    launch_start_handoff_active_ = false;
    launch_start_time_valid_ = false;
    RCLCPP_INFO(this->get_logger(),
                "Launch start released after %.2fs: target %.2f m/s is no "
                "longer above ramp %.2f m/s; returning to normal deceleration",
                launch_start_elapsed_, target_speed,
                launch_start_ramp_speed_);
    return false;
  }

  launch_start_ramp_speed_ = ramp_step.speed;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
                       "Launch ramp: %.2f -> target %.2f m/s (step %.3f, "
                       "dt %.3f)",
                       launch_start_ramp_speed_, target_speed, max_step, dt);

  if (!ramp_step.active) {
    launch_start_active_ = false;
    launch_start_handoff_active_ =
        launch_start_ramp_speed_ < target_speed;
    if (!launch_start_handoff_active_) {
      launch_start_time_valid_ = false;
    }
    RCLCPP_INFO(this->get_logger(),
                "Launch start released after %.2fs: %.2f -> ramp %.2f m/s, "
                "target %.2f m/s (release %.3f = %.2f x gain %.2f)%s",
                launch_start_elapsed_, launch_start_begin_speed_,
                launch_start_ramp_speed_, target_speed,
                effective_release_diff, launch_start_release_diff, gain,
                launch_start_handoff_active_
                    ? "; acceleration-limited handoff continuing"
                    : "");
  }

  // release 주기에도 마지막 램프 값을 출력합니다. 남은 차이가 있으면 다음
  // 주기부터 handoff limiter가 normal blend 후보의 상승량을 계속 제한합니다.
  return ramp_step.command_valid;
}

// 런타임 파라미터 하나가 반영해도 안전한 값인지 검사합니다.
// 숫자가 아닌 파라미터(토픽 이름, 플래그 등)는 검사 없이 통과시킵니다.
bool PurePursuit::validate_runtime_parameter(const rclcpp::Parameter &parameter,
                                             std::string *reason) const {
  const std::string &name = parameter.get_name();

  const RuntimeParameterBound *bound = nullptr;
  for (const auto &candidate : kRuntimeParameterBounds) {
    if (name == candidate.name) {
      bound = &candidate;
      break;
    }
  }

  const auto type = parameter.get_type();
  if (type != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      type != rclcpp::ParameterType::PARAMETER_INTEGER) {
    if (bound != nullptr) {
      if (reason) {
        *reason = name + " must be a number";
      }
      return false;
    }
    return true;
  }

  const double value =
      type == rclcpp::ParameterType::PARAMETER_INTEGER
          ? static_cast<double>(parameter.as_int())
          : parameter.as_double();

  // NaN/inf는 한 번 들어오면 제어 출력과 적분항까지 오염되어 복구되지 않습니다.
  if (!std::isfinite(value)) {
    if (reason) {
      *reason = name + " must be a finite number (NaN/inf is rejected)";
    }
    return false;
  }

  if (bound != nullptr &&
      (value < bound->min_value || value > bound->max_value)) {
    if (reason) {
      std::ostringstream oss;
      oss << name << " must be in [" << bound->min_value << ", ";
      if (bound->max_value == kUnbounded) {
        oss << "inf";
      } else {
        oss << bound->max_value;
      }
      oss << "] (got " << value << ")";
      *reason = oss.str();
    }
    return false;
  }

  return true;
}

// 런타임에 변경 가능한 파라미터 하나를 대응하는 멤버 변수에 반영합니다.
// 처리한 파라미터면 true, 목록에 없는 이름이면 false를 반환합니다.
bool PurePursuit::apply_runtime_parameter(const rclcpp::Parameter &parameter) {
  const std::string &name = parameter.get_name();

  // 속도 비율 파라미터는 int로도 들어올 수 있어 별도 처리
  auto as_clamped_ratio = [&parameter]() {
    const double value =
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
            ? static_cast<double>(parameter.as_int())
            : parameter.as_double();
    return std::clamp(value, 0.0, 1.0);
  };

  if (name == "drive_topic") {
    drive_topic = parameter.as_string();
  } else if (name == "test_mode") {
    test_mode = parameter.as_bool();
  } else if (name == "drive_test_topic") {
    drive_test_topic = parameter.as_string();
  } else if (name == "rviz_runtime_params_x") {
    rviz_runtime_params_x = parameter.as_double();
  } else if (name == "rviz_runtime_params_y") {
    rviz_runtime_params_y = parameter.as_double();
  } else if (name == "rviz_runtime_params_z") {
    rviz_runtime_params_z = parameter.as_double();
  } else if (name == "rf_speed_scale_channel") {
    rf_speed_scale_channel = parameter.as_int();
  } else if (name == "rf_max_limit_channel") {
    rf_max_limit_channel = parameter.as_int();
  } else if (name == "rf_enable_channel") {
    rf_enable_channel = parameter.as_int();
  } else if (name == "rf_enable_threshold") {
    rf_enable_threshold = parameter.as_int();
  } else if (name == "rf_value_min") {
    rf_value_min = parameter.as_int();
  } else if (name == "rf_value_max") {
    rf_value_max = parameter.as_int();
  } else if (name == "K_p") {
    K_p = parameter.as_double();
  } else if (name == "K_d") {
    K_d = parameter.as_double();
  } else if (name == "K_i") {
    K_i = parameter.as_double();
  } else if (name == "pose_preview_s") {
    pose_preview_s = parameter.as_double();
  } else if (name == "path_tangent_gain") {
    path_tangent_gain = parameter.as_double();
  } else if (name == "understeer_rad_per_mps2") {
    understeer_rad_per_mps2 = parameter.as_double();
  } else if (name == "heading_error_gain") {
    heading_error_gain = parameter.as_double();
  } else if (name == "velocity_percentage") {
    velocity_percentage = as_clamped_ratio();
    target_speed_scale_dirty_ = true;
  } else if (name == "max_speed_limit_percentage") {
    max_speed_limit_percentage = as_clamped_ratio();
    target_speed_scale_dirty_ = true;
  } else if (name == "min_lookahead") {
    min_lookahead = parameter.as_double();
  } else if (name == "max_lookahead") {
    max_lookahead = parameter.as_double();
  } else if (name == "lookahead_ratio") {
    lookahead_ratio = parameter.as_double();
  } else if (name == "speed_profile_distance_offset") {
    speed_profile_distance_offset = parameter.as_double();
  } else if (name == "steering_limit") {
    steering_limit = parameter.as_double();
  } else if (name == "steering_expo_gain") {
    steering_expo_gain = parameter.as_double();
  } else if (name == "steering_expo_curve") {
    steering_expo_curve = parameter.as_double();
  } else if (name == "steer_reduction_speed_threshold") {
    steer_reduction_speed_threshold = parameter.as_double();
  } else if (name == "steer_reduction_constant_coef") {
    steer_reduction_constant_coef = parameter.as_double();
  } else if (name == "steer_reduction_linear_coef") {
    steer_reduction_linear_coef = parameter.as_double();
  } else if (name == "steer_reduction_min_scale") {
    steer_reduction_min_scale = parameter.as_double();
  } else if (name == "speed_reduction_steer_angle_deg") {
    speed_reduction_angle_threshold = to_radians(parameter.as_double());
  } else if (name == "max_allowed_steer_drop_deg") {
    max_allowed_steer_drop = to_radians(parameter.as_double());
  } else if (name == "speed_reduction_adjust") {
    speed_reduction_adjust = parameter.as_double();
  } else if (name == "speed_reduction_prev_scale") {
    speed_reduction_prev_scale = parameter.as_double();
  } else if (name == "drive_output_rate_hz") {
    drive_output_rate_hz = parameter.as_double();
  } else if (name == "publish_drive_on_odom") {
    publish_drive_on_odom = parameter.as_bool();
  } else if (name == "visualization_rate_hz") {
    visualization_rate_hz = parameter.as_double();
  } else if (name == "steer_latest_blend") {
    steer_latest_blend = parameter.as_double();
  } else if (name == "steer_large_change_blend") {
    steer_large_change_blend = parameter.as_double();
  } else if (name == "steer_blend_change_threshold_deg") {
    steer_blend_change_threshold_deg = parameter.as_double();
  } else if (name == "steer_speed_filter_start_speed") {
    steer_speed_filter_start_speed = parameter.as_double();
  } else if (name == "steer_speed_filter_end_speed") {
    steer_speed_filter_end_speed = parameter.as_double();
  } else if (name == "steer_speed_filter_final_blend") {
    steer_speed_filter_final_blend = parameter.as_double();
  } else if (name == "speed_latest_blend") {
    speed_latest_blend = parameter.as_double();
  } else if (name == "min_searching_idx_offset") {
    min_searching_idx_offset = parameter.as_int();
  } else if (name == "max_searching_idx_offset") {
    max_searching_idx_offset = parameter.as_int();
  } else if (name == "slow_with_obs") {
    slow_with_obs = parameter.as_bool();
  } else if (name == "obs_slow_th") {
    slow_th_dist = parameter.as_double();
  } else if (name == "obs_slow_percentage") {
    slow_amount = parameter.as_double();
  } else if (name == "speed_to_erpm_gain") {
    speed_to_erpm_gain = parameter.as_double();
  } else if (name == "speed_to_erpm_offset") {
    speed_to_erpm_offset = parameter.as_double();
  } else if (name == "wheel_speed_deadband") {
    wheel_speed_deadband = parameter.as_double();
  } else if (name == "wheel_speed_scale") {
    wheel_speed_scale = parameter.as_double();
  } else if (name == "wheel_speed_timeout") {
    wheel_speed_timeout = parameter.as_double();
  } else if (name == "launch_start_enabled") {
    launch_start_enabled = parameter.as_bool();
  } else if (name == "launch_start_channel") {
    launch_start_channel = parameter.as_int();
    launch_start_prev_raw_valid_ = false;
  } else if (name == "launch_start_channel_threshold") {
    launch_start_channel_threshold = parameter.as_int();
  } else if (name == "launch_start_channel_2") {
    launch_start_channel_2 = parameter.as_int();
    launch_start_prev_raw_2_valid_ = false;
  } else if (name == "launch_start_channel_2_threshold") {
    launch_start_channel_2_threshold = parameter.as_int();
  } else if (name == "launch_start_engage_diff") {
    launch_start_engage_diff = parameter.as_double();
  } else if (name == "launch_start_release_diff") {
    launch_start_release_diff = parameter.as_double();
  } else if (name == "launch_start_accel") {
    launch_start_accel = parameter.as_double();
  } else if (name == "speed_hold_default_duration") {
    speed_hold_default_duration = parameter.as_double();
  } else if (name == "speed_hold_max_duration") {
    speed_hold_max_duration = parameter.as_double();
  } else if (name == "speed_hold_max_speed") {
    speed_hold_max_speed = parameter.as_double();
  } else {
    return false;
  }

  return true;
}

double PurePursuit::normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_ptr = std::make_shared<PurePursuit>();
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
