#ifndef ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H
#define ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H

#include "athena_motor_driver/controllers/controller_base.h"

#include <athena_motor_interface/athena_motor_interfaces.h>
#include <athena_motor_interface/msg/debug_data.hpp>
#include <athena_motor_interface/msg/full_motor_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hector_ros2_utils/node.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace athena_motor_driver
{

class AthenaMotorDriver : public hector::Node
{
  struct InternalData;

public:
  explicit AthenaMotorDriver( const rclcpp::NodeOptions &options );

  ~AthenaMotorDriver() override;

private:
  void setupSerial();

  void closeSerial();

  void setupController( const std::string &controller_type );

  void onReboot( const std_srvs::srv::Trigger::Request::SharedPtr request,
                 const std_srvs::srv::Trigger::Response::SharedPtr response );

  void update();

  void updateSettings();

  void declareMicroControllerParameters();

  rclcpp::SubscriptionBase::SharedPtr twist_sub_;
  rclcpp::SubscriptionBase::SharedPtr torque_sub_;
  rclcpp::Publisher<athena_motor_interface::msg::FullMotorStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<athena_motor_interface::msg::DebugData>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_service_;
  athena_motor_interface::msg::FullMotorStatus status_msg_;

  std::string controller_type_ = "diff_drive";
  ControllerBase::SharedPtr controller_;

  geometry_msgs::msg::Twist::ConstSharedPtr twist_msg_;
  MotorCommand last_motor_command_;
  rclcpp::Time last_motor_command_received_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<InternalData> data_;
  // Serial communication
  std::chrono::steady_clock::time_point last_setup_attempt_;
  std::unique_ptr<LibSerial::SerialPort> serial_;
  using CrossTalker = crosstalk::CrossTalker<4096, 2048>;
  std::unique_ptr<CrossTalker> cross_talker_;
  std::string port_name_ = "/dev/tty_drive_motor_controller";
  int baud_rate_ = 115200;
  double wheel_radius_ = 0.07;
  double angular_velocity_limit_ = 0.06;
  PIDGains left_velocity_pid_gains_;
  PIDGains right_velocity_pid_gains_;
  PIDGains left_position_pid_gains_;
  PIDGains right_position_pid_gains_;
  // Feed-forward control parameters for velocity control
  float left_velocity_feed_forward_gain_ = 0.0f;
  float left_velocity_feed_forward_offset_ = 0.0f;
  float right_velocity_feed_forward_gain_ = 0.0f;
  float right_velocity_feed_forward_offset_ = 0.0f;
  float left_velocity_feed_forward_k_s_rotational_ = 0.0f;
  float right_velocity_feed_forward_k_s_rotational_ = 0.0f;
  bool pid_updated_ = false;
  bool debug_ = false;
  bool disable_acceleration_limiting_ = false;
  double max_track_acceleration_rad_s2_ = 6.0;
  double max_track_deceleration_rad_s2_ = 16.0;
  double max_track_jerk_rad_s3_ = 120.0;
  double derivative_filter_cutoff_hz_ = 0.0;
  bool torque_mode_ = false;
  bool is_moving_ = false;
  bool invert_forward_direction_ = false;
};

} // namespace athena_motor_driver

#endif // ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H
