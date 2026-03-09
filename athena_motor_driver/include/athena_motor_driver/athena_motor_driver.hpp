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
  LadrcGains left_ladrc_gains_;
  LadrcGains right_ladrc_gains_;
  bool ladrc_updated_ = false;
  bool debug_ = false;
  bool disable_acceleration_limiting_ = false;
  bool torque_mode_ = false;
  bool is_moving_ = false;
  bool invert_forward_direction_ = false;
};

} // namespace athena_motor_driver

#endif // ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H
