#include "athena_motor_driver/athena_motor_driver.hpp"
#include "crosstalk_lib_serial_wrapper.hpp"
#include "message_conversions.hpp"

#include "athena_motor_driver/controllers/diff_drive_controller.hpp"
#include <algorithm>
#include <athena_motor_interface/athena_motor_interfaces.h>
#include <athena_motor_interface/msg/torque_command.hpp>

using namespace std::chrono_literals;

namespace athena_motor_driver
{

struct AthenaMotorDriver::InternalData {
  FullMotorStatus status;
};

AthenaMotorDriver::AthenaMotorDriver( const rclcpp::NodeOptions &options )
    : Node( "athena_motor_driver", rclcpp::NodeOptions( options ).enable_logger_service( true ) ),
      data_( std::make_unique<InternalData>() )
{
  // Declare serial parameters
  declare_readonly_parameter( "port_name", port_name_, "Serial port name" );
  declare_readonly_parameter( "baud_rate", baud_rate_, "Serial baud rate" );
  declare_reconfigurable_parameter( "angular_velocity_limit", std::ref( angular_velocity_limit_ ),
                                    "Limit for commanded angular velocity (rad/s)",
                                    hector::ParameterOptions<double>().setRange( 0.0, 2.0, 0.01 ) );
  declare_reconfigurable_parameter(
      "controller", std::ref( controller_type_ ), "Controller type",
      hector::ParameterOptions<std::string>()
          .setAdditionalConstraints( "Allowed values: diff_drive" )
          .onValidate( []( const auto &value ) { return value == "diff_drive"; } )
          .onUpdate( [this]( const std::string &value ) { setupController( value ); } ) );
  declareMicroControllerParameters();

  setupController( controller_type_ );
  setupSerial();
  timer_ = create_wall_timer( 20ms, [this]() { update(); } );
  last_motor_command_received_ = now();
  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, [this]( const geometry_msgs::msg::Twist::ConstSharedPtr &msg ) {
        if ( torque_mode_ )
          return;
        twist_msg_ = msg;
        last_motor_command_received_ = now();
      } );
  torque_sub_ = create_subscription<athena_motor_interface::msg::TorqueCommand>(
      "~/forward_torque", 1,
      [this]( const athena_motor_interface::msg::TorqueCommand::ConstSharedPtr &msg ) {
        if ( !torque_mode_ )
          return;
        last_motor_command_ = MotorCommand::Torque( msg->left, msg->right );
        last_motor_command_received_ = now();
      } );
  status_pub_ = create_publisher<athena_motor_interface::msg::FullMotorStatus>( "motor_status", 1 );
  if ( debug_ ) {
    debug_pub_ = create_publisher<athena_motor_interface::msg::DebugData>( "~/debug_data", 1 );
  }
  reboot_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reboot", [this]( const std_srvs::srv::Trigger::Request::SharedPtr request,
                          const std_srvs::srv::Trigger::Response::SharedPtr response ) {
        onReboot( request, response );
      } );
}

AthenaMotorDriver::~AthenaMotorDriver() { closeSerial(); }

void AthenaMotorDriver::setupSerial()
{
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  if ( now - last_setup_attempt_ < 5s ) {
    // Only reattempt to set up the serial port 5 seconds
    return;
  }
  last_setup_attempt_ = now;
  try {
    serial_ = std::make_unique<LibSerial::SerialPort>();
    if ( serial_->Open( port_name_ ); !serial_->IsOpen() ) {
      RCLCPP_ERROR( get_logger(), "Failed to open serial port." );
      return;
    }
    serial_->SetBaudRate( LibSerial::BaudRate::BAUD_1152000 );
    RCLCPP_INFO( get_logger(), "Serial port '%s' configured with baud rate %d", port_name_.c_str(),
                 baud_rate_ );
    cross_talker_ =
        std::make_unique<CrossTalker>( std::make_unique<crosstalk::LibSerialWrapper>( *serial_ ) );
    pid_updated_ = true; // Force PID update on first run
    updateSettings();
  } catch ( const std::exception &e ) {
    RCLCPP_ERROR( get_logger(), "Failed to initialize serial port: %s", e.what() );
    serial_.reset();
    cross_talker_.reset();
  }
}

void AthenaMotorDriver::closeSerial() { serial_->Close(); }

namespace
{
void updateStatus( athena_motor_interface::msg::MotorStatus &msg, const MotorStatus &status,
                   const rclcpp::Time &timestamp )
{
  if ( !status.valid || status.age_ms == UINT32_MAX )
    return;
  msg.valid = status.valid;
  msg.mode = static_cast<uint8_t>( status.mode );
  msg.error_code = static_cast<uint8_t>( status.error );
  msg.temperature = status.temperature;
  msg.target_velocity = status.target_velocity;
  msg.target_torque = status.target_torque;
  msg.torque = status.torque;
  msg.velocity_high = status.velocity_high;
  msg.velocity_low = status.velocity_low;
  msg.position = status.position;
  msg.acceleration = status.acceleration;
  msg.stamp = timestamp - rclcpp::Duration( std::chrono::milliseconds( status.age_ms ) );
}
} // namespace

void AthenaMotorDriver::update()
{
  if ( !serial_ || !serial_->IsOpen() ) {
    setupSerial();
    return;
  }
  try {
    crosstalk::WriteResult result = crosstalk::WriteResult::Success;
    if ( last_motor_command_received_ + 250ms < now() ) {
      if ( is_moving_ ) {
        RCLCPP_WARN( get_logger(), "No command in 250ms. Sending 0 command." );
        is_moving_ = false;
      }
      MotorCommand command =
          torque_mode_ ? MotorCommand::Torque( 0, 0 ) : MotorCommand::Velocity( 0, 0 );
      result = cross_talker_->sendObject( command );
    } else if ( torque_mode_ ) {
      is_moving_ = true;
      RCLCPP_DEBUG( get_logger(), "Sending torques: %f, %f", last_motor_command_.left,
                    last_motor_command_.right );
      result = cross_talker_->sendObject( last_motor_command_ );
    } else if ( twist_msg_ ) {
      is_moving_ = true;
      const double direction_sign = invert_forward_direction_ ? -1.0 : 1.0;
      double angular = twist_msg_->angular.z;
      double linear = twist_msg_->linear.x;
      if ( std::abs( angular ) > angular_velocity_limit_ ) {
        RCLCPP_DEBUG( get_logger(), "Commanded angular velocity exceeds limit." );
        angular = std::copysign( angular_velocity_limit_, angular );
        linear *= angular / twist_msg_->angular.z; // Scale linear velocity to maintain curvature
      }
      MotorCommand command = controller_->computeMotorCommand( direction_sign * linear, angular );
      RCLCPP_DEBUG( get_logger(), "Sending velocities: %f, %f", command.left, command.right );
      result = cross_talker_->sendObject( command );
    }
    if ( result != crosstalk::WriteResult::Success ) {
      RCLCPP_ERROR_STREAM( get_logger(), "Failed to send MotorCommand object. Error:"
                                             << crosstalk::to_string( result ) );
    }

    if ( pid_updated_ ) {
      pid_updated_ = false;
      UpdatePIDParamsCommand command( left_velocity_pid_gains_, right_velocity_pid_gains_,
                                      left_position_pid_gains_, right_position_pid_gains_ );
      command.left_velocity_startup_gain = left_velocity_startup_gain_;
      command.left_velocity_startup_offset = left_velocity_startup_offset_;
      command.right_velocity_startup_gain = right_velocity_startup_gain_;
      command.right_velocity_startup_offset = right_velocity_startup_offset_;
      auto result = cross_talker_->sendObject( command );
      if ( result == crosstalk::WriteResult::Success ) {
        RCLCPP_INFO( get_logger(), "Sending request to update PID Gains." );
        RCLCPP_INFO( get_logger(),
                     "Velocity:\n  Left: k_p=%f, K_i=%f, k_d=%f\n  Right: k_p=%f, K_i=%f, k_d=%f",
                     left_velocity_pid_gains_.k_p, left_velocity_pid_gains_.k_i,
                     left_velocity_pid_gains_.k_d, right_velocity_pid_gains_.k_p,
                     right_velocity_pid_gains_.k_i, right_velocity_pid_gains_.k_d );
        RCLCPP_INFO( get_logger(),
                     "Position:\n  Left: k_p=%f, K_i=%f, k_d=%f\n  Right: k_p=%f, K_i=%f, k_d=%f",
                     left_position_pid_gains_.k_p, left_position_pid_gains_.k_i,
                     left_position_pid_gains_.k_d, right_position_pid_gains_.k_p,
                     right_position_pid_gains_.k_i, right_position_pid_gains_.k_d );
        RCLCPP_INFO( get_logger(),
                     "Velocity Startup:\n  Left: gain=%f, offset=%f\n  Right: gain=%f, offset=%f",
                     left_velocity_startup_gain_, left_velocity_startup_offset_,
                     right_velocity_startup_gain_, right_velocity_startup_offset_ );
        RCLCPP_INFO( get_logger(), "You should see 'PID Gains updated.' next, if it worked." );

      } else {
        RCLCPP_ERROR_STREAM( get_logger(), "Failed to send UpdatePIDParamsCommand object. Error:"
                                               << crosstalk::to_string( result ) );
      }
    }

    std::vector<uint8_t> buffer;
    cross_talker_->processSerialData();
    std::optional<FullMotorStatus> full_motor_status;
    while ( cross_talker_->available() || cross_talker_->hasObject() ) {
      if ( cross_talker_->available() > 0 ) {
        buffer.resize( cross_talker_->available() );
        size_t bytes_read = cross_talker_->read( buffer.data(), buffer.size() );
        RCLCPP_INFO_STREAM( get_logger(),
                            "Read " << bytes_read << " bytes of non-object data from serial port: "
                                    << std::string( (const char *)buffer.data(), bytes_read ) );
      }
      cross_talker_->processSerialData( false );

      if ( !cross_talker_->hasObject() )
        return;
      switch ( cross_talker_->getObjectId() ) {
      case crosstalk::object_id<FullMotorStatus>(): {
        FullMotorStatus status;
        auto result = cross_talker_->readObject( status );
        if ( result != crosstalk::ReadResult::Success ) {
          if ( result != crosstalk::ReadResult::NotEnoughData )
            RCLCPP_ERROR_STREAM( get_logger(), "Failed to read FullMotorStatus object: "
                                                   << crosstalk::to_string( result ) );
          continue;
        }
        if ( full_motor_status ) {
          // If we already have a full status, merge the new one and overwrite valid fields
          if ( status.front_left.valid )
            full_motor_status->front_left = status.front_left;
          if ( status.front_right.valid )
            full_motor_status->front_right = status.front_right;
          if ( status.rear_left.valid )
            full_motor_status->rear_left = status.rear_left;
          if ( status.rear_right.valid )
            full_motor_status->rear_right = status.rear_right;
        } else {
          full_motor_status = status;
        }
        break;
      }
      case crosstalk::object_id<MotorError>(): {
        MotorError error;
        auto result = cross_talker_->readObject( error );
        if ( result != crosstalk::ReadResult::Success ) {
          if ( result != crosstalk::ReadResult::NotEnoughData )
            RCLCPP_ERROR_STREAM( get_logger(), "Failed to read MotorError object: "
                                                   << crosstalk::to_string( result ) );
          continue;
        }
        RCLCPP_ERROR( get_logger(), "Motor error: %d", static_cast<int>( error.error ) );
        break;
      }
      case crosstalk::object_id<AckCommand>(): {
        AckCommand ack;
        auto result = cross_talker_->readObject( ack );
        if ( result != crosstalk::ReadResult::Success ) {
          if ( result != crosstalk::ReadResult::NotEnoughData )
            RCLCPP_ERROR_STREAM( get_logger(), "Failed to read AckCommand object: "
                                                   << crosstalk::to_string( result ) );
          continue;
        }
        if ( ack.type == CommandType::CHANGE_PID_GAINS ) {
          RCLCPP_INFO( get_logger(), "PID gains updated." );
        } else if ( ack.type == CommandType::UPDATE_SETTINGS ) {
          RCLCPP_INFO( get_logger(), "Settings updated." );
        } else if ( ack.type == CommandType::TEENSY_REBOOT ) {
          RCLCPP_INFO( get_logger(), "Teensy reboot command acknowledged." );
          closeSerial();
        } else if ( ack.type != CommandType::MOTOR_COMMAND ) {
          RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000,
                                "Received ACK for unknown command type: %d",
                                static_cast<int>( ack.type ) );
        }
        break;
      }
      case crosstalk::object_id<MotorDebugData>(): {
        MotorDebugData debug_data;
        auto result = cross_talker_->readObject( debug_data );
        if ( result != crosstalk::ReadResult::Success ) {
          if ( result != crosstalk::ReadResult::NotEnoughData )
            RCLCPP_ERROR_STREAM( get_logger(), "Failed to read MotorDebugData object: "
                                                   << crosstalk::to_string( result ) );
          continue;
        }
        athena_motor_interface::msg::DebugData msg = toMsg( debug_data );
        if ( debug_pub_ )
          debug_pub_->publish( msg );
        break;
      }
      default:
        RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000,
                              "Received unknown object with ID: %d", cross_talker_->getObjectId() );
        cross_talker_->skipObject();
        break;
      }
    }
    if ( full_motor_status ) {
      const FullMotorStatus &status = *full_motor_status;
      rclcpp::Time stamp = this->now();
      updateStatus( status_msg_.front_left, status.front_left, stamp );
      updateStatus( status_msg_.front_right, status.front_right, stamp );
      updateStatus( status_msg_.rear_left, status.rear_left, stamp );
      updateStatus( status_msg_.rear_right, status.rear_right, stamp );
      if ( status.front_left.valid || status.front_right.valid || status.rear_left.valid ||
           status.rear_right.valid ) {
        status_pub_->publish( status_msg_ );
      }
    }
    return;
  } catch ( std::exception &e ) {
    RCLCPP_ERROR( get_logger(), "Error reading from serial port: %s. Trying to reopen.", e.what() );
  }
  try {
    serial_.reset();
  } catch ( std::exception &e ) {
    RCLCPP_ERROR( get_logger(), "Error closing serial port: %s", e.what() );
  }
}

void AthenaMotorDriver::setupController( const std::string &controller_type )
{
  auto node = std::shared_ptr<rclcpp::Node>( this, []( const rclcpp::Node * ) {} );
  if ( controller_type == "diff_drive" ) {
    controller_ = std::make_shared<DiffDriveController>( node );
  }
}

void AthenaMotorDriver::updateSettings()
{
  if ( !cross_talker_ )
    return;

  UpdateSettings settings;
  settings.enable_debug = debug_;
  settings.disable_acceleration_limiting = disable_acceleration_limiting_;
  settings.max_track_acceleration_rad_s2 = static_cast<float>( max_track_acceleration_rad_s2_ );
  settings.max_track_deceleration_rad_s2 = static_cast<float>( max_track_deceleration_rad_s2_ );
  settings.max_track_jerk_rad_s3 = static_cast<float>( max_track_jerk_rad_s3_ );
  settings.derivative_filter_cutoff_hz = static_cast<float>( derivative_filter_cutoff_hz_ );
  cross_talker_->sendObject( settings );
}

void AthenaMotorDriver::onReboot( const std_srvs::srv::Trigger::Request::SharedPtr,
                                  const std_srvs::srv::Trigger::Response::SharedPtr response )
{
  RCLCPP_INFO( get_logger(), "Rebooting motor controller..." );
  cross_talker_->sendObject( TeensyRebootCommand{} );
  response->success = true;
}

void AthenaMotorDriver::declareMicroControllerParameters()
{
  declare_reconfigurable_parameter(
      "enable_debug", std::ref( debug_ ), "Enable debug data collection and publishing",
      hector::ParameterOptions<bool>().onUpdate( [this]( const auto &value ) {
        RCLCPP_INFO( get_logger(), "Setting Debug mode to: %s", value ? "enabled" : "disabled" );
        if ( value && !debug_pub_ ) {
          debug_pub_ = create_publisher<athena_motor_interface::msg::DebugData>( "~/debug_data", 1 );
        }
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "disable_acceleration_limiting", std::ref( disable_acceleration_limiting_ ),
      "Only for PID tuning. NEVER check for normal operation!",
      hector::ParameterOptions<bool>().onUpdate( [this]( const auto &value ) {
        RCLCPP_INFO( get_logger(), "Setting acceleration limiting to: %s",
                     value ? "disabled" : "enabled" );
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "max_track_acceleration_rad_s2", std::ref( max_track_acceleration_rad_s2_ ),
      "Firmware VELOCITY ramp: max track drive acceleration reference (rad/s²)",
      hector::ParameterOptions<double>().setRange( 0.1, 200.0, 0.1 ).onUpdate( [this]( const auto & ) {
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "max_track_deceleration_rad_s2", std::ref( max_track_deceleration_rad_s2_ ),
      "Firmware VELOCITY ramp: max track drive deceleration reference magnitude (rad/s²)",
      hector::ParameterOptions<double>().setRange( 0.1, 200.0, 0.1 ).onUpdate( [this]( const auto & ) {
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "max_track_jerk_rad_s3",
      std::ref( max_track_jerk_rad_s3_ ), "Firmware VELOCITY ramp: max track drive jerk magnitude on velocity reference (rad/s³); 0 disables",
      hector::ParameterOptions<double>().setRange( 0.0, 5000.0, 1.0 ).onUpdate( [this]( const auto & ) {
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "derivative_filter_cutoff_hz", std::ref( derivative_filter_cutoff_hz_ ),
      "Derivative low-pass filter cutoff frequency in Hz. 0 = no filtering.",
      hector::ParameterOptions<double>().setRange( 0.0, 1000.0, 1.0 ).onUpdate( [this]( const auto & ) {
        updateSettings();
      } ) );
  declare_reconfigurable_parameter(
      "enable_torque_mode", std::ref( torque_mode_ ),
      "Use torque commands instead of velocity commands (ignores cmd_vel topic when active)" );
  declare_reconfigurable_parameter( "invert_forward_direction", std::ref( invert_forward_direction_ ),
                                    "Invert the forward direction of the robot" );
  hector::ParameterOptions<float> pid_options =
      hector::ParameterOptions<float>()
          .onValidate( []( const auto &value ) { return value >= 0; } )
          .onUpdate( [this]( const auto & ) { pid_updated_ = true; } );
  declare_reconfigurable_parameter( "left_velocity_pid.k_p", std::ref( left_velocity_pid_gains_.k_p ),
                                    "Left P-Gain", pid_options );
  declare_reconfigurable_parameter( "left_velocity_pid.k_i", std::ref( left_velocity_pid_gains_.k_i ),
                                    "Left I-Gain", pid_options );
  declare_reconfigurable_parameter( "left_velocity_pid.k_d", std::ref( left_velocity_pid_gains_.k_d ),
                                    "Left D-Gain", pid_options );
  declare_reconfigurable_parameter( "right_velocity_pid.k_p",
                                    std::ref( right_velocity_pid_gains_.k_p ), "Right P-Gain",
                                    pid_options );
  declare_reconfigurable_parameter( "right_velocity_pid.k_i",
                                    std::ref( right_velocity_pid_gains_.k_i ), "Right I-Gain",
                                    pid_options );
  declare_reconfigurable_parameter( "right_velocity_pid.k_d",
                                    std::ref( right_velocity_pid_gains_.k_d ), "Right D-Gain",
                                    pid_options );

  declare_reconfigurable_parameter( "left_position_pid.k_p", std::ref( left_position_pid_gains_.k_p ),
                                    "Left P-Gain", pid_options );
  declare_reconfigurable_parameter( "left_position_pid.k_i", std::ref( left_position_pid_gains_.k_i ),
                                    "Left I-Gain", pid_options );
  declare_reconfigurable_parameter( "left_position_pid.k_d", std::ref( left_position_pid_gains_.k_d ),
                                    "Left D-Gain", pid_options );
  declare_reconfigurable_parameter( "right_position_pid.k_p",
                                    std::ref( right_position_pid_gains_.k_p ), "Right P-Gain",
                                    pid_options );
  declare_reconfigurable_parameter( "right_position_pid.k_i",
                                    std::ref( right_position_pid_gains_.k_i ), "Right I-Gain",
                                    pid_options );
  declare_reconfigurable_parameter( "right_position_pid.k_d",
                                    std::ref( right_position_pid_gains_.k_d ), "Right D-Gain",
                                    pid_options );

  // Startup parameters for velocity control
  // Use the same pid_options which includes onUpdate callback that sets pid_updated_ = true
  declare_reconfigurable_parameter( "left_velocity_startup.gain",
                                    std::ref( left_velocity_startup_gain_ ),
                                    "Left velocity startup gain", pid_options );
  declare_reconfigurable_parameter( "left_velocity_startup.offset",
                                    std::ref( left_velocity_startup_offset_ ),
                                    "Left velocity startup offset", pid_options );
  declare_reconfigurable_parameter( "right_velocity_startup.gain",
                                    std::ref( right_velocity_startup_gain_ ),
                                    "Right velocity startup gain", pid_options );
  declare_reconfigurable_parameter( "right_velocity_startup.offset",
                                    std::ref( right_velocity_startup_offset_ ),
                                    "Right velocity startup offset", pid_options );
}

} // namespace athena_motor_driver

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( athena_motor_driver::AthenaMotorDriver )
