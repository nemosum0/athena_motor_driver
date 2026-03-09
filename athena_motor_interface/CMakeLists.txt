cmake_minimum_required(VERSION 3.8)
project(athena_motor_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DebugData.msg
  msg/FullMotorStatus.msg
  msg/LadrcDebugData.msg
  msg/MotorStatus.msg
  msg/MotorStatusDebugData.msg
  msg/TorqueCommand.msg
  DEPENDENCIES
  builtin_interfaces
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
