cmake_minimum_required(VERSION 3.8)
project(omnicare_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  # MSG
  "msg/MotorsPWM.msg"
  "msg/MotorsData.msg"
  "msg/FloorDetectorInfo.msg"
  

  # SRV
  "srv/OnOffNode.srv"
  "srv/SwitchFloor.srv"
  "srv/Checkpoints.srv"

   # ACT
   "action/EnterElevator.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
