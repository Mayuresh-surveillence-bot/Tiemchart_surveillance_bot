cmake_minimum_required(VERSION 3.8)
project(surveillance_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# NOTE (Chunk 1): No interfaces are defined yet, so nothing is generated.
# In a later chunk, add .msg/.srv/.action files under msg/, srv/, action/
# and register them here, for example:
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/PatrolStatus.msg"
#   "msg/SurveillanceEvent.msg"
#   "action/PatrolMission.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs
# )
#
# Remember to add the matching <depend> entries to package.xml.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
