cmake_minimum_required(VERSION 3.8)
project(surveillance_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install resource directories so they are discoverable at
# share/surveillance_robot_description/ after building. These are empty in
# Chunk 1 and will be populated in later chunks.
install(
  DIRECTORY urdf meshes rviz config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
