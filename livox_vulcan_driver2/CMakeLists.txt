cmake_minimum_required(VERSION 3.8)
project(livox_vulcan_driver2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(livox_ros_msg REQUIRED)

add_subdirectory("include/Fusion")

find_library(LIVOX_LIDAR_SDK_LIBRARY livox_lidar_sdk_shared REQUIRED)
find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
    src/livox_vulcan_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${LIVOX_LIDAR_SDK_LIBRARY}
  livox_ros_msg::livox_ros_msg__rosidl_typesupport_cpp
  Fusion
)

ament_auto_add_executable(${PROJECT_NAME}_node
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
