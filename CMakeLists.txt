cmake_minimum_required(VERSION 3.5)
project(mr76_radar_ros2_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

add_definitions(-D "USE_ROS2LOGGER") # 添加宏定义
message("-- #define USE_ROS2LOGGER")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(radar_msgs REQUIRED)


#Bin and Install
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/serial2can
  ${CMAKE_CURRENT_SOURCE_DIR}/mr76_radar
)

set(SERIAL2CAN_SOURCE
  ${CMAKE_CURRENT_SOURCE_DIR}/serial2can/log_module.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/serial2can/serial_interface_linux.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/serial2can/serial2can_interface.cpp
)

# set(MR76_RADAR_SOURCE
#   ${CMAKE_CURRENT_SOURCE_DIR}/mr76_radar/xxx.cpp
# )

# file(GLOB  SRC_C ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
# file(GLOB  SRC_CPP ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(serial2can_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/serial2can_node.cpp
  ${SERIAL2CAN_SOURCE}
)
ament_target_dependencies(serial2can_node rclcpp can_msgs)
target_link_libraries(serial2can_node pthread)

add_executable(${PROJECT_NAME}_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mr76_node.cpp
  # ${MR76_RADAR_SOURCE}
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp can_msgs radar_msgs)

# Install
install(TARGETS serial2can_node ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
