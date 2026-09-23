cmake_minimum_required(VERSION 3.8)
project(mission_database)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#==============================================================================
# FIND DEPENDENCIES
#==============================================================================

find_package(ament_cmake              REQUIRED)
find_package(rclcpp                   REQUIRED)
find_package(sensor_msgs              REQUIRED)
find_package(std_msgs                 REQUIRED)
find_package(west_point_comms_sim     REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(SQLite3                  REQUIRED)

#==============================================================================
# GENERATE INTERFACES  (service + message)
#==============================================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryBreadcrumbs.srv"
  "srv/GetRetracePath.srv"
  "msg/WaypointEvent.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

#==============================================================================
# MISSION DATABASE NODE
#==============================================================================

add_executable(mission_database_node
  src/mission_database_node.cpp
  src/retrace_service.cpp
)

target_include_directories(mission_database_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(mission_database_node
  rclcpp
  sensor_msgs
  std_msgs
  west_point_comms_sim
)

target_link_libraries(mission_database_node
  "${cpp_typesupport_target}"
  SQLite::SQLite3
)

#==============================================================================
# INSTALL
#==============================================================================

install(TARGETS mission_database_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/db_tak_replay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
