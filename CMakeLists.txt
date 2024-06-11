cmake_minimum_required(VERSION 3.5)
project(nav_novamob)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)

add_library(nav2_open_gesture_detected_bt_node SHARED
  src/open_hand_condition.cpp
)

add_library(nav2_closed_gesture_detected_bt_node SHARED
 src/closed_hand_condition.cpp
)

add_library(nav2_alwaystrue_bt_node SHARED
 src/always_true_condition.cpp
)

add_library(nav2_stop_robot_bt_node SHARED
 src/stop_robot_action.cpp
)

add_library(nav2_resume_nav_bt_node SHARED
 src/resume_nav_node.cpp
)

add_executable(closed_hand_node src/ClosedHandNode.cpp)
ament_target_dependencies(closed_hand_node ${dependencies}) 


list(APPEND plugin_libs nav2_open_gesture_detected_bt_node)
list(APPEND plugin_libs nav2_closed_gesture_detected_bt_node)
list(APPEND plugin_libs nav2_alwaystrue_bt_node)
list(APPEND plugin_libs nav2_stop_robot_bt_node)
list(APPEND plugin_libs nav2_resume_nav_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# ament_target_dependencies(nav2_open_gesture_detected_bt_node
#   ${dependencies}
# )

install(TARGETS ${plugin_libs} # nav2_open_gesture_detected_bt_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS closed_hand_node
  DESTINATION lib/${PROJECT_NAME}) 

include_directories(include)

install(
  DIRECTORY behavior_trees config src utils
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  # nav2_open_gesture_detected_bt_node
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})
ament_package()
