cmake_minimum_required(VERSION 3.16)
project(tower_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tower_nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------- 公共库 ----------
add_library(tower_nav_core
  src/tower_config.cpp
  src/tf_projection.cpp
  src/localization_source.cpp
  src/localization_filter.cpp
  src/trajectory_manager.cpp
  src/tracking_controller.cpp
  src/exception_monitor.cpp
  src/mission_state_machine.cpp
)
target_include_directories(tower_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tower_nav_core yaml-cpp)

# ---------- 节点 ----------
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node tower_nav_core)
ament_target_dependencies(localization_node
  rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

add_executable(trajectory_node src/trajectory_node.cpp)
target_link_libraries(trajectory_node tower_nav_core)
ament_target_dependencies(trajectory_node
  rclcpp geometry_msgs tf2 tf2_ros tower_nav_msgs)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node tower_nav_core)
ament_target_dependencies(controller_node
  rclcpp std_msgs geometry_msgs tower_nav_msgs)

add_executable(exception_node src/exception_node.cpp)
target_link_libraries(exception_node tower_nav_core)
ament_target_dependencies(exception_node
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tower_nav_msgs)

add_executable(mission_node src/mission_node.cpp)
target_link_libraries(mission_node tower_nav_core)
ament_target_dependencies(mission_node
  rclcpp std_msgs std_srvs tower_nav_msgs)

add_executable(status_node src/status_node.cpp)
ament_target_dependencies(status_node
  rclcpp std_msgs geometry_msgs tower_nav_msgs)

install(TARGETS
  localization_node trajectory_node controller_node
  exception_node mission_node status_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gen_trajectories.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS tower_nav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_tower_config test/test_tower_config.cpp)
  target_link_libraries(test_tower_config tower_nav_core)

  ament_add_gtest(test_tf_projection test/test_tf_projection.cpp)
  target_link_libraries(test_tf_projection tower_nav_core)

  ament_add_gtest(test_localization_source test/test_localization_source.cpp)
  target_link_libraries(test_localization_source tower_nav_core)

  ament_add_gtest(test_localization_filter test/test_localization_filter.cpp)
  target_link_libraries(test_localization_filter tower_nav_core)

  ament_add_gtest(test_trajectory_manager test/test_trajectory_manager.cpp)
  target_link_libraries(test_trajectory_manager tower_nav_core)

  ament_add_gtest(test_tracking_controller test/test_tracking_controller.cpp)
  target_link_libraries(test_tracking_controller tower_nav_core)

  ament_add_gtest(test_exception_monitor test/test_exception_monitor.cpp)
  target_link_libraries(test_exception_monitor tower_nav_core)

  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine tower_nav_core)

  # 集成测试:sim + 全栈闭环(耗时较长,超时放宽)
  add_launch_test(test/integration/test_mission_flow.py
    TIMEOUT 180
  )
endif()

ament_package()
