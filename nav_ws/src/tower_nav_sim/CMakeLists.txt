cmake_minimum_required(VERSION 3.16)
project(tower_nav_sim)

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
find_package(tower_nav_msgs REQUIRED)

# 仿真核心库(纯逻辑,可单测)
add_library(diff_drive_sim_core src/diff_drive_sim.cpp)
target_include_directories(diff_drive_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 仿真节点
add_executable(sim_node src/sim_node.cpp)
target_link_libraries(sim_node diff_drive_sim_core)
ament_target_dependencies(sim_node
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS diff_drive_sim_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_diff_drive_sim test/test_diff_drive_sim.cpp)
  target_link_libraries(test_diff_drive_sim diff_drive_sim_core)
endif()

ament_package()
