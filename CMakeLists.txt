cmake_minimum_required(VERSION 2.8.3)
project(kimera_lcd_ros)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(kimera_lcd_ros_node
  src/kimera-lcd-ros-node.cpp
  src/lcd-wrapper.cpp
)
target_link_libraries(kimera_lcd_ros_node KimeraVIO)

cs_install()

cs_export()
