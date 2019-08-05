cmake_minimum_required(VERSION 2.8.3)
project(lcd_ros)
add_compile_options(-std=c++14)

# TODO: change name of repo to lcd_ros
# TODO: figure out why generate_messages() isn't needed for catkin_simple

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_executable(lcd_ros_node
  src/LCD_ros_node.cpp
  src/LCD_ros.cpp
  src/StereoBuffer.cpp
)

target_link_libraries(lcd_ros_node SparkVio)

cs_install()
cs_export()
