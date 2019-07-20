cmake_minimum_required(VERSION 2.8.3)
project(lcd_ros)
add_compile_options(-std=c++14)

# TODO: change name of repo to lcd_ros
# TODO: add REQUIRED spark_vio

find_package(catkin_simple REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  roscpp
  tf2_ros
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  PoseError.msg
)

catkin_simple()

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

include_directories(include)

catkin_package(
  INCLUDE_DIRS include
)

cs_add_executable(stereo_lcd_ros
  src/LCD_ros_node.cpp
  src/LCD_ros.cpp
  src/StereoBuffer.cpp
)

target_link_libraries(stereo_lcd_ros SparkVio)

cs_install()
cs_export()
