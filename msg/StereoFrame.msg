# Message containing a full VIO::StereoFrame object.

std_msgs/Header header
uint64 id
uint64 timestamp
sensor_msgs/Image left_image
kimera_lcd_ros/CameraParams left_camera_params
sensor_msgs/Image right_image
kimera_lcd_ros/CameraParams right_camera_params
geometry_msgs/Pose L_Pose_R
kimera_lcd_ros/StereoMatchingParams stereo_matching_params