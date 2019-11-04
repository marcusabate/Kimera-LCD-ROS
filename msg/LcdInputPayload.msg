# Message containing necessary members of a 
# VIO::LoopClosureDetectorInputPayload payload. 

std_msgs/Header header
uint64 timestamp_kf # TODO(marcus): do we need this _and_ header?
uint32 cur_kf_id
kimera_lcd_ros/StereoFrame stereo_frame
geometry_msgs/Pose w_Pose_b_lkf 