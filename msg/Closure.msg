# Custom message for tracking loop closure events as they are detected
#
# cur_id is the frame id of the most recent StereoFrame that queried the db
# ref_id is the frame id of the StereoFrame with which cur_id frame matched
# cur_time is the timestamp of the recent StereoFrame
# ref_time is the timestamp of the matched StereoFrame
# transform is the relative 3D transformation estimate between the two frames
# NOTE: this transform is relative and therefore has no 'frame_id' or
# 'child_frame_id'.
# Transform is from ref frame to cur frame

uint32 cur_id
uint32 ref_id
time cur_time
time ref_time
geometry_msgs/Transform transform
