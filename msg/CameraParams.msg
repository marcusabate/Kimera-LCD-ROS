# Message containing a full VIO::CameraParams object.

float64 frame_rate
uint64[] image_size

float64[] intrinsics
float64[] extrinsics

string distortion_model
float64[] distortion_coefficients

float64[] body_Pose_cam