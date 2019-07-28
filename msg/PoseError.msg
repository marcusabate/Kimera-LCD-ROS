# Custom message type representing error in relative pose recovery
#
# rot_error is in radians, trans_error is in meters

int32 ref_id
int32 cur_id
bool to_scale

float32 rot_error
float32 trans_error
