# Status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK = 1
uint8 STATUS_AMBIGUOUS = 2
uint8 STATUS_HIGH_ERROR = 3

int32 tag_id
float64 x
float64 y

string frame_name_fiducial
uint8 fiducial_pose_status

string frame_name_fiducial_filtered
uint8 fiducial_filtered_pose_status

string frame_name_camera

geometry_msgs/PoseWithCovariance detection_covariance
string detection_covariance_reference_frame