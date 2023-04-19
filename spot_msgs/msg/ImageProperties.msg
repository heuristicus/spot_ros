string camera_source

# Polygon coordinates
geometry_msgs/Polygon image_data_coordinates

# Keypoint coordinates
uint8 KEYPOINT_UNKNOWN=0
uint8 KEYPOINT_SIMPLE=1
uint8 KEYPOINT_ORB=2

uint8 image_data_keypoint_type
int32[] keypoint_coordinate_x
int32[] keypoint_coordinate_y
string[] binary_descriptor
float64[] keypoint_score
float64[] keypoint_size
float64[] keypoint_angle

ImageSource image_source
ImageCapture image_capture

string frame_name_image_coordinates