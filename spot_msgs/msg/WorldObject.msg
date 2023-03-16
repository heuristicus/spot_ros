int32 id
string name
time acquisition_time

### Frame tree
FrameTreeSnapshot frame_tree_snapshot

### AprilTag properties
AprilTagProperties apriltag_properties

### Image properties
ImageProperties image_properties

### Dock properties
# Dock type enum
uint8 DOCK_TYPE_UNKNOWN=0
uint8 DOCK_TYPE_CONTACT_PROTOTYPE=2
uint8 DOCK_TYPE_SPOT_DOCK=3

int32 dock_id
uint8 dock_type

string frame_name_dock
bool dock_unavailable
bool from_prior_detection

### Ray properties
string ray_frame
geometry_msgs/Vector3 ray_origin
geometry_msgs/Vector3 ray_direction

### Bounding box properties
string bounding_box_frame
geometry_msgs/Vector3 bounding_box_size_ewrt_frame
