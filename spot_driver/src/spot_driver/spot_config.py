"""List of image sources for front image periodic query"""
front_image_sources = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "frontleft_depth",
    "frontright_depth",
]

"""List of image sources for side image periodic query"""
side_image_sources = [
    "left_fisheye_image",
    "right_fisheye_image",
    "left_depth",
    "right_depth",
]

"""List of image sources for rear image periodic query"""
rear_image_sources = ["back_fisheye_image", "back_depth"]

"""Service name for getting pointcloud of VLP16 connected to Spot Core"""
VELODYNE_SERVICE_NAME = "velodyne-point-cloud"

"""List of point cloud sources"""
point_cloud_sources = ["velodyne-point-cloud"]

"""List of image sources for hand image periodic query"""
hand_image_sources = [
    "hand_image",
    "hand_depth",
    "hand_color_image",
    "hand_depth_in_hand_color_frame",
]
