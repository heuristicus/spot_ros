# Image type enums
uint8 IMAGE_TYPE_UNKNOWN = 0
uint8 IMAGE_TYPE_VISUAL = 1
uint8 IMAGE_TYPE_DEPTH = 2

# Pixel format enums
uint8 PIXEL_FORMAT_UNKNOWN = 0
uint8 PIXEL_FORMAT_GREYSCALE_U8 = 1
uint8 PIXEL_FORMAT_RGB_U8 = 3
uint8 PIXEL_FORMAT_RGBA_U8 = 4
uint8 PIXEL_FORMAT_DEPTH_U16 = 5
uint8 PIXEL_FORMAT_GREYSCALE_U16 = 6

# Image format enums
uint8 FORMAT_UNKNOWN = 0
uint8 FORMAT_JPEG = 1
uint8 FORMAT_RAW = 2
uint8 FORMAT_RLE = 3

string name
int32 cols
int32 rows
float64 depth_scale

# Camera pinhole model parameters
float64 focal_length_x
float64 focal_length_y
float64 principal_point_x
float64 principal_point_y
float64 skew_x
float64 skew_y

uint8 image_type
uint8[] pixel_formats
uint8[] image_formats
