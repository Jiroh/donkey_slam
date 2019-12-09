# PATHS
CALIBRATION_DATA = "/home/jetson/catkin_ws/src/donkey_slam/calibration_data"
POINT_DATA_OUTPUT = "/home/jetson/data/point_data_test"
MONO_IMAGE_OUTPUT = "/home/jetson/data/mono_image_output"

# MISC
QUEUE_SIZE = 10
HEADER_ID = "base_link" # needs to be "/camera_link" for monocular calibration


# CAMERA
CAMERA_NAMESPACE = "/stereo_cam"
LEFT_CAMERA_NAME = "/left"
RIGHT_CAMERA_NAME = "/right"
LEFT_CAM = CAMERA_NAMESPACE + LEFT_CAMERA_NAME
RIGHT_CAM = CAMERA_NAMESPACE + RIGHT_CAMERA_NAME