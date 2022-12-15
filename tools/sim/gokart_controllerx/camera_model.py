import rospy
from sensor_msgs.msg import CameraInfo
import sensor_config as sens_cfg
class Camera_Model():
    def __init__(self):
        self.data = {
            "CAMERA_INFO": None,
            "CAMERA_IMAGE_RAW": None,
            "CAMERA_IMAGE_RAW_COMPRESSED": None
        }

    def camera_loop(self):
        rospy.Subscriber(sens_cfg.CAMERA_INFO, CameraInfo, self.CAMERA_INFO_callback)
        rospy.Subscriber(sens_cfg.CAMERA_IMAGE_RAW, CameraInfo, self.CAMERA_IMAGE_RAW_callback)
        rospy.Subscriber(sens_cfg.CAMERA_IMAGE_RAW_COMPRESSED, CameraInfo, self.CAMERA_IMAGE_RAW_COMPRESSED_callback)
        rospy.spin()

    def CAMERA_INFO_callback(self, data):
        self.data["CAMERA_INFO"] = data

    def CAMERA_IMAGE_RAW_callback(self, data):
        self.data["SCAMERA_IMAGE_RAW"] = data

    def CAMERA_IMAGE_RAW_COMPRESSED_callback(self, data):
        self.data["CAMERA_IMAGE_RAW_COMPRESSED"] = data
