import rospy
from std_msgs.msg import Float32
import sensor_config as sens_cfg
class IMU_Model():
    def __init__(self):
        self.data = {
            "ACC_X": None,
            "ACC_Y": None,
            "ACC_Z": None,
            "GYRO_X": None,
            "GYRO_Y": None,
            "GYRO_Z": None,
            "MAG_X": None,
            "MAG_Y": None,
            "MAG_Z": None,
            "POSE_PITCH": None,
            "POSE_HEADING": None,
            "POSE_ROLL": None
        }
        
    def IMU_loop(self):
        rospy.Subscriber(sens_cfg.ACC_X, Float32, self.ACC_X_callback)
        rospy.Subscriber(sens_cfg.ACC_Y, Float32, self.ACC_Y_callback)
        rospy.Subscriber(sens_cfg.ACC_Z, Float32, self.ACC_Z_callback)
        rospy.Subscriber(sens_cfg.GYRO_X, Float32, self.GYRO_X_callback)
        rospy.Subscriber(sens_cfg.GYRO_Y, Float32, self.GYRO_Y_callback)
        rospy.Subscriber(sens_cfg.GYRO_Z, Float32, self.GYRO_Z_callback)
        rospy.Subscriber(sens_cfg.MAG_X, Float32, self.MAG_X_callback)
        rospy.Subscriber(sens_cfg.MAG_Y, Float32, self.MAG_Y_callback)
        rospy.Subscriber(sens_cfg.MAG_Z, Float32, self.MAG_Z_callback)
        rospy.Subscriber(sens_cfg.POSE_PITCH, Float32, self.POSE_PITCH_callback)
        rospy.Subscriber(sens_cfg.POSE_HEADING, Float32, self.POSE_HEADING_callback)
        rospy.Subscriber(sens_cfg.POSE_ROLL, Float32, self.POSE_ROLL_callback)
        rospy.spin()

    def ACC_X_callback(self, data):
        self.data["ACC_X"] = data

    def ACC_Y_callback(self, data):
        self.data["ACC_Y"] = data

    def ACC_Z_callback(self, data):
        self.data["ACC_Z"] = data

    def GYRO_X_callback(self, data):
        self.data["GYRO_X"] = data

    def GYRO_Y_callback(self, data):
        self.data["GYRO_Y"] = data

    def GYRO_Z_callback(self, data):
        self.data["GYRO_Z"] = data

    def MAG_X_callback(self, data):
        self.data["MAG_X"] = data

    def MAG_Y_callback(self, data):
        self.data["MAG_Y"] = data

    def MAG_Z_callback(self, data):
        self.data["MAG_Z"] = data

    def POSE_PITCH_callback(self, data):
        self.data["POSE_PITCH"] = data
    
    def POSE_HEADING_callback(self, data):
        self.data["POSE_HEADING"] = data
        
    def POSE_ROLL_callback(self, data):
        self.data["POSE_ROLL"] = data
