import rospy
from std_msgs.msg import Float32
import sensor_config as sens_cfg
class Propulsion_Model():
    def __init__(self):
        self.data = {
            "SPEED": None,
            "TURN": None,
            "WHEEL_SPEED_FL": None,
            "WHEEL_SPEED_FR": None
        }
        

    def propulsion_loop(self):
        rospy.Subscriber(sens_cfg.SPEED, Float32, self.SPEED_callback)
        rospy.Subscriber(sens_cfg.TURN, Float32, self.TURN_callback)
        rospy.Subscriber(sens_cfg.WHEEL_SPEED_FL, Float32, self.WHEEL_SPEED_FL_callback)
        rospy.Subscriber(sens_cfg.WHEEL_SPEED_FR, Float32, self.WHEEL_SPEED_FR_callback)
        rospy.spin()

    def SPEED_callback(self, data):
        self.data["SPEED"] = data

    def TURN_callback(self, data):
        self.data["TURN"] = data

    def WHEEL_SPEED_FL_callback(self, data):
        self.data["WHEEL_SPEED_FL"] = data

    def WHEEL_SPEED_FR_callback(self, data):
        self.data["WHEEL_SPEED_FR"] = data