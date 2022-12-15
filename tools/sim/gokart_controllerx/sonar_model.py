import rospy
from std_msgs.msg import Float32
import sensor_config as sens_cfg
class Sonar_Model():
    def __init__(self):
        self.data = {
            "SONAR_1": None,
            "SONAR_2": None,
            "SONAR_3": None,
            "SONAR_4": None,
            "SONAR_5": None,
            "SONAR_6": None
        }
 

    def sonar_loop(self):
        rospy.Subscriber(sens_cfg.SONAR_1, Float32, self.SONAR_1_callback)
        rospy.Subscriber(sens_cfg.SONAR_2, Float32, self.SONAR_2_callback)
        rospy.Subscriber(sens_cfg.SONAR_3, Float32, self.SONAR_3_callback)
        rospy.Subscriber(sens_cfg.SONAR_4, Float32, self.SONAR_4_callback)
        rospy.Subscriber(sens_cfg.SONAR_5, Float32, self.SONAR_5_callback)
        rospy.Subscriber(sens_cfg.SONAR_6, Float32, self.SONAR_6_callback)

        rospy.spin()

    def SONAR_1_callback(self, data):
        self.data["SONAR_1"] = data

    def SONAR_2_callback(self, data):
        self.data["SONAR_2"] = data

    def SONAR_3_callback(self, data):
        self.data["SONAR_3"] = data

    def SONAR_4_callback(self, data):
        self.data["SONAR_4"] = data

    def SONAR_5_callback(self, data):
        self.data["SONAR_5"] = data

    def SONAR_6_callback(self, data):
        self.data["SONAR_6"] = data
        

