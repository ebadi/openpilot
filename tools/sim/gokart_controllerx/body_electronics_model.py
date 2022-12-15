import rospy
from std_msgs.msg import Float32
import sensor_config as sens_cfg
class Body_Electronics_Model():
    def __init__(self):
        self.data = {
            "BLINK": None,
            "BREAKLIGHT":None,
            "HEADLIGHT": None,
            "HONK": None 
        }
       

    def body_electronics_loop(self):
        rospy.Subscriber(sens_cfg.BLINK, Float32, self.BLINK_callback)
        rospy.Subscriber(sens_cfg.BREAKLIGHT, Float32, self.BREAKLIGHT_callback)
        rospy.Subscriber(sens_cfg.HEADLIGHT, Float32, self.HEADLIGHT_callback)
        rospy.Subscriber(sens_cfg.HONK, Float32, self.HONK_callback)
        rospy.spin()

    def BLINK_callback(self, data):
        self.data["BLINK"] = data

    def BREAKLIGHT_callback(self, data):
        self.data["BREAKLIGHT"] = data

    def HEADLIGHT_callback(self, data):
        self.data["HEADLIGHT"] = data

    def HONK_callback(self, data):
        self.data["HONK"] = data