import rospy
from sensor_msgs.msg import LaserScan
import sensor_config as sens_cfg
class Lidar_Model():
    def __init__(self):
        self.data = {
            "S1": None,
            "A3": None
        }
        

    def lidar_loop(self):
        rospy.Subscriber(sens_cfg.A3, LaserScan, self.A3_callback)
        rospy.Subscriber(sens_cfg.S1, LaserScan, self.S1_callback)
        rospy.spin()

    def S1_callback(self, data):
        self.data["S1"] = data

    def A3_callback(self, data):
        self.data["A3"] = data
        

