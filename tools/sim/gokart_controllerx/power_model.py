import rospy
from std_msgs.msg import Float32
import sensor_config as sens_cfg
class Power_Model():
    def __init__(self):
        self.data = {
            "POWER_ENABLE_CHANNEL_1": None,
            "POWER_ENABLE_CHANNEL_2": None,
            "POWER_ENABLE_CHANNEL_3": None,
            "POWER_ENABLE_CHANNEL_4": None,
            "POWER_STATUS_CHANNEL_1": None,
            "POWER_STATUS_CHANNEL_2": None,
            "POWER_STATUS_CHANNEL_3": None,
            "POWER_STATUS_CHANNEL_4": None,
            "PROP_VOLTAGE_1": None,
            "PROP_VOLTAGE_2": None,
            "PROP_VOLTAGE_3": None,
            "PROP_VOLTAGE_4": None,
            "PROP_VOLTAGE_TOTAL": None,
            "MOTOR_CURRENT": None,
            "SYSTEM_VOLTAGE_1": None,
            "SYSTEM_VOLTAGE_2": None,
            "SYSTEM_VOLTAGE_TOTAL": None
        }
        

    def power_loop(self):
        rospy.Subscriber(sens_cfg.POWER_ENABLE_CHANNEL_1, Float32, self.POWER_ENABLE_CHANNEL_1_callback)
        rospy.Subscriber(sens_cfg.POWER_ENABLE_CHANNEL_2, Float32, self.POWER_ENABLE_CHANNEL_2_callback)
        rospy.Subscriber(sens_cfg.POWER_ENABLE_CHANNEL_3, Float32, self.POWER_ENABLE_CHANNEL_3_callback)
        rospy.Subscriber(sens_cfg.POWER_ENABLE_CHANNEL_4, Float32, self.POWER_ENABLE_CHANNEL_4_callback)
        rospy.Subscriber(sens_cfg.POWER_STATUS_CHANNEL_1, Float32, self.POWER_STATUS_CHANNEL_1_callback)
        rospy.Subscriber(sens_cfg.POWER_STATUS_CHANNEL_2, Float32, self.POWER_STATUS_CHANNEL_2_callback)
        rospy.Subscriber(sens_cfg.POWER_STATUS_CHANNEL_3, Float32, self.POWER_STATUS_CHANNEL_3_callback)
        rospy.Subscriber(sens_cfg.POWER_STATUS_CHANNEL_4, Float32, self.POWER_STATUS_CHANNEL_4_callback)
        rospy.Subscriber(sens_cfg.PROP_VOLTAGE_1, Float32, self.PROP_VOLTAGE_1_callback)
        rospy.Subscriber(sens_cfg.PROP_VOLTAGE_2, Float32, self.PROP_VOLTAGE_2_callback)
        rospy.Subscriber(sens_cfg.PROP_VOLTAGE_3, Float32, self.PROP_VOLTAGE_3_callback)
        rospy.Subscriber(sens_cfg.PROP_VOLTAGE_4, Float32, self.PROP_VOLTAGE_4_callback)
        rospy.Subscriber(sens_cfg.PROP_VOLTAGE_TOTAL, Float32, self.PROP_VOLTAGE_TOTAL_callback)
        rospy.Subscriber(sens_cfg.MOTOR_CURRENT, Float32, self.MOTOR_CURRENT_callback)
        rospy.Subscriber(sens_cfg.SYSTEM_VOLTAGE_1, Float32, self.SYSTEM_VOLTAGE_1_callback)
        rospy.Subscriber(sens_cfg.SYSTEM_VOLTAGE_2, Float32, self.SYSTEM_VOLTAGE_2_callback)
        rospy.Subscriber(sens_cfg.SYSTEM_VOLTAGE_TOTAL, Float32, self.SYSTEM_VOLTAGE_TOTAL_callback)
        rospy.spin()

    def POWER_ENABLE_CHANNEL_1_callback(self, data):
        self.data["POWER_ENABLE_CHANNEL_1"] = data

    def POWER_ENABLE_CHANNEL_2_callback(self, data):
        self.data["POWER_ENABLE_CHANNEL_2"] = data
    
    def POWER_ENABLE_CHANNEL_3_callback(self, data):
        self.data["POWER_ENABLE_CHANNEL_3"] = data

    def POWER_ENABLE_CHANNEL_4_callback(self, data):
        self.data["POWER_ENABLE_CHANNEL_4"] = data

    def POWER_STATUS_CHANNEL_1_callback(self, data):
        self.data["POWER_STATUS_CHANNEL_1"] = data

    def POWER_STATUS_CHANNEL_2_callback(self, data):
        self.data["POWER_STATUS_CHANNEL_2"] = data

    def POWER_STATUS_CHANNEL_3_callback(self, data):
        self.data["POWER_STATUS_CHANNEL_3"] = data

    def POWER_STATUS_CHANNEL_4_callback(self, data):
        self.data["POWER_STATUS_CHANNEL_4"] = data

    def PROP_VOLTAGE_1_callback(self, data):
        self.data["PROP_VOLTAGE_1"] = data

    def PROP_VOLTAGE_2_callback(self, data):
        self.data["PROP_VOLTAGE_2"] = data

    def PROP_VOLTAGE_3_callback(self, data):
        self.data["PROP_VOLTAGE_3"] = data

    def PROP_VOLTAGE_4_callback(self, data):
        self.data["PROP_VOLTAGE_4"] = data

    def PROP_VOLTAGE_TOTAL_callback(self, data):
        self.data["PROP_VOLTAGE_TOTAL"] = data

    def MOTOR_CURRENT_callback(self, data):
        self.data["MOTOR_CURRENT"] = data 

    def SYSTEM_VOLTAGE_1_callback(self, data):
        self.data["SYSTEM_VOLTAGE_1"] = data

    def SYSTEM_VOLTAGE_2_callback(self, data):
        self.data["SYSTEM_VOLTAGE_2"] = data

    def SYSTEM_VOLTAGE_TOTAL_callback(self, data):
        self.data["SYSTEM_VOLTAGE_TOTAL"] = data