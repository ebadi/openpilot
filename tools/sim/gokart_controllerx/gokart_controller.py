import rospy
import sys
import os
import time
import threading
import sensor_config as sens_cfg

import body_electronics_model
import camera_model
import imu_model
import lidar_model
import power_model
import propulsion_model
import sonar_model

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import String


class Gokart_Controller():
    def __init__(self):
        self.create_publishers()
        self.create_models()
        self.start_threads()
        
    def create_publishers(self):
        self.speed_publisher = rospy.Publisher(sens_cfg.SPEED, Float32, queue_size=10)
        self.turn_publisher = rospy.Publisher(sens_cfg.TURN, Float32, queue_size=10)
        self.blink_publisher = rospy.Publisher(sens_cfg.BLINK_ADAS, Float32, queue_size=10)
        self.headlight_publisher = rospy.Publisher(sens_cfg.HEADLIGHT_ADAS, Float32, queue_size=10)
        self.honk_publisher = rospy.Publisher(sens_cfg.HONK_ADAS, Float32, queue_size=10)
        self.brakelight_publisher = rospy.Publisher(sens_cfg.BRAKELIGHT_ADAS, Float32, queue_size=10)
        self.moodlight_publisher = rospy.Publisher(sens_cfg.MOODLIGHT_ADAS, Float32, queue_size=10)
        self.powermode_publisher = rospy.Publisher(sens_cfg.POWERMODE_ADAS, Float32 , queue_size=10)
        # Hamid
        self.control_speed_publisher = rospy.Publisher(sens_cfg.CONTROL_SPEED, Float32, queue_size=10)
        self.control_turn_publisher = rospy.Publisher(sens_cfg.CONTROL_TURN, Float32, queue_size=10)
        


    def create_models(self):
        self.body_electronics_model = body_electronics_model.Body_Electronics_Model()
        self.camera_model = camera_model.Camera_Model()
        self.imu_model = imu_model.IMU_Model()
        self.lidar_model = lidar_model.Lidar_Model()
        self.power_model = power_model.Power_Model()
        self.propulsion_model = propulsion_model.Propulsion_Model()
        self.sonar_model = sonar_model.Sonar_Model()

    def start_threads(self):
        threading.Thread(target=self.body_electronics_model.body_electronics_loop).start()
        threading.Thread(target=self.camera_model.camera_loop).start()
        threading.Thread(target=self.imu_model.IMU_loop).start()
        threading.Thread(target=self.lidar_model.lidar_loop).start()
        threading.Thread(target=self.power_model.power_loop).start()
        threading.Thread(target=self.propulsion_model.propulsion_loop).start()
        threading.Thread(target=self.sonar_model.sonar_loop).start()

    def get_body_electronics(self):
        return self.body_electronics_model.data

    def get_camera(self):
        return self.camera_model.data

    def get_imu(self):
        return self.imu_model.data

    def get_lidar(self):
        return self.lidar_model.data

    def get_power_info(self):
        return self.power_model.data

    def get_propulsion_info(self):
        return self.propulsion_model.data
        
    def get_sonar_info(self):
        return self.sonar_model.data
    
    def set_power_enable(self):
        raise NotImplementedError

    def set_speed(self, speed):
        self.control_speed_publisher.publish(speed)

    def set_turn_rate(self, degrees):
        self.control_turn_publisher.publish(degrees)
        
    def get_body_electronics(self):
        return self.body_electronics_model.data

    def set_headlight(self, mode):
        self.headlight_publisher.publish(mode)

    # Hamid
    def set_brakelight(self, mode):
        self.brakelight_publisher.publish(mode)

    def set_blink(self, direction):
        self.blink_publisher.publish(direction)

    def set_mode(self, mode):
        self.powermode_publisher(mode)
    
    def set_moodlight(self, mood):
        self.moodlight_publisher.publish(mood)

    def set_horn(self, mode):
        self.honk_publisher.publish(mode)
    
# if __name__ == "__main__":
#     if len(sys.argv) != 2:
#         print("Too few arguments. Usage: ")
#         print("gokart_controller.py <ROS_MASTER_URI>")
#         sys.exit()
#     #os.environ["ROS_MASTER_URI"] = sys.argv[1]
#     g = Gokart_Controller(sys.argv[1])