from gokart_controller import Gokart_Controller
import time
import rospy
gc = Gokart_Controller("http://192.168.150.107:11311")
import random
i = 5
b = 0.1
while not rospy.is_shutdown():
    #print(gc.get_camera())
    #print(gc.set_speed(5))
    i = random.uniform(0, 10)
    print(gc.set_turn_rate(i))
    if i > 8 or i < 2 : 
        b = -1 * b 
    print ("i=", i, "  b=", b )
    # rate.sleep()
