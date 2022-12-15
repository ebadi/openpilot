from gokart_controller import Gokart_Controller
import time
import rospy
import sys
import select
import os 


#export ROS_MASTER_URI=http://dobby.local:11311
#export ROS_IP=192.168.150.178


gc = Gokart_Controller("http://dobby.local:11311")
rate = rospy.Rate(10)

Functions = ["1.Headlights","2.Breaklights","3.Blinkers","4.Moodlight","5.Horn","6.Sonar sensors","7.Powermode"]
Sensors = ["Sonar1","Sonar2","Sonar3","Sonar4","Sonar4","Sonar5","Sonar6","All data"]
Powermodes = ["\n1.Locked","2.unlocked","3.Key in","4.Accessory","5.On","6.Running"]

def start():
    print(*Functions,sep='\n')
    x = int(input("choose function\n"))
    if not x < 8:
        print("Wrong value!\n\n")
    else:
        return x





while not rospy.is_shutdown():
    x = start()
    while x == 1:
        Headlight_mode = int(input("\nEnter Headlights mode\n0 for low intensity\n1 for high intensity\nAny other number to go back to the functions'\n"))
        if Headlight_mode >= 2:
            Headlight_mode = 0
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            gc.headlight_publisher.publish(Headlight_mode)
            print('\n') 
            rate.sleep()
    
    while x == 2:
        Breaklight_mode = int(input("\nEnter Breaklight mode\n0 For low intensity\n1 For high intensity\nAny other number To go back to functions\n"))
        if Breaklight_mode >= 2:
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            gc.brakelight_publisher.publish(Breaklight_mode)
            print('\n')
            rate.sleep()

    while x == 3:
        Blinker_mode = int(input("\nEnter Blinkers direction\n0 For Off \n1 For Left\n2 For Right\n3 For Emergency blinkers \nAny other number To go back to functions\n"))
        if Blinker_mode >= 4:
            Blinker_mode = 0
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            gc.blink_publisher.publish(Blinker_mode)
            print('\n')
            rate.sleep()

    while x == 4:
        Moodlight_mode = int(input("\nEnter Moodlight mode\nAvailable moods between 1 and 7\nAny other number To go back to functions\n"))
        if Moodlight_mode >= 8:
            Moodlight_mode = 0
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            gc.moodlight_publisher.publish(Moodlight_mode)
            print('\n')
            rate.sleep()
    
    while x == 5:
        Horn_mode = int(input("\nEnter horn mode\n0 For Off\n1 To honk for 1 secound\nAny other number To go back to functions\n"))
        if Horn_mode >= 2:
            Horn_mode = 0
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            gc.honk_publisher.publish(Horn_mode)
            time.sleep(1)
            Horn_mode = 0
            gc.honk_publisher.publish(Horn_mode)
            print('\n')
            rate.sleep()
    
    while x == 6:
        Sensors_print = int(input("Sonar sensors view\nPress 1 for printing all sensors data\nAny other number To go back to functions\n"))
        if Sensors_print >= 2:
            print("Going back to functions!\n\n")
            x == 0
            break
        else:
            while not rospy.is_shutdown():
                print(gc.get_sonar_info())


    
    while x == 7:
        
        print(*Powermodes,sep='\n')
        powermode_mode = int(input("\nChoose Powermode\nAny other number To go back to functions\n"))
        if powermode_mode >= 8:
            print("Going back to functions!\n\n")
            x == 0
            break

        else:
            gc.powermode_publisher.publish(powermode_mode)
            print('\n')
            rate.sleep()
            
    

    else:
        x == 0

