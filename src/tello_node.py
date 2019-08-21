#!/usr/bin/env python

import tello
import cv2
import threading
import time 
import sys

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from djitello.srv import *


drone = None


def command_handler(req):
    rospy.loginfo('sent command to drone :%s'%req.command) 
    return SendCommandResponse(drone.send_command(req.command))


def takeoff(data):
    global drone
    drone.takeoff()

def land(data):
    global drone
    drone.land()

def sendingCommand():  # for keeping tello alive
    """
    start a while loop that sends 'command' to tello every 5 second
    """    
    while not rospy.is_shutdown():
        drone.send_command('command') 
        time.sleep(5)

if __name__ == "__main__":

    bridge = CvBridge()
    rospy.init_node('tello_node', anonymous=False)
    image_pub = rospy.Publisher("tello_node/image_raw",Image,queue_size=10)
    rospy.Service('tello_node/send_command', SendCommand, command_handler)
    rospy.Subscriber("tello_node/takeoff", String, takeoff) # just for quick take off
    rospy.Subscriber("tello_node/land", String, land) # just for quick land

    try:
        drone = tello.Tello('', 8889)
        time.sleep(0.5)
        sending_command_thread = threading.Thread(target = sendingCommand)
        sending_command_thread.daemon = True
        sending_command_thread.start()

        while not rospy.is_shutdown():
            img = drone.read()
            if img is None :
                continue 
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

    except Exception as e:
        rospy.logerr('Error in connecting to tello : %s'%e)
