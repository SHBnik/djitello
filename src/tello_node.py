#!/usr/bin/env python


import time
import sys
import tello
import cv2
import threading

import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from djitello.srv import *
import os
import sys
 


drone = None




def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg



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


    # Parse yaml file
    path = os.path.expanduser("~/catkin_ws")
    camera_info_msg = yaml_to_CameraInfo(path+"/src/djitello/src/cam_calibration.yaml")


    bridge = CvBridge()
    rospy.init_node('tello_node', anonymous=False)
    image_pub = rospy.Publisher("tello_node/image_raw",Image,queue_size=10)
    caminfo_pub = rospy.Publisher("tello_node/camera_info", CameraInfo, queue_size=10)
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
            caminfo_pub.publish(camera_info_msg)
    except Exception as e:
        rospy.logerr('Error in connecting to tello : %s'%e)
