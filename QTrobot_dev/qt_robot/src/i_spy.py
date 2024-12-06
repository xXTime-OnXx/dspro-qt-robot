#!/usr/bin/env python3
import rospy
import cv2
import time
import random

from std_msgs.msg import String
from sensor_msgs.msg import Image

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition
from custom_interfaces.srv import Detectron
from cv_bridge import CvBridge

class ISpy():
    
    def __init__():
        pass