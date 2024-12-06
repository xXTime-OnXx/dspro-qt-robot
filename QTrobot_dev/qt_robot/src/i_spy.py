#!/usr/bin/env python3
import rospy
import time
import random

from std_msgs.msg import String

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

from object_detection import objectDetection

class ISpy():
    
    def __init__(self):
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/gesture/play')
        # rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
        rospy.loginfo('All services are available')
                
        self.talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        self.gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.object_detection = objectDetection()
        rospy.loginfo('All services are initialized')
        
        rospy.loginfo('before choose spy object')
        # self.speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        
        rospy.loginfo('before choose spy object')
        
        self.object_whitelist = ['laptop', 'cell phone', 'bottle']
    
    def play_game(self):
        try:
            # self.talk_text_service("Hello")
            
            # self.gesture_play_service("QT/bye", 0)
            rospy.loginfo('before choose spy object')
            
            spy_object = self._choose_spy_object()
            
            i_spy_text = f'I spy something starting with the letter {spy_object[0]}'
            self.talk_text_service(i_spy_text)
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
    def _choose_spy_object(self):
        # choose random object
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
        object_set = set(map(lambda x: x.class_name, object_list))
        return random.choice(list(object_set))
    
if __name__ == '__main__':
    rospy.init_node('i_spy_robot', anonymous=True)
    
    i_spy = ISpy()
    i_spy.play_game()