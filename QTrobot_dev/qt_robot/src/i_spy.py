#!/usr/bin/env python3
import rospy
import time
import random

from std_msgs.msg import String

from qt_robot_interface.srv import behavior_talk_text
from qt_gesture_controller.srv import gesture_play
from custom_interfaces.srv import MicrophoneBasedSpeechRecognition

from object_detection import objectDetection
from gemini_adapter import GeminiAdapter

class ISpy():
    
    def __init__(self):
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/gesture/play')
        rospy.wait_for_service('/custom/speech/sr/microphone_recognize')
        rospy.loginfo('All services are available')
                
        self.talk_text_service = rospy.ServiceProxy('/qt_robot/speech/say', behavior_talk_text)
        self.gesture_play_service = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
        self.object_detection = objectDetection()        
        self.speech_recognition_service = rospy.ServiceProxy('/custom/speech/sr/microphone_recognize', MicrophoneBasedSpeechRecognition)
        rospy.loginfo('All services are initialized')
        
        self.gemini = GeminiAdapter(
            'We are playing a game of i spy i will shortly give you the object that currently is spied. '+
            'If I guess the object correct you will just return the text "correct". if its wrong you will just return the text "wrong".'+
            'Please answer all the questions about the spied object and if I ask for a hint give me a little hint.')
        rospy.loginfo('Init Gemini request succeeded')

        self.object_whitelist = ['laptop', 'cell phone', 'bottle']
        
    
    def play_game(self):
        try:
            self.talk_text_service("Hello, let's play the game 'I spy'")
            
            # self.gesture_play_service("QT/bye", 0)
            
            spy_object = self._choose_spy_object()
            
            self.gemini.request('the spied object is '+ spy_object)

            i_spy_text = f'I spy something starting with the letter {spy_object[0]}'
            self.talk_text_service(i_spy_text)
            
            guessed_right = False
            while(not guessed_right):
                speech_input = self.speech_recognition_service("en-US")
                
                #TODO: check if speech_input.text predicts the correct object with ai service
                res = self.gemini.request(speech_input.text)
                rospy.loginfo(res)
                if res == 'correct':
                    guessed_right = True
                else:
                    self.talk_text_service('Thats wrong')

                
            self.talk_text_service("You guessed the word!")
            
            
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            
    def _choose_spy_object(self):
        objects = self.object_detection.detect_objects()
        object_list = [object for object in objects if object.class_name in self.object_whitelist]
            
        object_set = set(map(lambda x: x.class_name, object_list))
        return random.choice(list(object_set))
    
if __name__ == '__main__':
    rospy.init_node('i_spy_robot', anonymous=True)

    i_spy = ISpy() 
    i_spy.play_game()