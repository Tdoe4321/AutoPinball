import rospy
import smach

from std_msgs.msg import Int32
from pinball_messages.srv import get_light
from pinball_messages.srv import get_switch
from pinball_messages.msg import override_light

import time

from Classes import playfield
from Classes import light
from Classes import switch

import gamestate

class Normal_Play(smach.State):
    # Returns an object of what a light is based on row and column
    def get_light_object(self, row, column):
        try:
            responce = self.get_light_call(row, column)
            return light.Light(responce.on, responce.last_time_on, responce.pin, responce.general_light_on_time, responce.override_light)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)
            return False

    # Returns an object of what a switch is based on row and column
    def get_switch_object(self, row, column):
        try:
            responce = self.get_switch_call(row, column)
            return switch.Switch(responce.on, responce.last_time_on, responce.pin, responce.num_times_triggered)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)
            return False

    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_lost'])
        # ROS variables
        # 1 = left_flipper ON, 2 = right_flipper ON, -1 = left_flipper OFF, -2 = right_flipper OFF
        self.flipper_pub = rospy.Publisher("flip_flipper", Int32, queue_size=1)
        
        # int corresponding to pin number of light to turn off or on
        self.light_on_pub = rospy.Publisher("light_on", Int32, queue_size=10)
        self.light_off_pub = rospy.Publisher("light_off", Int32, queue_size=10)
        
        # Let's you blink, hold, etc. on lights
        self.override_light_pub = rospy.Publisher("override_light", override_light, queue_size=10)

        # Get a reference to a switch at an instant
        self.get_light_call = rospy.ServiceProxy('get_light', get_light)
        self.get_switch_call = rospy.ServiceProxy('get_switch', get_switch)

    def execute(self, userdata):
        print("Normal_play")

        print("Game Initializing...")
        time.sleep(1)
        print("Ready to Play!")

        '''
        #How to make sommething blink, hold, etc.
        override_msg = override_light()
        override_msg.row = "top"
        override_msg.column = 0
        override_msg.override = "Hold"
        self.override_light_pub.publish(override_msg)

        time.sleep(3)

        override_msg.override = "None"
        self.override_light_pub.publish(override_msg)
        '''

        '''
        # How to get a quick reference to a light of switch
        print(self.get_light_object("top", 0).pin)
        print(self.get_switch_object("top", 0).pin)
        '''

        while not rospy.is_shutdown():
            return 'ball_lost'
