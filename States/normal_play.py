import rospy
import smach

from std_msgs.msg import Int32
from pinball_messages.srv import getData
from pinball_messages.msg import override_light

import time

from Classes import playfield

import gamestate

class Normal_Play(smach.State):
    def switch_callback(self, data):
        #TODO: Make case statement for what to do
        pass

    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_lost'])
        # ROS variables
        # Int32 corresponding to switch triggered (int based on pin number)
        switch_sub = rospy.Subscriber("switch_triggered", Int32, self.switch_callback)
        # 1 = left_flipper ON, 2 = right_flipper ON, -1 = left_flipper OFF, -2 = right_flipper OFF
        self.flipper_pub = rospy.Publisher("flip_flipper", Int32, queue_size=1)
        # int corresponding to pin number of light to turn off or on
        self.light_on_pub = rospy.Publisher("light_on", Int32, queue_size=10)
        self.light_off_pub = rospy.Publisher("light_off", Int32, queue_size=10)
        
        self.override_light_pub = rospy.Publisher("override_light", override_light, queue_size=10)

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

        ''' How to get data for score, etc.
        try:
            getData_call = rospy.ServiceProxy('get_data', getData)
            responce = getData_call("Hello")
            print(responce.data)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)
        '''

        while not rospy.is_shutdown():
            return 'ball_lost'
