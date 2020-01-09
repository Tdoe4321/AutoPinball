import rospy
import smach

from std_msgs.msg import Int32

from Classes import playfield

import gamestate

class Normal_Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_lost'])

    def execute(self, userdata):
        print("Normal_play")
        while(True):
            rospy.loginfo("Hello World")
            return 'ball_lost'

    def switch_callback(self, data):
        pass

    # ROS variables
    switch_sub = rospy.Subscriber("switch_state", Int32, switch_callback)
    flipper_pub = rospy.Publisher("flip_flipper", Int32, queue_size=1)

    playfield = None