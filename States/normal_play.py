import rospy
import smach

from std_msgs.msg import Int32

from Classes import playfield

import gamestate

class Normal_Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_lost'])

    def execute(self, userdata):
        pass

    def switch_callback(self, data):
        pass

    # ROS variables
    switch_sub = rospy.Subscriber("switch_state", TODO:Decide variable, switch_callback)
    flipper_pub = rospy.Publisher("flip_flipper", Int32, queue_size=1)

    playfield = None