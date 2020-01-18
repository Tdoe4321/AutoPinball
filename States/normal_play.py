import rospy
import smach

from std_msgs.msg import Int32

from Classes import playfield

import gamestate

class Normal_Play(smach.State):
    def switch_callback(self, data):
        pass

    def __init__(self):
        smach.State.__init__(self, outcomes=['ball_lost'])
        # ROS variables
        # 1 = left_flipper ON, 2 = right_flipper ON, -1 = left_flipper OFF, -2 = right_flipper OFF
        self.flipper_pub = rospy.Publisher("flip_flipper", Int32, queue_size=1)
        # int corresponding to pin number of light to turn off or on
        self.light_on_pub = rospy.Publisher("light_on", Int32, queue_size=10)
        self.light_off_pub = rospy.Publisher("light_off", Int32, queue_size=10)
        
        self.playfield = None

    def execute(self, userdata):
        print("Normal_play")
        while(True):
            self.light_on_pub.publish(1)
            rospy.loginfo("Hello World")
            return 'ball_lost'

    # Int32 corresponding to switch triggered (int based on pin number)
    switch_sub = rospy.Subscriber("switch_triggered", Int32, switch_callback)
