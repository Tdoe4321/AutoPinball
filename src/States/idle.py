import rospy
import smach

from std_msgs.msg import Bool
from AutoPinball.msg import override_light

import time

class Idle(smach.State):
    def runtime_init(self):
        # ROS variables                
        # Let's you blink, hold, etc. on lights
        self.override_light_pub = rospy.Publisher("override_light", override_light, queue_size=10)

        # Switch list update
        self.switch_start_button = rospy.Subscriber("switch_start_button_triggered", Bool, self.start_game_callback)

        # For when you don't have a start button hooked up
        self.debug = False

        # So we can listen for the start button being pressed
        self.start = False

    def runtime_deinitialize(self):
        # ROS variables                
        # Let's you blink, hold, etc. on lights
        self.override_light_pub = None

        # Switch list update
        self.switch_start_button = None

        # For when you don't have a start button hooked up
        self.debug = None

        # So we can listen for the start button being pressed
        self.start = None

    def start_game_callback(self, data):
        self.start = True

    def __init__(self):
        smach.State.__init__(self, outcomes=['start_game'])
        self.runtime_deinitialize()

    def execute(self, userdata):
        print("Idle_State")

        print("Game Initializing...")
        self.runtime_init()
        time.sleep(1)
        print("Ready to Play!")

        # TODO: add all the lights we want to blink when idleing, i'm thinking we alternate all of them
        self.override_light_pub.publish("top",0,"Blink_Slow")
        self.override_light_pub.publish("mid",0,"Blink_Slow")
        self.override_light_pub.publish("bot",0,"Blink_Slow")

        # Every second, we ask the user to press the start button
        rate = rospy.Rate(1) # 1hz
        while not rospy.is_shutdown():
            print("Press the start buton to begin!")
            rate.sleep()
            if self.start:
                self.runtime_deinitialize()
                #self.start = False
                return 'start_game'
            if self.debug:
                self.runtime_deinitialize()
                return 'start_game'
