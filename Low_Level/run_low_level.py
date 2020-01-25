import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../Classes') # Still is not how I should be doing this, but...it works

from playfield import Playfield

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool

def turn_on(light):
    light.on = True
    light.last_time_on = rospy.get_rostime().to_sec()
    light_on_pub.publish(light.pin)

def turn_off(light):
    light.on = False
    light_off_pub.publish(light.pin)

def switch_top_0(data):
    turn_on(myPlay.lights["top"][0])
    myPlay.switches["top"][0].num_times_triggered += 1
    # Do other things, score, etc.
    
def switch_mid_0(data):
    turn_on(myPlay.lights["mid"][0])
    myPlay.switches["mid"][0].num_times_triggered += 1
    # Do other things, score, etc.

def switch_bot_0(data):
    turn_on(myPlay.lights["bot"][0])
    myPlay.switches["bot"][0].num_times_triggered += 1
    # Do other things, score, etc.

myPlay = Playfield()

rospy.init_node('low_level')

switch_top_0_sub = rospy.Subscriber("switch_top_0_triggered", Bool, switch_top_0)
switch_mid_0_sub = rospy.Subscriber("switch_mid_0_triggered", Bool, switch_mid_0)
switch_bot_0_sub = rospy.Subscriber("switch_bot_0_triggered", Bool, switch_bot_0)

light_on_pub = rospy.Publisher('light_on', Int32, queue_size=10)
light_off_pub = rospy.Publisher('light_off', Int32, queue_size=10)

if __name__ == "__main__":
    # Setup all the pins for the switches on the playfield
    myPlay.switches["top"][0].pin = 11
    myPlay.switches["mid"][0].pin = 12
    myPlay.switches["bot"][0].pin = 13

    # Setup all the pins for the lights on the playfield
    myPlay.lights["top"][0].pin = 3 # Test Numbers
    myPlay.lights["mid"][0].pin = 4
    myPlay.lights["bot"][0].pin = 5

    # This checks for lights that should be shutdown
    while not rospy.is_shutdown():
        for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
            i = 0 # Tracking index
            for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
                # if the light is on AND we aren't overriding it AND it's been longer than the set time...
                if curr_light.on and not curr_light.override_light and (rospy.get_rostime().to_sec() - curr_light.last_time_on) > curr_light.general_light_on_time:
                    print("row: " + row + ", light: " + str(i) + " Has been turned off")
                    turn_off(curr_light)

                i += 1 # incrementing index