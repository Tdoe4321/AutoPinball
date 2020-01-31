# Source correct files
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../Classes') # Still is not how I should be doing this, but...it works

# Status of playfield
from playfield import Playfield

# Ros stuff
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from pinball_messages.srv import get_light, get_lightResponse
from pinball_messages.srv import get_switch, get_switchResponse
from pinball_messages.msg import override_light

# Time and scheduling
import sched, time

# Capture ctl + c
import signal

# Based on ROS srv of row and column, return the information inside the light
def handle_get_light(req):
    light = myPlay.lights[req.row][req.column]
    return get_lightResponse(light.on, light.last_time_on, light.pin, light.general_light_on_time, light.override_light) # Have a lookup table for information

# Based on ROS srv of row and column, return the information inside the light
def handle_get_switch(req):
    switch = myPlay.switches[req.row][req.column]
    return get_switchResponse(switch.on, switch.last_time_on, switch.pin, switch.num_times_triggered)

# Can put the mode of any switch into "Blink, Hold, etc." 
def handle_override_light(override):
    light = myPlay.lights[override.row][override.column]
    if override.override == "None":
        light.override_light = None
        turn_off(light)
    else:
        light.override_light = override.override
        turn_on(light)

# Turns on a light. If it is supposed to be blinking, it tells it to turn off
def turn_on(light):
    light.on = True
    light.last_time_on = rospy.get_rostime().to_sec()
    light_on_pub.publish(light.pin)
    if light.override_light is None:
        schedule.enter(light.general_light_on_time, 1, turn_off, argument=(light,)) 
    elif light.override_light == "Blink_Slow":
        schedule.enter(1, 1, turn_off, argument=(light,))
    elif light.override_light == "Blink_Med":
        schedule.enter(0.6, 1, turn_off, argument=(light,))
    elif light.override_light == "Blink_Fast":
        schedule.enter(0.3, 1, turn_off, argument=(light,))

    #print("on")

# Turns off a light. If it is supposed to be blinking, it tieels it to turn on
def turn_off(light):
    light.on = False
    light_off_pub.publish(light.pin)
    if light.override_light == "Blink_Slow":
        schedule.enter(1, 1, turn_on, argument=(light,))
    elif light.override_light == "Blink_Med":
        schedule.enter(0.6, 1, turn_on, argument=(light,))
    elif light.override_light == "Blink_Fast":
        schedule.enter(0.3, 1, turn_on, argument=(light,))

    #print("off")

# Callback for each switch on the playfield...
def switch_top_0(data):
    light = myPlay.lights["top"][0]
    turn_on(light)
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

# Capture ros shutdown
def signal_handler():
        print('\nExiting...')
        for event in schedule.queue:
            schedule.cancel(event)

        # Turn off all lights
        for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
            for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
                light_off_pub.publish(curr_light.pin)

# The status of the playfield, lights, switches, score, etc
myPlay = Playfield()

# ROS initialization and shutdown
rospy.init_node('low_level')
rospy.on_shutdown(signal_handler)

# Ros services to return the lights and switches
get_light_service = rospy.Service('get_light', get_light, handle_get_light)
get_switch_service = rospy.Service('get_switch', get_switch, handle_get_switch)

# ROS subscriber to change the status of a light from blinking to not or vice versa
override_light_sub = rospy.Subscriber("override_light", override_light, handle_override_light)

# ROS subscribers for each switch that will exist on the playfield
switch_top_0_sub = rospy.Subscriber("switch_top_0_triggered", Bool, switch_top_0)
switch_mid_0_sub = rospy.Subscriber("switch_mid_0_triggered", Bool, switch_mid_0)
switch_bot_0_sub = rospy.Subscriber("switch_bot_0_triggered", Bool, switch_bot_0)

# ROS publishers to turn on or off lights
light_on_pub = rospy.Publisher('light_on', Int32, queue_size=10)
light_off_pub = rospy.Publisher('light_off', Int32, queue_size=10)

# Scheduler to keep track of when we want to turn on.off devices on the playfield
schedule = sched.scheduler(time.time, time.sleep)

if __name__ == "__main__":
    # Setup all the pins for the switches on the playfield
    myPlay.switches["top"][0].pin = 11
    myPlay.switches["mid"][0].pin = 12
    myPlay.switches["bot"][0].pin = 13

    # Setup all the pins for the lights on the playfield
    myPlay.lights["top"][0].pin = 3 # Test Numbers
    myPlay.lights["mid"][0].pin = 4
    myPlay.lights["bot"][0].pin = 5

    # Make sure everything is off at startup
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            light_off_pub.publish(curr_light.pin)
    
    # Keep the scheduler in a loop
    while not rospy.is_shutdown():
        schedule.run()
