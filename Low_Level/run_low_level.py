# Source correct files
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../Classes') # Still is not how I should be doing this, but...it works

# Status of playfield
from playfield import Playfield

# Ros stuff
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from pinball_messages.srv import get_light, get_lightResponse
from pinball_messages.srv import get_switch, get_switchResponse
from pinball_messages.msg import override_light

# Time and scheduling
import sched, time

# Capture ctl + c
import signal

# Here we reset all playfield components to begin the game
def reset_all_components(data):
    # Let the user know we are resetting the game
    print("Deleting all events")
    
    # Turn off all lights
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            print(curr_light.override_light)
            light_off_pub.publish(curr_light.pin)

    # New playfield reference
    myPlay.reset()
    myPlay.setup_pins()
    print("2")
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            print(curr_light.override_light)
    return
    # clear all old scheduled events
    for event in schedule.queue:
        schedule.cancel(event)

# Keeps the last five commands stored here so we can change mode if you get a sertain combo:
def new_switch_hit(pin):
    myPlay.switch_list.append(pin)
    myPlay.switch_list.pop(0)
    print(myPlay.switch_list)
    switch_list_pub.publish(data=myPlay.switch_list)

# Publishes out new score value
def update_score(score_to_add):
    myPlay.score += score_to_add
    update_score_pub.publish(myPlay.score)

# Published out new bonus value
def update_bonus(bonus_to_add):
    myPlay.bonus += (bonus_to_add * myPlay.bonus_modifier)
    update_bonus_pub.publish(myPlay.bonus)

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
    print("ON")
    print(myPlay.lights["top"][0].override_light)
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
    print("OFF")
    print(myPlay.lights["top"][0].override_light)
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
    switch = myPlay.switches["top"][0]
    light = myPlay.lights["top"][0]
    if not light.override_light:
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    # Do other things, score, etc.
    
def switch_mid_0(data):
    switch = myPlay.switches["mid"][0]
    light = myPlay.lights["mid"][0]
    if not light.override_light:
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    # Do other things, score, etc.

def switch_bot_0(data):
    switch = myPlay.switches["bot"][0]
    light = myPlay.lights["bot"][0]
    if not light.override_light:
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    # Do other things, score, etc.

def switch_bot_1(data):
    switch = myPlay.switches["bot"][1]
    print("BOT PIN: " + str(switch.pin))
    new_switch_hit(switch.pin)
    reset_all_components(True)

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
switch_bot_1_sub = rospy.Subscriber("switch_bot_1_triggered", Bool, switch_bot_1)

# ROS subscirber that checkes when the start button is pressed
switch_start_button = rospy.Subscriber("switch_start_button_triggered", Bool, reset_all_components)

# ROS publishers to turn on or off lights
light_on_pub = rospy.Publisher('light_on', Int32, queue_size=10)
light_off_pub = rospy.Publisher('light_off', Int32, queue_size=10)

# ROS publisher to update score
update_score_pub = rospy.Publisher('update_score', Int32, queue_size=10)
update_bonus_pub = rospy.Publisher('update_bonus', Int32, queue_size=10)

# ROS publisher for when a new switch is hit
switch_list_pub = rospy.Publisher('switch_list', Int32MultiArray, queue_size=10)

# Scheduler to keep track of when we want to turn on.off devices on the playfield
schedule = sched.scheduler(time.time, time.sleep)

if __name__ == "__main__":
    myPlay.setup_pins()

    # Make sure everything is off at startup
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            light_off_pub.publish(curr_light.pin)

    '''
    while not rospy.is_shutdown():
        print(myPlay.switch_list)
        switch_bot_0(True)
        time.sleep(2)
        switch_mid_0(True)
        time.sleep(2)
    '''

    #rate = rospy.Rate(1)

    # Keep the scheduler in a loop
    while not rospy.is_shutdown():
        schedule.run()
        #rate.sleep()
        #if myPlay.lights["top"][0].override_light:
        #    print("VAL: " + myPlay.lights["top"][0].override_light)