#!/usr/bin/env python

# Source correct files
import sys, os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + '/../Classes') # Still is not how I should be doing this, but...it works

# Schedule tasks
from apscheduler.schedulers.background import BackgroundScheduler
from datetime import datetime  
from datetime import timedelta 

# Status of playfield
from playfield import Playfield

# Ros stuff
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from AutoPinball.srv import get_light, get_lightResponse
from AutoPinball.srv import get_switch, get_switchResponse
from AutoPinball.msg import override_light
from AutoPinball.msg import flip_flipper

# Time
import time

# Capture ctl + c
import signal

# Explicitly checks the bottom row of the playfield for all the lights being on
def check_bottom_row():
    # If a single one is not on, then leave
    for single_light in myPlay.lights["bot"]:
        if not single_light.on:
            return

    # If we've made it here, that means that all of them are on
    myPlay.bonus_modifier += 1
    update_score(10000)

    for single_light in myPlay.lights["bot"]:
        turn_off(single_light)

# Method to check for switch debouncing
def is_separate_trigger(switch):
    if rospy.get_rostime().to_sec() - switch.last_time_on >= switch.debounce_time:
        return True
    else:
        return False

# Shift a group of lights to the left, given the left-most light
# and the (inclusive) number of lights to shift
def shift_left(row, left_index, num_of_lights):
    temp_light_status = myPlay.lights[row][left_index].on
    for i in range(left_index, left_index + num_of_lights - 1):
        if(myPlay.lights[row][i + 1].on):
            turn_on(myPlay.lights[row][i])
        else:
            turn_off(myPlay.lights[row][i])
    
    if(temp_light_status):
        turn_on(myPlay.lights[row][left_index+num_of_lights - 1])
    else:
        turn_off(myPlay.lights[row][left_index+num_of_lights - 1])

def shift_right(row, right_index, num_of_lights):
    temp_light_status = myPlay.lights[row][right_index].on
    for i in range(right_index, right_index - num_of_lights + 1, -1):
        if(myPlay.lights[row][i - 1].on):
            turn_on(myPlay.lights[row][i])
        else:
            turn_off(myPlay.lights[row][i])
    
    if(temp_light_status):
        turn_on(myPlay.lights[row][right_index-num_of_lights + 1])
    else:
        turn_off(myPlay.lights[row][right_index-num_of_lights + 1])

# Method to handle all the changeing of mode
def change_mode(new_mode):
    print("Changing to new mode: " + new_mode)
    myPlay.mode=new_mode
    update_message_pub.publish("MODE: " + new_mode)

# Will use flipper.general_flipper_on_time if time_until_off is not set. 
# Set it to 0 to hold
def flipper_on(flipper, time_util_off=-1):
    print("ON: " + str(flipper.flipper_num))
    if flipper.on:
        return
    flipper.on = True
    flipper.last_time_on = rospy.get_rostime().to_sec()
    flipper_pub.publish(flipper.flipper_num)
    if time_util_off > 0:
        try:
            schedule.add_job(flipper_off, 'date', run_date=(datetime.now() + timedelta(seconds=time_util_off)), args=[flipper], id=str(flipper.flipper_num) + "OFF")
        except:
            print("Tried to schedule flipper off: " + str(flipper.flipper_num))
    elif time_util_off == 0:
        # Will HOLD the flipper until you turn it off
        pass
    else:
        try:
            schedule.add_job(flipper_off, 'date', run_date=(datetime.now() + timedelta(seconds=flipper.general_flipper_on_time)), args=[flipper], id=str(flipper.flipper_num) + "OFF")
        except:
            print("Tried to schedule flipper off: " + str(flipper.flipper_num))

def flipper_off(flipper):
    print("OFF: " + str(flipper.flipper_num))
    if not flipper.on:
        return
    flipper.on = False
    flipper_pub.publish(flipper.flipper_num * -1)

def flip_flipper_callback(flipper_msg):
    if flipper_msg.flipper == 1:
        flipper_on(myPlay.left_flipper, flipper_msg.time)
    elif flipper_msg.flipper == 2:
        flipper_on(myPlay.right_flipper, flipper_msg.time)
    elif flipper_msg.flipper == -1:
        flipper_off(myPlay.left_flipper)
    elif flipper_msg.flipper == -2:
        flipper_off(myPlay.right_flipper)

# Schedule a time to turn a light (or a coil) on
def schedule_on(light, time_until):
    try:
        schedule.add_job(turn_on, 'date', run_date=calc_date(time_until, light), args=[light], id=str(light.pin) + "ON")
    except:
        print("Tried to schedule on, job with same name, no can do!")

# Schedule a time to turn a light (or a coil) off
def schedule_off(light, time_until):
    try:
        schedule.add_job(turn_off, 'date', run_date=calc_date(time_until, light), args=[light], id=str(light.pin) + "OFF") 
    except:
        print("Tried to schedule off, job with same name, no can do!")

# Calculate the time to turn on or off a light
def calc_date(seconds_in_future, light):
    if light.blink_start_time == -1:
        return datetime.now() + timedelta(seconds=seconds_in_future)
    elif light.on:
        return light.blink_start_time + timedelta(seconds=(seconds_in_future + (2*light.curr_number_blink * seconds_in_future)))
    elif not light.on:
        return light.blink_start_time + timedelta(seconds=(seconds_in_future + ((2*light.curr_number_blink - 1) * seconds_in_future)))

# Here we reset all playfield components to begin the game
def reset_all_components():
    # Let the user know we are resetting the game
    print("Deleting all events")
    
    # Turn off all flippers
    flipper_off(myPlay.left_flipper)
    flipper_off(myPlay.right_flipper)

    # Turn off all lights
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            pin_off_pub.publish(curr_light.pin)

    # Turn off all coils
    for coil in myPlay.coils:
        pin_off_pub.publish(coil.pin)

    # New playfield reference
    myPlay.reset()
    myPlay.setup_pins()

    # clear all old scheduled events
    for job in schedule.get_jobs():
        job.remove()

    # Publish out that we have no points
    update_score(0)
    update_bonus(0)

# Keeps the last five commands stored here so we can change mode if you get a sertain combo:
def new_switch_hit(pin):
    myPlay.switch_list.append(pin)
    myPlay.switch_list.pop(0)
    switch_list_pub.publish(data=myPlay.switch_list)

# Publishes out new score value
def update_score(score_to_add):
    if myPlay.mode != "Idle" and myPlay.mode != "Idle_Waiting":
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
    if override.override == light.override_light:
        pass
    else:
        light.override_light = override.override
        if override == "None":
            turn_off(light)
        else:
            turn_on(light)

def local_override_light(override, light):
    if override == light.override_light:
        pass
    else:
        light.override_light = override
        if override == "None":
            turn_off(light)
            light.curr_number_blink = 0
            light.blink_start_time = -1
            try:
                schedule.remove_job(str(light.pin) + "ON")
            except:
                print("Tried to remove job, but no name - NONE")
        else:
            light.curr_number_blink = 0
            light.blink_start_time = datetime.now()
            try:
                schedule.remove_job(str(light.pin) + "ON")            
            except:
                print("Tried to remove job, but no name - ON")
            try:
                schedule.remove_job(str(light.pin) + "OFF")            
            except:
                print("Tried to remove job, but no name - OFF")
            turn_on(light)



# Turns on a light. If it is supposed to be blinking, it tells it to turn off
def turn_on(light):
    light.on = True
    light.last_time_on = rospy.get_rostime().to_sec()
    pin_on_pub.publish(light.pin)
    if light.override_light == "Hold":
        pass
    elif light.override_light == "Blink_Slow":
        schedule_off(light, 1)
        light.curr_number_blink += 1
    elif light.override_light == "Blink_Med":
        schedule_off(light, 0.6)
        light.curr_number_blink += 1
    elif light.override_light == "Blink_Fast":
        schedule_off(light, 0.3)
        light.curr_number_blink += 1
    else:
        schedule_off(light, light.general_light_on_time)

    #print("on")

# Turns off a light. If it is supposed to be blinking, it tells it to turn on
def turn_off(light):
    light.on = False
    pin_off_pub.publish(light.pin)
    if light.override_light == "Blink_Slow":
        schedule_on(light, 1)
    elif light.override_light == "Blink_Med":
        schedule_on(light, 0.6)
    elif light.override_light == "Blink_Fast":
        schedule_on(light, 0.3)

    #print("off")

# Callback for each switch on the playfield...
################  TOP ################
def switch_top_0(data): # Left Orbit 
    switch = myPlay.switches["top"][0]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][0]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(1000)
    switch.last_time_on = rospy.get_rostime().to_sec()
    
def switch_top_1(data): # Left Bumper
    switch = myPlay.switches["top"][1]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["top"][0]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    turn_on(myPlay.coils[0])
    new_switch_hit(switch.pin)
    print("left")
    update_score(1)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_top_2(data): # Middle Bumper
    switch = myPlay.switches["top"][2]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["top"][1]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    turn_on(myPlay.coils[1])
    new_switch_hit(switch.pin)
    print("mid")
    update_score(10)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_top_3(data): # Right Bumper
    switch = myPlay.switches["top"][3]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["top"][2]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    turn_on(myPlay.coils[2])
    new_switch_hit(switch.pin)
    print("right")
    update_score(100)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_top_4(data): # Right Orbit
    switch = myPlay.switches["top"][4]
    if not is_separate_trigger(switch):
        return
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(1000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_top_5(data): # Spinner
    switch = myPlay.switches["top"][5]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][7]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(50)
    switch.last_time_on = rospy.get_rostime().to_sec()

################  MID ################
def switch_mid_0(data): # Left plafield lane
    switch = myPlay.switches["mid"][0]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][1]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(1000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_mid_1(data): # Left Standup
    switch = myPlay.switches["mid"][1]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][2]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(3000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_mid_2(data): # Middle Rollover
    switch = myPlay.switches["mid"][2]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][3]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(10000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_mid_3(data): # Right Standup
    switch = myPlay.switches["mid"][3]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][4]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(3000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_mid_4(data): # Right playfield lane
    switch = myPlay.switches["mid"][4]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][5]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(1000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_mid_5(data): # Right Ramp
    switch = myPlay.switches["mid"][5]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["mid"][6]
    if light.override_light == "None":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    update_score(1000)
    switch.last_time_on = rospy.get_rostime().to_sec()

################  BOT ################
def switch_bot_0(data): # Left Outlane
    switch = myPlay.switches["bot"][0]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["bot"][0]
    if light.override_light == "None" or light.override_light == "Hold":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    check_bottom_row()
    update_score(5000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_1(data): # Left Inlane
    switch = myPlay.switches["bot"][1]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["bot"][1]
    if light.override_light == "None" or light.override_light == "Hold":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    check_bottom_row()
    update_score(500)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_2(data): # Left Slingshot
    switch = myPlay.switches["bot"][2]
    if not is_separate_trigger(switch):
        return
    switch.num_times_triggered += 1
    turn_on(myPlay.coils[3])
    new_switch_hit(switch.pin)
    update_score(99)
    print("l Sling")
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_3(data): # Right Slingshot
    switch = myPlay.switches["bot"][3]
    if not is_separate_trigger(switch):
        return
    switch.num_times_triggered += 1
    turn_on(myPlay.coils[4])
    new_switch_hit(switch.pin)
    update_score(100)
    print("R Sling")
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_4(data): # Right Inlane
    switch = myPlay.switches["bot"][4]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["bot"][2]
    if light.override_light == "None" or light.override_light == "Hold":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    check_bottom_row()
    update_score(500)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_5(data): # Right Outlane
    switch = myPlay.switches["bot"][5]
    if not is_separate_trigger(switch):
        return
    light = myPlay.lights["bot"][3]
    if light.override_light == "None" or light.override_light == "Hold":
        turn_on(light)
    switch.num_times_triggered += 1
    new_switch_hit(switch.pin)
    check_bottom_row()
    update_score(5000)
    switch.last_time_on = rospy.get_rostime().to_sec()

def switch_bot_6(data): # Left Flipper
    switch = myPlay.switches["bot"][6]
    if not is_separate_trigger(switch):
        return
    switch.num_times_triggered += 1
    switch.last_time_on = rospy.get_rostime().to_sec()
    shift_left("bot", 0, 4)

def switch_bot_7(data): # Right Flipper
    switch = myPlay.switches["bot"][7]
    if not is_separate_trigger(switch):
        return
    switch.num_times_triggered += 1
    switch.last_time_on = rospy.get_rostime().to_sec()
    shift_right("bot", 3, 4)

def switch_bot_8(data): # Drain
    switch = myPlay.switches["bot"][8]
    if not is_separate_trigger(switch):
        return
    print("Ball Drained")
    new_switch_hit(switch.pin)
    flipper_off(myPlay.left_flipper)
    flipper_off(myPlay.right_flipper)
    change_mode("Final_Screen")
    switch.last_time_on = rospy.get_rostime().to_sec()

################  OTHER ################
def switch_start_button(data):
    print("Start Button Pressed!")
    reset_all_components()
    change_mode("Normal_Play")

# Capture ros shutdown
def signal_handler():
        print('\nExiting...')

        # Turn off all lights
        for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
            for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
                pin_off_pub.publish(curr_light.pin)

        # Turn off flippers
        flipper_off(myPlay.left_flipper)
        flipper_off(myPlay.right_flipper)  
        
        # Turn off all coils
        for coil in myPlay.coils:
            pin_off_pub.publish(coil.pin)
        
        schedule.shutdown()

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
# TOP
switch_top_0_sub = rospy.Subscriber("switch_top_0_triggered", Bool, switch_top_0)
switch_top_1_sub = rospy.Subscriber("switch_top_1_triggered", Bool, switch_top_1)
switch_top_2_sub = rospy.Subscriber("switch_top_2_triggered", Bool, switch_top_2)
switch_top_3_sub = rospy.Subscriber("switch_top_3_triggered", Bool, switch_top_3)
switch_top_4_sub = rospy.Subscriber("switch_top_4_triggered", Bool, switch_top_4)
switch_top_5_sub = rospy.Subscriber("switch_top_5_triggered", Bool, switch_top_5)

# MID
switch_mid_0_sub = rospy.Subscriber("switch_mid_0_triggered", Bool, switch_mid_0)
switch_mid_1_sub = rospy.Subscriber("switch_mid_1_triggered", Bool, switch_mid_1)
switch_mid_2_sub = rospy.Subscriber("switch_mid_2_triggered", Bool, switch_mid_2)
switch_mid_3_sub = rospy.Subscriber("switch_mid_3_triggered", Bool, switch_mid_3)
switch_mid_4_sub = rospy.Subscriber("switch_mid_4_triggered", Bool, switch_mid_4)
switch_mid_5_sub = rospy.Subscriber("switch_mid_5_triggered", Bool, switch_mid_5)

# BOT
switch_bot_0_sub = rospy.Subscriber("switch_bot_0_triggered", Bool, switch_bot_0)
switch_bot_1_sub = rospy.Subscriber("switch_bot_1_triggered", Bool, switch_bot_1)
switch_bot_2_sub = rospy.Subscriber("switch_bot_2_triggered", Bool, switch_bot_2)
switch_bot_3_sub = rospy.Subscriber("switch_bot_3_triggered", Bool, switch_bot_3)
switch_bot_4_sub = rospy.Subscriber("switch_bot_4_triggered", Bool, switch_bot_4)
switch_bot_5_sub = rospy.Subscriber("switch_bot_5_triggered", Bool, switch_bot_5)
switch_bot_6_sub = rospy.Subscriber("switch_bot_6_triggered", Bool, switch_bot_6)
switch_bot_7_sub = rospy.Subscriber("switch_bot_7_triggered", Bool, switch_bot_7)
switch_bot_8_sub = rospy.Subscriber("switch_bot_8_triggered", Bool, switch_bot_8)

# ROS subscirber that checkes when the start button is pressed
switch_start_button_sub = rospy.Subscriber("switch_start_button_triggered", Bool, switch_start_button)

# ROS publishers to turn on or off lights
pin_on_pub = rospy.Publisher('pin_on', Int32, queue_size=10)
pin_off_pub = rospy.Publisher('pin_off', Int32, queue_size=10)

# ROS publisher to update score
update_score_pub = rospy.Publisher('update_score', Int32, queue_size=10)
update_bonus_pub = rospy.Publisher('update_bonus', Int32, queue_size=10)

# ROS publisher to update GUI message
update_message_pub = rospy.Publisher('update_message', String, queue_size=10)

# ROS publisher for when a new switch is hit
switch_list_pub = rospy.Publisher('switch_list', Int32MultiArray, queue_size=10)

# Flipper Publisher
flipper_pub = rospy.Publisher('flip_flipper', Int32, queue_size=10)

# Flipper Subscribers
flipper_sub = rospy.Subscriber('internal_flip_flipper', flip_flipper, flip_flipper_callback)

# Scheduler to keep track of when we want to turn on.off devices on the playfield
schedule = BackgroundScheduler()
schedule.start()

if __name__ == "__main__":
    myPlay.setup_pins()

    # Make sure everything is off at startup
    for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
        for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
            pin_off_pub.publish(curr_light.pin)

    # Turn off flippers
    flipper_off(myPlay.left_flipper)
    flipper_off(myPlay.right_flipper)  
        
    # Turn off all coils
    for coil in myPlay.coils:
        pin_off_pub.publish(coil.pin)

    '''
    while not rospy.is_shutdown():
        print(myPlay.switch_list)
        switch_bot_0(True)
        time.sleep(2)
        switch_mid_0(True)
        time.sleep(2)
    '''

    rate = rospy.Rate(10)

    '''
    local_override_light("Blink_Slow", myPlay.lights["top"][0])
    local_override_light("Blink_Med", myPlay.lights["mid"][0])
    local_override_light("Blink_Fast", myPlay.lights["bot"][0])
    
    time.sleep(.5)

    local_override_light("Blink_Slow", myPlay.lights["top"][0])
    local_override_light("Blink_Slow", myPlay.lights["mid"][0])
    local_override_light("Blink_Slow", myPlay.lights["bot"][0])
    '''
    change_mode("Idle")

    checking_highscore = False

    # Keep the scheduler in a loop
    while not rospy.is_shutdown():
        if myPlay.mode == "Idle":
            # TODO: Change this to be by pin number maybe? that way I can do one loop?
            print("Game Setting Up")
            i = 1
            for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
                for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
                    i += 1
                    if i % 2 == 0 and curr_light.pin != -1:
                        local_override_light("Blink_Slow", light=curr_light)
            i = 1
            time.sleep(1)
            for row in myPlay.lights: # for every row in the playfield (top, mid, bot)...
                for curr_light in myPlay.lights[row]: # ...and for every element 'i' in that row...
                    i += 1
                    if i % 2 == 1 and curr_light.pin != -1:
                        local_override_light("Blink_Slow", light=curr_light)
            print("Done Setting up")
            #local_override_light("Blink_Med", light=myPlay.lights["top"][0])
            change_mode("Idle_Waiting")
            reset_all_components()
            myPlay.setup_pins()
            change_mode("Normal_Play")
            
        if myPlay.mode == "Normal_Play":
            pass

        if myPlay.mode == "Final_Screen":
            print("Congradulations! Your score is: " + str(myPlay.score))
            if(checking_highscore):
                user_name = raw_input("Please enter your name and we will tell you if you scored a high score\n")
                myPlay.check_high_score(user_name, myPlay.score)
            reset_all_components()
            change_mode("Idle")

        #if myPlay.mode == "High_Score":


        rate.sleep()
