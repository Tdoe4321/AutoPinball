#!/usr/bin/env python

from pynput.keyboard import Key, Listener 
import rospy

from AutoPinball.msg import flip_flipper

import signal, sys

publish_flipper = rospy.Publisher('internal_flip_flipper', flip_flipper, queue_size=10)

def on_press(key):
    #print('{0} pressed'.format(key))
    if key == Key.right:
        publish_flipper.publish(2, 0)
    elif key == Key.left:
        publish_flipper.publish(1, 0)

def on_release(key):
    if key == Key.right:
        publish_flipper.publish(-2, 0)
    elif key == Key.left:
        publish_flipper.publish(-1, 0)
    elif key == Key.esc:
        # Stop listener
        return False

def signal_handler(sig, frame):
    return False

if __name__ == "__main__":
    rospy.init_node("Keybaord_Flippers")
    

    with Listener(on_press=on_press, on_release=on_release) as listner:
        signal.signal(signal.SIGINT, signal_handler)
        listner.join()
