from pynput.keyboard import Key, Listener 
import rospy

from std_msgs.msg import Int32

rospy.init_node("Keybaord_Flippers")

publish_flipper = rospy.Publisher('internal_flip_flipper', Int32, queue_size=10)

def on_press(key):
    #print('{0} pressed'.format(key))
    if key == Key.right:
        publish_flipper.publish(2)
    elif key == Key.left:
        publish_flipper.publish(1)

def on_release(key):
    if key == Key.right:
        publish_flipper.publish(-2)
    elif key == Key.left:
        publish_flipper.publish(-1)
    elif key == Key.esc:
        # Stop listener
        return False

with Listener(on_press=on_press, on_release=on_release) as listner:
    listner.join()