import rospy
import smach

from States.normal_play import *

def createStateMachine():
    rospy.init_node('AutoPinball_StateMachine')

    # Create the top level SMACH state machine
    sm_PINBALL = smach.StateMachine(outcomes=['game_over'])
    
    # Open the container
    with sm_PINBALL:
        smach.StateMachine.add('NORMAL_PLAY', Normal_Play(), transitions={'ball_lost':'game_over'})


    # Execute SMACH plan
    outcome = sm_PINBALL.execute()


def main():
    createStateMachine()


if __name__ == '__main__':
    main()