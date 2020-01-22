import sys
sys.path.append('/home/tdoe321/projects/AutoPinball/Classes') # Bad Tyler - No hard coded paths!

from playfield import Playfield

import rospy

if __name__ == "__main__":
    myPlay = Playfield()

    print(myPlay.lights["top"][0].on)
    print(myPlay.switches["mid"][2].time_since_last_on)