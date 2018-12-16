#!/usr/bin/env python

import rospy
from obst_avoid import TrajectoryCreator


def main():
    rospy.init_node('trajectory_creator_node', anonymous=False)

    # instantiate standalone trajectory creator at max frequency
    freq = rospy.get_param('/trajectory_creator_node/frequency')
    trajectory_creator = TrajectoryCreator(standalone=True, frequency=freq)


if __name__ == '__main__':
    main()
