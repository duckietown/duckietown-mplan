#!/usr/bin/env python

import rospy
from obst_avoid import TrajectoryCreator


def main():
    rospy.init_node('trajectory_creator_node', anonymous=False)

    # instantiate standalone trajectory creator at max frequency
    trajectory_creator = TrajectoryCreator(standalone=True, frequency=10)


if __name__ == '__main__':
    main()
