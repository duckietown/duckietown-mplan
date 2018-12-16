#!/usr/bin/env python

import rospy
from obst_avoid import TrajectorySampler


def main():
    rospy.init_node('trajectory_sampler_node', anonymous=False)

    # instantiate standalone trajectory sampler with 10 hz
    freq = rospy.get_param('/trajectory_sampler_node/frequency')
    trajectory_sampler = TrajectorySampler(standalone=True, frequency=freq)


if __name__ == '__main__':
    main()
