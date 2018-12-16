#!/usr/bin/env python

import rospy
from obst_avoid_viz import PathViz


def main():
    rospy.init_node('path_viz_node', anonymous=False)

    # instantiate standalone worker with 20 hz
    path_viz = PathViz(standalone=True, frequency=20)


if __name__ == '__main__':
    main()
