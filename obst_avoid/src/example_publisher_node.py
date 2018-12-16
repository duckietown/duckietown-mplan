#!/usr/bin/env python

import rospy
from obst_avoid import ExamplePublisher


def main():
    rospy.init_node('worker_template_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    example_publisher = ExamplePublisher(standalone=True, frequency=10)


if __name__ == '__main__':
    main()
