#!/usr/bin/env python
from obst_avoid import WorkerTemplate
import rospy


def main():
    rospy.init_node('worker_template_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    worker_template = WorkerTemplate(standalone=True, frequency=10)


if __name__ == '__main__':
    main()
