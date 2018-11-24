import rospy
import trajectory_creator


def main():
    rospy.init_node('trajectory_creator_node', anonymous=False)

    # instantiate trajectory creator at max frequency
    trajectory_creator = TrajectoryCreator(-1, standalone=True)


if __name__ == '__main__':
    main()
