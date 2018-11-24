import rospy
import trajectory_sampler


def main():
    rospy.init_node('trajectory_sampler_node', anonymous=False)

    # instantiate standalone trajectory sampler with 10 hz
    trajectory_sampler = TrajectorySampler(10, standalone=True)


if __name__ == '__main__':
    main()
