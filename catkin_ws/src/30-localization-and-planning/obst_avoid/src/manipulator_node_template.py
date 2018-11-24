from obst_avoid import manipulator_template
import rospy


def main():
    rospy.init_node('manipulator_template_node', anonymous=False)

    # instantiate standalone manipulator with at 10 hz
    manipulator_template = ManipulatorTemplate(10, standalone=True)


if __name__ == '__main__':
    main()
