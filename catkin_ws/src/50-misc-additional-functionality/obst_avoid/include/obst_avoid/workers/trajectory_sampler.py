from worker_base import WorkerBase

import rospy
from std_msgs.msg import Empty


class TrajectorySampler(WorkerBase):
    """
    The trajectory sampler. It gets a trajectory and samples it at the current
    time point. From this sample it then calculates the kinematic commands which
    can be passed on to the drive controller.
trajectory_sampler
    Parameters
    ----------
    trajectory: Trajectory
        the most recent trajectory as calculated by the trajectory creator. A
        subscriber will always save the newest published trajectory to this
        member object
    """

    def __init__(self, standalone=True, frequency=-1):
        """
        Call constructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectorySampler, self).__init__(standalone, frequency)

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectorySampler, self).__del__()

    def init(self):
        """
        Initialise all members of class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.trajectory = []

    def initIO(self):
        """
        Instantiate all input / output behaviour of worker. This mainly
        includes ros-publishers / -subscribers and advertised services.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        # TODO
        self.trajectory_sub = rospy.Subscriber(
            'obst_avoid/trajectory', Empty, self.trajectoryCb)

        # TODO
        self.command_pub = rospy.Publisher(
            'default_topic_name', Empty, queue_size=10)

    def trajectoryCb(self, data):
        # TODO add proper saving of trajectory
        self.trajectory = []

    def advance(self, Ts=1.0):
        """
        Grabs the current obstacle list, stored in self.obstacle_list and uses
        this to populate the cost grid. The cost grid is then passed to the
        trajectory solver which finds an optimal path and stores it as a
        trajectory. This trajectory is then published.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """

        command_msg = Empty()  # TODO, add proper message and populate it
        self.command_pub.publish(command_msg)

    def shutdown(self):
        """
        Clean up class before process end.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        pass
