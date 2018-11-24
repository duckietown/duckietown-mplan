from worker_base import WorkerBase
from obst_avoid.manipulators import CostGridPopulator
from obst_avoid.manipulators import CostGridSolver

import rospy
from std_msgs.msg import Empty


class TrajectoryCreator(WorkerBase):
    """
    The trajectory creator. It gets a list of obstacles and the pose of the
    actor duckiebot. From this it first populates the cost grid and the solves
    it to find the optimal paths. This optimal path is then published as a
    trajectory.

    Parameters
    ----------
    cost_grid_populator: CostGridPopulator
        The worker used to populate a cost grid

    cost_grid_solver: CostGridSolver
        The worker used to obtain a trajectory from a cost grid

    actor: Obstacle
        obstacle object misused for saving the pose, twist and collision-
        information from the actor duckiebot.

    obstacle_list: Obstacle[]
        list of obstacles in the field of view
    """

    def __init__(self, standalone=True, frequency=-1):
        """
        Call constructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectoryCreator, self).__init__(standalone, frequency)

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(TrajectoryCreator, self).__del__()

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
        self.cost_grid_populator = CostGridPopulator()
        self.cost_grid_solver = CostGridSolver()
        self.actor = []
        self.obstacle_list = []

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
        # TODO add callback and make the subscriber save to self.actor
        self.actor_sub = rospy.Subscriber(
            'default_topic_name', Empty, self.actorCb)

        # TODO add callback and make the subscriber save to self.obstacle_list
        self.obstacle_sub = rospy.Subscriber(
            'default_topic_name', Empty, self.obstacleCb)

        # TODO add topic name, type etc
        self.trajectory_pub = rospy.Publisher(
            'obst_avoid/trajectory', Empty, queue_size=10)

    def actorCb(self, data):
        # TODO add proper saving of actor state
        self.actor = []

    def obstacleCb(self, data):
        # TODO add proper saving of obstacle state list
        self.obstacle_list = []

    def advance(self, Ts=1.0):
        """
        Grabs the trajectory object, stored in self.trajectory and samples it.
        Then publishes the calculated command.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """

        # populate the cost grid
        cost_grid = self.cost_grid_populator.populate(
            self.obstacle_list, self.actor)

        # solve the cost grid for a trajectory
        trajectory = self.cost_grid_solver.solve(cost_grid, self.actor)

        # convert the trajectory to a msg
        trajectory_msg = trajectory.toMsg()

        # publish trajectory msg
        self.trajectory_pub.publish(trajectory_msg)

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
