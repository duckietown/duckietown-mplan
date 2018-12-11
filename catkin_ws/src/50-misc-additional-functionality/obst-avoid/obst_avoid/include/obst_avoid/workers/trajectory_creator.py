import rospy
from std_msgs.msg import Empty
import duckietown_msgs.msg as dtmsg
import os
import numpy as np
import tf

from worker_base import WorkerBase
from obst_avoid.manipulators import CostGridPopulator
from obst_avoid.manipulators import CostGridSolver
from obst_avoid.containers import Obstacle
from obst_avoid.containers import Trajectory

from visualization_msgs.msg import MarkerArray, Marker


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
        rospy.loginfo('[TrajectoryCreator.__init__] init complete')

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
        # cost grid parameters from ros parameter server
        self.cost_grid_params =	{
            'n_t' : rospy.get_param('cost_grid/depth/time'),
            'n_x' : rospy.get_param('cost_grid/depth/x'),
            'n_y' : rospy.get_param('cost_grid/depth/y'),
            'dt' : rospy.get_param('cost_grid/delta/time'),
            'dx' : rospy.get_param('cost_grid/delta/x'),
            'dy' : rospy.get_param('cost_grid/delta/y')
        }
        self.max_actor_vel = rospy.get_param('velocity/max')

        self.cost_grid_populator = CostGridPopulator(self.cost_grid_params, self.max_actor_vel)
        self.cost_grid_solver = CostGridSolver()
        self.actor = Obstacle()
        self.obstacle_list = []
        marker = Marker()
        marker.ns='tiles'
        self.map = MarkerArray()
        self.map.markers.append(marker)

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
        self.actor_sub = rospy.Subscriber(
            'obst_avoid/actor', dtmsg.Actor, self.actorCb)

        self.obstacle_sub = rospy.Subscriber(
            'obst_avoid/obstacles', dtmsg.Obstacles, self.obstacleCb)

        self.map_sub = rospy.Subscriber(
            'duckietown_map', MarkerArray, self.mapCb)

        self.trajectory_pub = rospy.Publisher(
            'obst_avoid/trajectory', dtmsg.TimedPath, queue_size=10)

        self.cost_grid_viz_pub = rospy.Publisher(
            'obst_avoid/cost_grid', MarkerArray, queue_size=10)

    def actorCb(self, data):
        self.actor.fromMsg(data.moving_object)

    def obstacleCb(self, data):
        self.obstacle_list = []
        for obstacle_msg in list(data.moving_objects):
            new_obstacle = Obstacle()
            new_obstacle.fromMsg(obstacle_msg)

            self.obstacle_list.append(new_obstacle)

    def mapCb(self, data):
        self.map = data

    def parseMap(self):
        # find the closest tile and return its position and orientation
        pos = []
        orientation = []
        type = []
        for elem in self.map.markers:
            if elem.ns == 'tiles':
                name = os.path.basename(os.path.normpath(elem.mesh_resource))
                if name != 'asphalt.dae':
                    quaternion = [elem.pose.orientation.x, elem.pose.orientation.y, elem.pose.orientation.z, elem.pose.orientation.w]
                    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
                    orientation.append(yaw)
                    pos.append([elem.pose.position.x, elem.pose.position.y])
                    type.append(name)

        argmin = self.findClosestTile(pos)
        x = pos[argmin][0]
        y = pos[argmin][1]
        theta = orientation[argmin]
        return [x, y, theta+1.574], type[argmin]

    def findClosestTile(self, tile_positions):
        tile_positions = np.asarray(tile_positions)
        deltas = tile_positions - [self.actor.x, self.actor.y]
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

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
        origin, type = self.parseMap()
        cost_grid = self.cost_grid_populator.populate(self.actor, self.obstacle_list, self.cost_grid_params, self.max_actor_vel, origin)
        print('populated')

        # solve the cost grid for a trajectory
        trajectory = self.cost_grid_solver.solve(cost_grid, self.cost_grid_params)
        print('solved')
        # trajectory = Trajectory()
        # trajectory.start_time = rospy.Time.now()
        # trajectory.duration = 50
        # trajectory.ts = 5
        # trajectory.positions = [[0,0],[2,0],[4,0],[6,0],[10,0],[10,0]]
        # trajectory.times = [0,.2,.4,.6,.8,1]

        # convert the trajectory to a msg
        trajectory_msg = trajectory.toMsg()

        # publish trajectory msg
        self.trajectory_pub.publish(trajectory_msg)
        print('published1')

        # publish cost_grid msg
        cost_grid_marker = cost_grid.toVizMsg(self.cost_grid_params)
        self.cost_grid_viz_pub.publish(cost_grid_marker)
        print('published2')

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
