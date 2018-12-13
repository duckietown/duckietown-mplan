import rospy
from std_msgs.msg import Empty
import duckietown_msgs.msg as dtmsg
import os
import numpy as np
import tf
import math

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
        self.tile_size = 0.585
        self.actor.x = rospy.get_param('x_pos_set')*self.tile_size
        self.actor.y = rospy.get_param('y_pos_set')*self.tile_size
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
        self.actor_sub = rospy.Subscriber(
            'obst_avoid/actor', dtmsg.Actor, self.actorCb)

        self.obstacle_sub = rospy.Subscriber(
            'obst_avoid/obstacles', dtmsg.Obstacles, self.obstacleCb)

        self.trajectory_pub = rospy.Publisher(
            'obst_avoid/trajectory', dtmsg.TimedPath, queue_size=10)

        self.cost_grid_viz_pub = rospy.Publisher(
            'obst_avoid/cost_grid', MarkerArray, queue_size=10)

        self.tile_pub = rospy.Publisher(
            'obst_avoid/tiles', MarkerArray, queue_size=10)


        map = rospy.wait_for_message("duckietown_map", MarkerArray)
        self.parseMapToTileList(map)
        self.tile_current = self.findClosestTile(rospy.get_param('x_pos_set')*self.tile_size, rospy.get_param('y_pos_set')*self.tile_size)
        self.tile_next = self.getNextTile(self.tile_current)

    def actorCb(self, data):
        self.actor.fromMsg(data.moving_object)

    def obstacleCb(self, data):
        self.obstacle_list = []
        for obstacle_msg in list(data.moving_objects):
            new_obstacle = Obstacle()
            new_obstacle.fromMsg(obstacle_msg)

            self.obstacle_list.append(new_obstacle)

    def mapCb(self, data):
        self.map_sub.unregister()
        self.parseMapToTileList(data)

    def mapToPositiveAngle(self, angle):
        if angle > 2*math.pi:
            angle -= 2*math.pi
        elif angle < -2*math.pi:
            angle += 2*math.pi
        return angle

    def parseMapToTileList(self, map):
        # find the closest tile and return its position and orientation
        self.tiles = []
        for elem in map.markers:
            if elem.ns == 'tiles':
                name = os.path.basename(os.path.normpath(elem.mesh_resource))
                if name != 'asphalt.dae':
                    quaternion = [elem.pose.orientation.x, elem.pose.orientation.y, elem.pose.orientation.z, elem.pose.orientation.w]
                    yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
                    yaw = self.mapToPositiveAngle(yaw)
                    self.tiles.append({'position' : [elem.pose.position.x, elem.pose.position.y, yaw], 'type' : name, 'entry_angle' : yaw})

    def findClosestTile(self, x, y):
        tile_positions = [elem['position'] for elem in self.tiles]
        tile_positions = np.asarray(tile_positions)
        deltas = tile_positions[:,:2] - [x, y]
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return self.tiles[np.argmin(dist_2)]

    def distToTile(self, x, y, tile):
        return ((x-tile['position'][0])**2 + (y-tile['position'][1])**2)**0.5

    def distVecFromTile(self, x, y, tile):
        return np.asarray([x-tile['position'][0], y-tile['position'][1]])

    def getUnitVecFromTheta(self, theta, length=1):
        return np.asarray([math.cos(theta), math.sin(theta), 0])*length

    def getNextTile(self, tile):
        delta_theta = tile['entry_angle']-tile['position'][2]
        delta_theta = self.mapToPositiveAngle(delta_theta)

        xy = [0,0,0]
        next_entry_angle = 0

        if tile['type']=='straight.dae':
            next_entry_angle = tile['entry_angle']

        elif tile['type']=='curve_left.dae':
            if delta_theta == math.pi*2/2:
                next_entry_angle = tile['entry_angle']+math.pi/2
            if delta_theta == math.pi/2:
                next_entry_angle = tile['entry_angle']-math.pi/2

        elif tile['type']=='curve_right.dae':
            if delta_theta == math.pi*2/2:
                next_entry_angle = tile['entry_angle']-math.pi/2
            if delta_theta == math.pi*3/2:
                next_entry_angle = tile['entry_angle']+math.pi/2

        elif tile['type']=='4way.dae':
            next_entry_angle = tile['entry_angle']

        elif tile['type']=='3way_left.dae':
            next_entry_angle = tile['entry_angle']

        elif tile['type']=='3way_right.dae':
            next_entry_angle = tile['entry_angle']

        else:
            print('kakcki')

        xy = tile['position']+self.getUnitVecFromTheta(next_entry_angle, self.tile_size)
        next_tile = self.findClosestTile(xy[0], xy[1])
        next_tile['entry_angle'] = next_entry_angle
        return next_tile

    def getMarker(self, marker_id, tile):
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.id = marker_id
        marker.ns = "tile_markers"

        marker.type = marker.ARROW
        marker.action = marker.ADD

        marker.pose.position.x = tile['position'][0]
        marker.pose.position.y = tile['position'][1]
        marker.pose.position.z = 0.2

        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        q = tf.transformations.quaternion_from_euler(0, 0, tile['entry_angle'])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0/marker_id
        marker.color.a = 1.0

        return marker

    def publishTiles(self, tile1, tile2):
        marker_msg = MarkerArray()
        marker_msg.markers.append(self.getMarker(1, tile1))
        marker_msg.markers.append(self.getMarker(2, tile2))
        self.tile_pub.publish(marker_msg)

    def rotation_matrix(self, theta, axis=[0,0,1]):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians.
        """
        axis = np.asarray(axis)
        axis = axis / math.sqrt(np.dot(axis, axis))
        a = math.cos(theta / 2.0)
        b, c, d = -axis * math.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                         [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                         [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

    def getCostGridOrigin(self, tile_current, tile_next, actor):
        type = tile_current['type']

        if type == 'straight.dae':
            d = self.distVecFromTile(actor.x, actor.y, tile_current)
            e = self.getUnitVecFromTheta(tile_current['entry_angle'])[:2]
            offset = np.dot(d, e)*e
            cost_grid_origin = [tile_current['position'][0]+offset[0], tile_current['position'][1]+offset[1], tile_next['entry_angle']]

        elif type == 'curve_left.dae':
            # tile position in world frame
            w_P_t = tile_current['position'][:2]

            # duckiebot positon in world frame
            w_P_d = np.array([actor.x, actor.y])

            # corner position in tile frame TODO check
            self.rotation_matrix(tile_current['position'][2])
            t_P_c = np.dot(self.rotation_matrix(tile_current['position'][2]), np.array([-self.tile_size/2, self.tile_size/2, 0]))[:2]

            # corner position in world frame
            w_P_c = w_P_t + t_P_c

            # unit radius vector
            radius = w_P_d - w_P_c
            unit_radius = radius / np.linalg.norm(radius)

            # get positional cost_grid_origin
            cost_grid_origin = w_P_c + self.tile_size/2 * unit_radius

            # get theta angle of cost_grid and append to
            theta = self.mapToPositiveAngle(2*(tile_current['entry_angle']-tile_current['position'][2]+3/4*math.pi))

            cost_grid_origin = np.append(cost_grid_origin, [theta])


        elif type == 'curve_right.dae':
            # tile position in world frame
            w_P_t = tile_current['position'][:2]

            # duckiebot positon in world frame
            w_P_d = np.array([actor.x, actor.y])

            # corner position in tile frame TODO check
            self.rotation_matrix(tile_current['position'][2])
            t_P_c = np.dot(self.rotation_matrix(tile_current['position'][2]), np.array([-self.tile_size/2, -self.tile_size/2, 0]))[:2]

            # corner position in world frame
            w_P_c = w_P_t + t_P_c

            # unit radius vector
            radius = w_P_d - w_P_c
            unit_radius = radius / np.linalg.norm(radius)

            # get positional cost_grid_origin
            cost_grid_origin = w_P_c + self.tile_size/2 * unit_radius

            # get theta angle of cost_grid and append to
            theta = self.mapToPositiveAngle(2*(tile_current['entry_angle']-tile_current['position'][2]+1/4*math.pi))

            cost_grid_origin = np.append(cost_grid_origin, [theta])

        else:
            d = self.distVecFromTile(self.actor.x, self.actor.y, self.tile_current)
            e = self.getUnitVecFromTheta(self.tile_current['entry_angle'])[:2]
            offset = np.dot(d, e)*e
            cost_grid_origin = [self.tile_current['position'][0]+offset[0], self.tile_current['position'][1]+offset[1], self.tile_next['entry_angle']]

        return cost_grid_origin

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

        if self.distToTile(self.actor.x, self.actor.y, self.tile_current) > self.distToTile(self.actor.x, self.actor.y, self.tile_next):
            print('update called on tile shift')
            self.tile_current = self.tile_next
            self.tile_next = self.getNextTile(self.tile_current)

        cost_grid_origin = self.getCostGridOrigin(self.tile_current, self.tile_next, self.actor)

        entry_angle = self.tile_current['entry_angle']
        d = self.distVecFromTile(self.actor.x, self.actor.y, self.tile_current)
        e = self.getUnitVecFromTheta(self.tile_current['entry_angle']+math.pi/2)[:2]
        dist_to_centerline = np.dot(d, e)


        # get the filled cost grid
        cost_grid = self.cost_grid_populator.populate(self.actor, self.obstacle_list, self.cost_grid_params, self.max_actor_vel, cost_grid_origin, dist_to_centerline)

        # solve the cost grid for a trajectory
        trajectory = self.cost_grid_solver.solve(cost_grid, self.cost_grid_params)

        # convert the trajectory to a msg
        trajectory_msg = trajectory.toMsg()

        # publish trajectory msg
        self.trajectory_pub.publish(trajectory_msg)

        # publish cost_grid msg
        cost_grid_marker = cost_grid.toVizMsg(self.cost_grid_params)
        self.cost_grid_viz_pub.publish(cost_grid_marker)

        self.publishTiles(self.tile_current, self.tile_next)

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
