import math
import rospy
from visualization_msgs.msg import Marker



class StreetObstruction:
    """
    Represents a street obstruction in the world frame


    Parameters
    ----------
    x_start: float
        start x-position of the obstacle

    y_start: float
        start y-position of the obstacle

    x_dim: float
        max length of object in x direction

    y_dim: float
        max length of object in y direction

    z_dim: float
        max length of object in z direction

    x_end: float
        end x-position of the obstacle

    y_end: float
        end y-position of the obstacle

    time: float
        object should be around
    id: int
        unique identification
    """

    def __init__(self, x_start=0, y_start=0, x_dim=0.1, y_dim=0.1, z_dim=0.1, x_end=0, y_end=0, time=5, id=-1):
        """
        Represents a street obstruction in the world frame


        Parameters
        ----------
        x_start: float
            start x-position of the obstacle

        y_start: float
            start y-position of the obstacle

        x_dim: float
            max length of object in x direction

        y_dim: float
            max length of object in y direction

        z_dim: float
            max length of object in z direction

        x_end: float
            end x-position of the obstacle

        y_end: float
            end y-position of the obstacle

        time: float
            object should be around
        id: int
            identification
        """
        self.x_start = x_start
        self.y_start = y_start
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.x_end = x_end
        self.y_end = y_end
        self.time = time
        self.id = id
        self.x_dist = self.x_end - self.x_start
        self.y_dist = self.y_end - self.y_start
        self.x_vel = self.x_dist / self.time
        self.y_vel = self.y_dist / self.time
        now = rospy.get_time()
        self.start_time = now
        self.pre_time = self.start_time
        self.x_prev = self.x_start
        self.y_prev = self.y_start


    def __del__(self):
        pass

    def __str__(self):
        return 'start position: ({} / {}) \n dimension: ({} / {} / {}) \n end position: ({} / {}) \n time: {} \n id: {}'.format(self.x_start, self.y_start, self.x_dim, self.y_dim, self.z_dim, self.x_end,
self.y_end, self.time, self.id)


    def update(self):
        # print('street obstruction, do nothing')

        return


    def get(self):

        return self.x_start, self.y_start, self.x_dim, self.y_dim, self.z_dim

    def getMsg(self):

        marker = Marker()

        marker.header.frame_id = "/map"
        marker.id = self.id
        marker.ns = "street_obstruction"

        marker.type = marker.CUBE
        marker.action = marker.ADD

        cur_time = rospy.get_time()
        dt = cur_time - self.start_time
        dt = dt % self.time

        marker.pose.position.x = self.x_start + dt*self.x_vel
        marker.pose.position.y = self.y_start + dt*self.y_vel
        marker.pose.position.z = self.z_dim/2.0

        marker.scale.x = self.x_dim
        marker.scale.y = self.y_dim
        marker.scale.z = self.z_dim

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0


        return marker
