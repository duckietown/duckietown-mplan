import sympy as sp
import math
from duckietown_msgs.msg import MovingObject

class Obstacle:
    """
    Represents an obstacle in the world frame

    # TODO decide in which frame these parameters are given
    # add type specifier and object specifier as member


    Parameters
    ----------
    x: float
        x-position of the obstacle

    y: float
        y-position of the obstacle

    radius: float
        bounding box radius of the obstacle

    x_dot: float
        x-velocity of the obstacle

    y_dot: float
        y-velocity of the obstacle
    """

    def __init__(self, x=0, y=0, r=0.1, x_dot=0, y_dot=0):
        """
        Construct an obstacles with the given parameters

        Parameters
        ----------
        x: float
            x-position of the obstacle

        y: float
            y-position of the obstacle

        radius: float
            bounding box radius of the obstacle

        x_dot: float, optional
            x-velocity of the obstacle

        y_dot: float, optional
            y-velocity of the obstacle
        """
        self.x = x
        self.y = y
        self.radius = r
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.theta = 0

        self.init_obstacles_fun()

    def __del__(self):
        pass

    def __str__(self):
        return 'position: ({} / {}) \n velocity: ({} / {}) \n radius: {}'.format(self.x, self.y, self.x_dot, self.y_dot, self.radius)

    def init_obstacles_fun(self):
        x = sp.Symbol('x')
        y = sp.Symbol('y')
        obs_x = sp.Symbol('obs_x')
        obs_y = sp.Symbol('obs_y')
        obs_x_dot = sp.Symbol('obs_x_dot')
        obs_y_dot = sp.Symbol('obs_y_dot')
        t = sp.Symbol('t')

        max_cost = 100 # cost at radius = max_cost/2
        function_degree = 20

        obstacle_cost_fun = sp.Function('obstacles_fun')
        obstacle_cost_fun = 2*max_cost * 2**(-((((x + t*obs_x_dot - obs_x)**2 + (y + t*obs_y_dot - obs_y)**2)/self.radius**2)**(function_degree*self.radius/2)))


        self.getCost = sp.lambdify([x, y, t, obs_x, obs_y, obs_x_dot, obs_y_dot ], obstacle_cost_fun)

        # DEBUG VISUALIZATIONS
        # print "init obstacle fun"
        # print self.obstacles_fun
        # self.obstacles_fun.subs([(t,10)])
        # sp.plotting.plot3d(self.obstacles_fun, (x, 0, 1.5), (y, -0.2, 0.2), xlim=[-0.1,1.5], ylim=[-0.3,0.3])


    def toMsg(self):
        """
        convert the instance to a ros message which can be published

        Parameters
        ----------
        none

        Returns
        -------
        duckietown_msgs.msg.MovingObject
            the message containing all information from the instance of this
            class
        """
        msg = MovingObject()
        msg.pose.x = self.x
        msg.pose.y = self.y
        msg.twist.x = self.x_dot
        msg.twist.y = self.y_dot
        msg.safety_radius = self.r

    def fromMsg(self, msg):
        """
        so to say a copy constructor from a msg

        Parameters
        ----------
        msg: duckietown_msgs.msg.MovingObject

        Returns
        -------
        none
        """
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta
        self.x_dot = msg.twist.x
        self.y_dot = msg.twist.y
        self.radius = msg.safety_radius

    def getState(self):
        """
        Get the pose and twist of the obstacle directly

        Parameters
        ----------
        none

        Returns
        -------
        float
            the x position
        float
            the y position
        float
            the x velocity
        float
            the y velocity
        """
        return self.x, self.y, self.x_dot, self.y_dot
