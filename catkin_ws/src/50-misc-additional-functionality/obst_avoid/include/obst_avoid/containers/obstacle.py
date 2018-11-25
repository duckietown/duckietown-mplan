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

    def __init__(self, x=0, y=0, r=0, x_dot=0, y_dot=0):
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

    def __del__(self):
        pass

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
        pass

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
        self.radius = msg.safety_radius
        self.x_dot = msg.twist.x
        self.y_dot = msg.twist.y

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
        return self.x, self.y, self.y_dot, self.y_dot
