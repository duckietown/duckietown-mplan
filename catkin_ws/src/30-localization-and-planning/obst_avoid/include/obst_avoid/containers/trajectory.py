import numpy as np
import rospy


class Trajectory:
    """
    Container object for representation of a generic (timed) 2d trajectory.

    The trajectory is defined to start at time self.start_time and be of duration
    self.duration. The theoretical setpoint values are stored in the
    self.positions array. The normalized time corresponding to each trajectory
    point is saved in the self.times array. The bounds for the trajectory values
    are saved in the self.lower_bounds / self.upper_bounds arrays.

    For more information on the member variables see their specific
    documentation.

    Parameters
    ----------
    start_time: rospy.Time
        time object represanting the start time of trajectory

    duration: float
        exact duration which the whole trajectory should take to execute. Has to
        be greater than 0.

    ts: float
        The timestep duration between consecutive points of trajectory.
        duration / ts = n.

    positions: numpy.ndarray<float>
        n x 2 numpy array containing the setpoint positions of the trajectory.
        First column corresponds to x values, second column to y values.

    times: numpy.ndarray<float>
        n x 1 numpy array containing the temporal setpoint of each trajectory
        point. Values are scaled to unit duration, meaning the first times entry
        will always be 0, the last 1.

    lower_bounds: numpy.ndarray<float>
        n x 2 numpy array containing the positional lower bounds of the
        trajectory. First column corresponds to x values, second column to y
        values.

    upper: numpy.ndarray<float>
        n x 2 numpy array containing the positional upper bounds of the
        trajectory. First column corresponds to x values, second column to y
        values.
    """

    def __init__(self):
        self.start_time = 0
        self.duration = 0
        self.ts = 0
        self.positions = []
        self.times = []
        self.lower_bounds = []
        self.upper_bounds = []

    def __del__(self):
        pass

    def getPositionFromTimePoint(time_point):
        """
        get the positional set point corresponding to the unnormalized time t.

        Parameters
        ----------
        time_point: rospy.Time
            the time point for which the trajectory set point shall be returned

        Returns
        -------
        numpy.ndarray<float>
            1 x 2 numpy array containing the x and y positional set point
            correspoding to the given time.
        """
        # convert the time_point to a duration
        time_duration = (time_point - self.start_time).to_sec()

        # call the main getPosition method with the duration argument
        return getPosition(time_duration)

    def getPosition(time_duration):
        """
        get the positional set point corresponding to the unnormalized time t.

        Parameters
        ----------
        time_duration: float
            the unnormalized duration in seconds since the start of the
            trajectory

        Returns
        -------
        numpy.ndarray<float>
            1 x 2 numpy array containing the x and y positional set point
            correspoding to the given time.
        """
        # TODO
        pass

    def toMsg():
        """
        convert the instance to a ros message which can be published

        Parameters
        ----------
        none

        Returns
        -------
        trajectory_msg #TODO define this message
            the message containing all information from the instance of this
            class
        """
        # TODO
        pass
