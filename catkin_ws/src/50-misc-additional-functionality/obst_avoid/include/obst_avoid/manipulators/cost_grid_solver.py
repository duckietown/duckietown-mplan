from obst_avoid.containers import Obstacle
from obst_avoid.containers import CostGrid
from obst_avoid.containers import Trajectory

import rospy

import duckietown_msgs.msg as dtmsg


class CostGridSolver:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        pass

    def __del__(self):
        pass

    def solve(self, cost_grid, actor_position):
        """
        Find the optimal path through the cost grid and save the path in a
        trajectory

        Parameters
        ----------
        cost_grid : CostGrid
            a populated cost grid

        Returns
        -------
        Trajectory
            a trajectory corresponding to the optimal path
        """

        assert cost_grid.isPopulated()

        trajectory = Trajectory()

        #@Victor remove the following, this was just for test
        trajectory.start_time = rospy.Time.now()
        trajectory.duration = 5
        trajectory.ts = 0.1
        vector = dtmsg.Vector2D()
        vector.x = 2
        vector.y = 2
        trajectory.positions = {vector, vector}
        trajectory.times = [0,1]

        # TODO add values to trajectory

        return trajectory
