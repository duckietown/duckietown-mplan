from obst_avoid.containers import Obstacle
from obst_avoid.containers import CostGrid
from obst_avoid.containers import Trajectory

import rospy
import networkx as nx

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

        # takes edge AND node costs into account
        def weight_func(u, v, d):
            node_u_wt = cost_grid.costs.nodes[u].get('node_weight', 0)
            node_v_wt = cost_grid.costs.nodes[v].get('node_weight', 0)
            edge_wt = d.get('weight', 0)
            return node_u_wt/2. + node_v_wt/2. + edge_wt

        # solve the SP problem and print solution for verification
        path = nx.dijkstra_path(cost_grid.costs, (0,0), (0,2), weight_func)
        print(path)

        # output solution to trajectory object
        trajectory.start_time = rospy.Time.now()
        trajectory.duration = 5
        trajectory.ts = 0.1
        trajectory.positions = path
        trajectory.times = range(0, len(path))

        return trajectory
