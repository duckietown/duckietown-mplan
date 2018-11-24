import obstacle
import cost_grid
import trajectory


class CostGridSolver:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        pass

    def __del__(self):
        pass

    def solve(cost_grid):
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
        # TODO add values to trajectory

        return trajectory
