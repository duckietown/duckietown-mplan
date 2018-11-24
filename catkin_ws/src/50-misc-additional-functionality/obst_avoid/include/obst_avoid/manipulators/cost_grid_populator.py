from obst_avoid.containers import Obstacle
from obst_avoid.containers import CostGrid


class CostGridPopulator:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        pass

    def __del__(self):
        pass

    def populate(self, actor_position, list_of_obstacles):
        """
        Create a cost grid and populate it according to the obstacles

        Parameters
        ----------
        actor_position : undefinde position object #TODO
            an object containing the position and speed of acting duckiebot
        list_of_obstacles: obstacle[]
            a list of obstacle objects know to be around

        Returns
        -------
        CostGrid : a cost grid where each grid point has an assigned cost
        """

        cost_grid = CostGrid(0, 0)  # TODO init cost grid with correct size

        # TODO populate the grid

        cost_grid.populated = True
        return cost_grid
