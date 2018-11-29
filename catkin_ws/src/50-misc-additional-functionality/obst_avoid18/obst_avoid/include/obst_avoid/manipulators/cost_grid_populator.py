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
            a list of obstacle objects known to be around

        Returns
        -------
        CostGrid : a cost grid where each grid point has an assigned cost
        """

        cost_grid = CostGrid()

        # state optimization truncation values
        x_min = -1
        x_max = 1
        y_min = 0
        y_max = 2

        # iterate through obstacles and add a cost to each location
        for o in list_of_obstacles:
            y_o = o.x # x and y are flipped for random number generation
            x_o = o.y
            if x_min<=x_o and x_o<=x_max and y_min<=y_o and y_o<=y_max:
                cost_grid.costs.nodes[(x_o, y_o)]['node_weight'] =  1

        cost_grid.populated = True
        return cost_grid
