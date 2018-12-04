from obst_avoid.containers import Obstacle
from obst_avoid.containers import CostGrid


class CostGridPopulator:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        pass

    def __del__(self):
        pass

    def connectGraph(graph, actor_x, actor_y):
        """
        connect the graph of an obstacle grid

        Parameters
        ----------
        graph : networkx.Graph
            the graph to be connected
        actor : containers.Obstacle
            state object of the obstacle

        Returns
        -------
        networkx.Graph : the connected graph
        """
        pass

    def populate(self, actor_position, list_of_obstacles, max_actor_vel):
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

        connectGraph(cost_grid.nodes, actor_position.x, actor_position.y)

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

    def getCost(x, y, t, obstacle_list):
        """
        return the value of the costfunction at a specific time point

        Parameters
        ----------
        x : float
            x position of the requested cost value
        y : float
            y position of the requested cost value
        t : float
            time of requested cost value
        obstacle_list : containers.Obstacle[]
            list of obstacles state objects

        Returns
        -------
        float : the requested cost
        """
        out = 0
        for obstacle in obstacle_list:
            # attention: getCost currently only returns 0
            out += obstacle.getCost(x,y,t)

        #TODO add cost of cake
        #out += cake_cost

        #TODO add cost of street boundaries
        #out += street_cost
        return out
