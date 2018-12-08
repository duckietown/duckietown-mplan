from obst_avoid.containers import Obstacle
from obst_avoid.containers import CostGrid
import math


class CostGridPopulator:
    """Populates the cost grid from the existing objects"""

    def __init__(self):
        """
        initialize cost_grid object

        Parameters
        ----------
        empty

        Returns
        -------
        networkx.Graph : the connected graph
        """
        self.cost_grid = CostGrid()

        # add nodes to cost_grid object
        for k in range(cost_grid_params.get('n_t')):
            for i in range(cost_grid_params.get('n_x')):
                for j in range(cost_grid_params.get('n_y')):
                    x_pos=i*cost_grid_params.get('d_x')
                    y_pos=(j-1)/2.0*cost_grid_params.get('d_y') # for centered coordinate system
                    t_pos=k*cost_grid_params.get('d_t')
                    self.cost_grid.costs.add_node((i, j, k), x_pos=x_pos, y_pos=y_pos, t_pos=t_pos, node_weight=0.0)

    def __del__(self):
        pass

    def connectGraph(self, graph, actor_x, actor_y, cost_grid_params, max_actor_vel):
        """
        connect the graph of an obstacle grid

        Parameters
        ----------
        graph : networkx.Graph
            the graph to be connected
        actor : containers.Obstacle
            state object of the actor
        max_actor_vel: float
            the maximum velocity in [m/s] of a duckiebot

        Returns
        -------
        networkx.Graph : the connected graph
        """
        # connect start node (find closest node index to current actor position) to first layer (layer 0)
        k_0 = 0
        i_0 = 0
        j_0 = int(round(actor_y/cost_grid_params.get('d_y')*2.0 + 1))
        self.cost_grid.costs.add_weighted_edges_from([('S', (i_0,j_0,k_0), 0.0)])

        # connect layers from layer 0 to layer n_t-1
        # initialize first iteration
        k_n = k_0
        i_n = i_0
        j_n = j_0
        active_nodes = [(i_n, j_n, k_n)]

        # instantiate mask
        mask = [(0,0), (1,0), (-1,0), (0,1), (0, -1), (1,1), (-1,1), (1, -1), (-1, -1), (0,2), (0,-2)]

        # iterate
        for k in range(cost_grid_params.get('n_t')):
            next_nodes = []
            for current_node in active_nodes:
                for mask_element in mask:
                    mask_x = i_n + mask_element[0]
                    mask_y = j_n + maks_element[1]
                    curr_node = (i_n, j_n, k_n)
                    next_node = (mask_x, mask_y, k_n + 1)
                    if next_node not in next_nodes and mask_x>=0 and mask_x<=cost_grid_params.get('n_x')-1 and mask_y>=-(cost_grid_params.get('n_y')-1)/2 and mask_y<=(cost_grid_params.get('n_y')-1)/2:
                        self.cost_grid.costs.add_weighted_edges_from(curr_node, next_node, self.getEdgeCost(curr_node, next_node))
                        next_nodes.append((mask_x, mask_y, k_n + 1))
            active_nodes = next_nodes

        # connect end node to last layer (layer n_t-1)
        k = cost_grid_params.get('n_t') - 1
        for i in range(cost_grid_params.get('n_x')):
            for j in range(cost_grid_params.get('n_y')):
                self.cost_grid.costs.add_weighted_edges_from([((i,j,k), 'E', 0.0)])

    def populate(self, actor_position, list_of_obstacles, cost_grid_params, max_actor_vel):
        """
        Create a cost grid and populate it according to the obstacles

        Parameters
        ----------
        actor_position : undefinde position object #TODO
            an object containing the position and speed of acting duckiebot
        list_of_obstacles: obstacle[]
            a list of obstacle objects known to be around
        cost_grid_params: dictionary
            a python dictionary containing the size and resolution of the cost
            grid. Entries are 'n_t', 'n_x', 'n_y', 'dt', 'dx', 'dy'
        max_actor_vel: float
            the maximum velocity in [m/s] of a duckiebot

        Returns
        -------
        CostGrid : a cost grid where each grid point has an assigned cost
        """
        # add weighted nodes to cost_grid object
        for k in range(cost_grid_params.get('n_t')):
            for i in range(cost_grid_params.get('n_x')):
                for j in range(cost_grid_params.get('n_y')):
                    self.cost_grid.costs.setCost(i, j, k, self.getCost(self.cost_grid.costs.getX_pos(i,j,k), self.cost_grid.costs.getY_pos(i,j,k), self.cost_grid.costs.getT_pos(i,j,k), list_of_obstacles)

        # add weighted edges to graph
        # self.connectGraph(self.cost_grid.costs, actor_position.x, actor_position.y, cost_grid_params, max_actor_vel)
        self.connectGraph(self.cost_grid.costs, 0.0, 0.0, cost_grid_params, max_actor_vel)

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

    def getEdgeCost(curr_node, next_node):
        """
        defines the relation from edge length to edge cost
        (currently proportional to the euclidean distance)

        Parameters
        ----------
        curr_node: tuple(3)
            the current node
        next_node: tuple(3)
            the next node

        Returns
        -------
        float : the requested cost
        """
        curr_delta_x = self.cost_grid.costs.getX_pos(next_node[0], next_node[1], next_node[2])-self.cost_grid.costs.getX_pos(curr_node[0], curr_node[1], curr_node[2])

        curr_delta_y = self.cost_grid.costs.getY_pos(next_node[0], next_node[1], next_node[2])-self.cost_grid.costs.getY_pos(curr_node[0], curr_node[1], curr_node[2])

        norm_factor = 1.0; # depends on the order of magnitude of the node cost function
        distance = norm_factor * math.sqrt(curr_delta_x**2 + curr_delta_y**2)

        return distance
