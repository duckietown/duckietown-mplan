import numpy as np
import networkx as nx

class CostGrid:
    """

    Parameters
    ----------
    populated: bool
        represents wheter the cost grid has been populated. Has to be set
        manually by the populator object after population

    n_x: int
        size of cost grid in x direction

    n_y: int
        size of cost grid in y direction

    costs = numpy.ndarray<float>
        n_x x n_y numpy array containing all the weights

    """

    def __init__(self):
        """
        Create cost grid of size n_x x n_y

        Parameters
        ----------
        n_x: int
            size of cost grid in x direction

        n_y: int
            size of cost grid in y direction
        """
        self.populated = False

        # initialize Graph object
        self.costs = nx.Graph()

        # initialize nodes of Graph object
        self.costs.add_node((0,0), node_weight=0)   # S
        self.costs.add_node((-1,1), node_weight=0)  # A
        self.costs.add_node((1,1), node_weight=0)   # B
        self.costs.add_node((0,2), node_weight=0)   # E

        # initialize edges of Graph object
        self.costs.add_weighted_edges_from([((0,0), (-1,1), 0),  \
            ((0,0), (1,1), 0), ((-1,1), (0,2), 0), ((1,1), (0,2), 0)])

    def __del__(self):
        pass

    def isPopulated(self):
        return self.populated

    def getCost(x, y):
        """
        Get the value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point

        Returns
        -------
        float
            value of the cost grid at desired point
        """
        return self.costs[x][y]
