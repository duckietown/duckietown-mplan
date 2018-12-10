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

    """

    def __init__(self):
        """
        Create cost grid with start and end nodes

        Parameters
        ----------
        empty

        Returns
        -------
        empty
        """
        self.populated = False

        # initialize Graph object
        self.costs = nx.DiGraph()

        # initialize theoretical start and end node (data values unused)
        self.costs.add_node('S', x_pos=0, y_pos=0, t_pos=0, node_weight=0.0)
        self.costs.add_node('E', x_pos=0, y_pos=0, t_pos=0, node_weight=0.0)

    def __del__(self):
        pass

    def isPopulated(self):
        return self.populated

    def setCost(self, x, y, t, cost):
        """
        Get the value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point
        cost: float
            cost value of the desired cost point

        Returns
        -------
        empty
        """
        self.costs.nodes[(x, y, t)]['node_weight'] = cost

    def getCost(self, x, y, t):
        """
        Get the value of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            value of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['node_weight']

    def getX_pos(self, x, y, t):
        """
        Get the x_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            x_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['x_pos']

    def getY_pos(self, x, y, t):
        """
        Get the y_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            y_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['y_pos']

    def getT_pos(self, x, y, t):
        """
        Get the t_pos of the cost grid at a certain point

        Parameters
        ----------
        x: int
            x index of desired cost point
        y: int
            y index of desired cost point
        t: int
            t index of desired cost point

        Returns
        -------
        float
            t_pos of the cost grid at desired point
        """
        return self.costs.nodes[(x, y, t)]['t_pos']
