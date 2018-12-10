import numpy as np
import networkx as nx
from visualization_msgs.msg import MarkerArray, Marker


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

    def getXPos(self, x, y, t):
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

    def getYPos(self, x, y, t):
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

    def getTpos(self, x, y, t):
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

    def toVizMsg(self, cost_grid_params):
        # local variables
        n_t = cost_grid_params.get('n_t')
        n_x = cost_grid_params.get('n_x')
        n_y = cost_grid_params.get('n_y')
        id = 1
        marker_array_msg = MarkerArray()
        # add weighted nodes to cost_grid object
        for k in range(n_t):
            for i in range(n_x):
                for j in range(n_y):
                    cost = self.costs.nodes[(i, j, k)]['node_weight']

                    # visualizations of each cost grid element
                    marker = Marker()
                    marker.id = id
                    id += 1
                    marker.ns = 'cost_grid'
                    marker.type = Marker.SPHERE
                    marker.header.frame_id = "/map"
                    marker.action = Marker.ADD
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05
                    marker.color.a = 0.5
                    marker.color.r = cost
                    marker.color.g = 0.1
                    marker.color.b = 0.1
                    marker.pose.position.x = self.getXPos(i,j,k)
                    marker.pose.position.y = self.getYPos(i,j,k)
                    marker.pose.position.z = self.getTpos(i,j,k)/10

                    marker_array_msg.markers.append(marker)

        return marker_array_msg
