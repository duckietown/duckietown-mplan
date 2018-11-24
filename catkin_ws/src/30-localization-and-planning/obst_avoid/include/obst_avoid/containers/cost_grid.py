import numpy as np


class CostGrid:
    """
    Represents a discrete grid where every point has a cost

    #TODO define convention for x and y indices, where do they start

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

    def __init__(self, n_x, n_y):
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
        self.n_x = n_x
        self.n_y = n_y
        self.costs = np.empty((n_x, n_y))

    def __del__(self):
        pass

    def isPopulated():
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
