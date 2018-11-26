from worker_base import WorkerBase
# from duckietown_msgs.msg import Obstacles
# from duckietown_msgs.msg import Actor

import duckietown_msgs.msg as dtmsg
import rospy

class ExamplePublisher(WorkerBase):
    """
    Template for a worker object. DO NOT EDIT THIS FILE! COPY IT!
    """

    def __init__(self, standalone=True, frequency=-1):
        """
        Call constructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(ExamplePublisher, self).__init__(standalone, frequency)
        rospy.loginfo('[ExamplePublisher.__init__] init complete')

    def __del__(self):
        """
        Call destructor method from base class.
        It should not be necessary to add any code here... add all concerning
        cleaning and shutting down in shutdown method.
        """
        super(ExamplePublisher, self).__del__()

    def init(self):
        """
        Initialise all members of class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.init_time = rospy.Time.now()

    def initIO(self):
        """
        Instantiate all input / output behaviour of worker. This mainly
        includes ros-publishers / -subscribers and advertised services.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.obstacles_pub = rospy.Publisher(
            'obst_avoid/obstacles', dtmsg.Obstacles, queue_size=10)

        self.actor_pub = rospy.Publisher(
            'obst_avoid/actor', dtmsg.Actor, queue_size=10)

    def advance(self, Ts=1.0):
        """
        Main method where data is processed and output generated. Gets called
        once in every loop with frequency self.frequency.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time

        Returns
        -------
        none
        """

        # publish an obstacle, varying between -1 or 1
        moving_object_msg = dtmsg.MovingObject()
        duration = (rospy.Time.now()-self.init_time).to_sec()
        var_y = round(duration % 3) * 2 - 3
        moving_object_msg.pose.y = round(var_y)
        moving_object_msg.pose.x = 1


        obstacle_msg = dtmsg.Obstacles()
        obstacle_msg.moving_objects = [moving_object_msg]
        self.obstacles_pub.publish(obstacle_msg)

        # simulate that actor drives forwards with constant speed from 0 continously to 2
        actor_msg = dtmsg.Actor()
        actor_msg.moving_object.pose.x = (duration % 20) / 20 * 2
        actor_msg.moving_object.pose.y = 0

        self.actor_pub.publish(actor_msg)





    def shutdown(self):
        """
        Clean up class before process end.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        # add your code here...
        pass