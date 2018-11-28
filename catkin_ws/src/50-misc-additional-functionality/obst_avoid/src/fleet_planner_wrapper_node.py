#!/usr/bin/env python

import rospy
import duckietown_msgs.msg as dtmsg
import flock_simulator.msg as fsmsg

def main():
    rospy.init_node('worker_template_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    fleet_planner_sub = rospy.Subscriber(
                '/flock_simulator/state', fpmsg.FlockState, fleetPlannerCb)
    obstacle_pub = rospy.Publisher(
                'obst_avoid/obstacles', dtmsg.Obstacles, queue_size=10)

    command_sub = rospy.Subscriber(
                '/obst_avoid/command', dtmsg.Twist2DStamped, commandCb)
    fleet_command_pub = rospy.Publisher(
                'flock_simulator/command', fpmsg.FlockCommand, queue_size=10)



    #TODO add these variables to rosparam server and read from there
    duckie_radius = 0.2
    actor_id = '0'


def fleetPlannerCb(msg):
    # empty array for saving the obstacles
    obstacle_list = []

    # parse every duckie state to a moving object msg
    for duck in msg.duckie_states:
        if duck.duckie_id not actor_id:
            obstacle = dtmsg.MovingObstacle()
            obstacle.header = msg.header
            obstacle.pose = duck.pose
            obstacle.twist = duck.velocity
            obstacle.safety_radius = duckie_radius
            obstacle_list.append(obstacle)

    # create and fill Obstacles msg and publish it
    obstacle_list_msg = dtmsg.Obstacles()
    obstacle_list_msg.moving_objects = obstacle_list
    obstacle_pub.publish(obstacle_list_msg)


def commandCb(msg):
    # 

if __name__ == '__main__':
    main()
