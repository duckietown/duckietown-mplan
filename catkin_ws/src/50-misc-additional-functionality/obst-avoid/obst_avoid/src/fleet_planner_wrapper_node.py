#!/usr/bin/env python

import math
import rospy
import duckietown_msgs.msg as dtmsg
import flock_simulator.msg as fsmsg


#TODO add these variables to rosparam server and read from there
duckie_radius = 0.2
actor_id = '0'
sampler_frequency = 0.1
actor_omega = 0


def main():
    rospy.init_node('worker_template_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    fleet_planner_sub = rospy.Subscriber(
                '/flock_simulator/state', fsmsg.FlockState, fleetPlannerCb)
    obstacle_pub = rospy.Publisher(
                'obst_avoid/obstacles', dtmsg.Obstacles, queue_size=10)

    command_sub = rospy.Subscriber(
                '/obst_avoid/command', dtmsg.Twist2DStamped, commandCb)
    fleet_command_pub = rospy.Publisher(
                'flock_simulator/command', fsmsg.FlockCommand, queue_size=10)



def fleetPlannerCb(msg):
    # empty array for saving the obstacles
    obstacle_list = []

    # parse every duckie state to a moving object msg
    for duck in msg.duckie_states:
        if duck.duckie_id == actor_id:
            obstacle = dtmsg.MovingObstacle()
            obstacle.pose = duck.pose
            obstacle.twist = duck.velocity
            obstacle.safety_radius = duckie_radius
            obstacle_list.append(obstacle)
        else:
            actor_omega = duck.pose.theta

    # create and fill Obstacles msg and publish it
    obstacle_list_msg = dtmsg.Obstacles()
    obstacle_list_msg.header = msg.header
    obstacle_list_msg.moving_objects = obstacle_list
    obstacle_pub.publish(obstacle_list_msg)


def commandCb(msg):
    # empty array for saving the duckie commands (just one entry needed)
    command_list = []

    # parse obst_avoid command msg to fleet_planner command msg
    command = fsmsg.DuckieCommand()
    command.duckie_id = actor_id
    command.on_rails = True
    command.command.linear.x = math.sin(actor_omega)*msg.v
    command.command.linear.y = math.cos(actor_omega)*msg.v
    command.command.linear.z = 0
    command.command.angular.x = 0
    command.command.angular.y = 0
    command.command.angular.z = msg.omega


    # create and fill command msg and publish it
    commands_msg.header = msg.header
    commands_msg.dt = sampler_frequency
    commands_msg = fsmsg.FlockCommand()
    fleet_command_pub.publish(command_msg)

if __name__ == '__main__':
    main()
