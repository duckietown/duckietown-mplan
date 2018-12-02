#!/usr/bin/env python

import math
import rospy
import duckietown_msgs.msg as dtmsg
import flock_simulator.msg as fsmsg


#TODO add these variables to rosparam server and read from there
duckie_radius = 0.2
actor_id = "duckie-0"
sampler_frequency = 0.1
actor_omega = 0
fleet_command_pub = rospy.Publisher(
        'flock_simulator/commands', fsmsg.FlockCommand, queue_size=10)
obstacle_pub = rospy.Publisher(
        'obst_avoid/obstacles', dtmsg.Obstacles, queue_size=10)
actor_pub = rospy.Publisher(
        'obst_avoid/actor', dtmsg.Actor, queue_size=10)

def fleetPlannerCb(msg):
    # empty array for saving the obstacles
    obstacle_list = []
    actor_msg = dtmsg.Actor()

    # parse every duckie state to a moving object msg
    # if the duckie is our main controllerd duckie publish it to the actor topic
    # else publish to obstacle topic
    for duck in msg.duckie_states:
        if duck.duckie_id.data == actor_id:
            actor_msg.header = msg.header
            actor_msg.name = duck.duckie_id.data
            actor_msg.moving_object.pose.x = duck.pose.x
            actor_msg.moving_object.pose.y = duck.pose.y
            actor_msg.moving_object.pose.theta = duck.pose.theta
            actor_msg.moving_object.twist.x = duck.velocity.linear.x
            actor_msg.moving_object.twist.y = duck.velocity.linear.y
            actor_msg.moving_object.safety_radius = duckie_radius

            # save omega for command calculation
            actor_omega = duck.pose.theta
            
        else:
            obstacle = dtmsg.MovingObject()
            obstacle.pose.x = duck.pose.x
            obstacle.pose.y = duck.pose.y
            obstacle.pose.theta = duck.pose.theta
            obstacle.twist.x = duck.velocity.linear.x
            obstacle.twist.y = duck.velocity.linear.y
            obstacle.safety_radius = duckie_radius
            obstacle_list.append(obstacle)


    # create and fill Obstacles msg and publish it
    obstacle_list_msg = dtmsg.Obstacles()
    obstacle_list_msg.header = msg.header
    obstacle_list_msg.moving_objects = obstacle_list
    obstacle_pub.publish(obstacle_list_msg)
    actor_pub.publish(actor_msg)

def commandCb(msg):

    # empty array for saving the duckie commands (just one entry needed)
    command_list = []

    # parse obst_avoid command msg to fleet_planner command msg
    command = fsmsg.DuckieCommand()
    command.duckie_id.data = actor_id
    command.on_rails.data = False
    command.command.linear.x = math.cos(actor_omega)*msg.v
    command.command.linear.y = math.sin(actor_omega)*msg.v
    command.command.linear.z = 0
    command.command.angular.x = 0
    command.command.angular.y = 0
    command.command.angular.z = msg.omega


    # create and fill command msg and publish it
    commands_msg = fsmsg.FlockCommand()
    commands_msg.header = msg.header
    commands_msg.dt.data = sampler_frequency
    commands_msg.duckie_commands = [command]
    fleet_command_pub.publish(commands_msg)


def main():
    rospy.init_node('fleet_planner_wrapper_node', anonymous=False)

    # instantiate standalone worker with at 10 hz
    fleet_planner_sub = rospy.Subscriber(
                '/flock_simulator/state', fsmsg.FlockState, fleetPlannerCb)
    command_sub = rospy.Subscriber(
            '/obst_avoid/twist', dtmsg.Twist2DStamped, commandCb)



    rospy.spin()




if __name__ == '__main__':
    main()
