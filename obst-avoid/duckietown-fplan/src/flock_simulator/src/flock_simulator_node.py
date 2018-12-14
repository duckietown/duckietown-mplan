#!/usr/bin/env python

import random
import rospy
import tf
import state_manager
import street_obstruction
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import Pose2D, Twist, Vector3
from flock_simulator.msg import FlockState, FlockCommand, DuckieState


class FlockSimulatorNode(object):
    def __init__(self, map_name, n_duckies, x_pos, y_pos, theta, street_obstruction):
        self.node_name = rospy.get_name()

        self.street_obstruction = street_obstruction

        self.state_manager = state_manager.StateManager(
            map_name, n_duckies, x_pos, y_pos, theta, self.street_obstruction)

        # Subscribers
        self.sub_paths = rospy.Subscriber(
            '/flock_simulator/commands',
            FlockCommand,
            self.cbCommands,
            queue_size=1)

        # Publishers
        self.pub_state = rospy.Publisher(
            '/flock_simulator/state', FlockState, queue_size=1)
        self.msg_state = FlockState()

        self.pub_obstructions = rospy.Publisher(
            '/flock_simulator/street_obstruction', Marker, queue_size=1)

        self.isUpdating = False

    def cbCommands(self, msg):
        # Return same state if last callback has not finished
        if self.isUpdating:
            rospy.logwarn(
                'State not finished updating. Publishing previous state again.'
            )
            self.pub_state.publish(self.msg_state)
            return

        # Update state
        self.isUpdating = True
        dt = msg.dt.data
        commands = self.getCommands(msg)
        self.state_manager.updateState(commands, dt)
        self.isUpdating = False

        # Publish
        self.msg_state = self.generateFlockStateMsg(self.state_manager.duckies)
        self.pub_state.publish(self.msg_state)
        self.pub_obstructions.publish(self.street_obstruction.getMsg())
        self.publishTf(self.state_manager.duckies)

    def getCommands(self, msg):
        commands = {}
        for command in msg.duckie_commands:
            commands[command.duckie_id.data] = {
                'linear': command.command.linear.x,
                'angular': command.command.angular.z,
                'on_rails': command.on_rails.data
            }
        return commands

    def generateFlockStateMsg(self, duckies):
        msg = FlockState()
        msg.header.stamp = rospy.Time.now()
        for duckie_id in duckies:
            duckie = duckies[duckie_id]
            duckiestate_msg = DuckieState()
            duckiestate_msg.duckie_id = String(data=duckie_id)
            duckiestate_msg.on_service = Bool(data=duckie['on_service'])
            duckiestate_msg.pose = Pose2D(
                x=duckie['pose'].p[0] * self.state_manager.map.tile_size,
                y=duckie['pose'].p[1] * self.state_manager.map.tile_size,
                theta=duckie['pose'].theta)
            duckiestate_msg.velocity = Twist(
                linear=Vector3(duckie['velocity']['linear'], 0, 0),
                angular=Vector3(0, 0, duckie['velocity']['angular']))

            duckiestate_msg.in_fov = [
                String(data=visible_duckie)
                for visible_duckie in duckie['in_fov']
            ]
            duckiestate_msg.collision_level = Int8(
                data=duckie['collision_level'])
            msg.duckie_states.append(duckiestate_msg)
        return msg

    def publishTf(self, duckies):
        for duckie_id in duckies:
            duckie = duckies[duckie_id]
            stamp = rospy.Time.now()
            theta = duckie['pose'].theta
            x = duckie['pose'].p[0] * self.state_manager.map.tile_size
            y = duckie['pose'].p[1] * self.state_manager.map.tile_size

            transform_broadcaster.sendTransform((x, y, 0),
                                                tf.transformations.quaternion_from_euler(
                                                    0, 0, theta),
                                                stamp, duckie_id, "duckiebot_link")

    def onShutdown(self):
        rospy.loginfo('[%s] Shutdown.' % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('flock_simulator_node', anonymous=False)
    map_name = rospy.get_param('~map_name')
    n_duckies = rospy.get_param('~n_duckies', 2)
    x_pos = rospy.get_param('~x_pos_set', False)
    y_pos = rospy.get_param('~y_pos_set', False)
    theta = rospy.get_param('~theta_set', False)
    x_start = 1  # rospy.get_param('~x_start')
    y_start = 1  # rospy.get_param('~y_start')
    x_dim = rospy.get_param('~x_dim', 0.2)
    y_dim = rospy.get_param('~y_dim', 0.2)
    z_dim = rospy.get_param('~z_dim', 0.2)
    x_end = rospy.get_param('~x_end', x_start + y_start)  # default random
    y_end = rospy.get_param('~y_end', y_start + x_start)  # default random
    time = rospy.get_param('~time', 5)
    id = 0  # rospy.get_param('~id')
    street_obstruction = street_obstruction.StreetObstruction(
        x_start, y_start, x_dim, y_dim, z_dim, x_end, y_end, time, id)
    transform_broadcaster = tf.TransformBroadcaster()
    flock_simulator_node = FlockSimulatorNode(
        map_name, n_duckies, x_pos, y_pos, theta, street_obstruction)
    rospy.on_shutdown(flock_simulator_node.onShutdown)
    rospy.spin()
