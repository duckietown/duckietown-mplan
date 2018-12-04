#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from duckietown_msgs.msg import TimedPath
from geometry_msgs.msg import PoseStamped


class PathViz:
    """A class for visualization of the planned path"""
    def __init__(self):
        self.timed_path_sub = rospy.Subscriber('obst_avoid/trajectory', TimedPath, self.timedPathSubCb)
        self.path_pub = rospy.Publisher('obst_avoid_viz/path', Path, queue_size=10)

    def __del__(self):
        pass

    def timedPathSubCb(self, data):

        path = Path()
        path.header.stamp = data.start_time
        path.header.frame_id = 'map'
        for i, elem in enumerate(data.positions):
            pose = PoseStamped()
            pose.header.stamp = data.start_time + rospy.Duration(data.times[i]*data.duration)
            pose.header.frame_id = 'map'
            pose.pose.position.x = data.positions[i].x
            pose.pose.position.y = data.positions[i].y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0

            path.poses.append(pose)
        self.path_pub.publish(path)

        # trajectory = Trajectory()
        # trajectory.fromMsg(data)
        # x,y = trajectory.getPositionFromTimePoint(rospy.Time.now())
        # marker_msg = Marker()
        # marker_msg.header.stamp = d
        # marker_msg.header.frame_id = 'map'


def main():
    rospy.init_node('path_node')
    path_viz = PathViz()
    rospy.spin()

if __name__ == '__main__':
    main()
