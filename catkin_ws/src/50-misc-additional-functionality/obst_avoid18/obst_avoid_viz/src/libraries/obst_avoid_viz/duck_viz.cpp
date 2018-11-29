
#include "obst_avoid_viz/duck_viz.hpp"

namespace obst_avoid_viz {

DuckViz::DuckViz(ros::NodeHandle &nh) : nh_(nh) {

  // init subscribers and publishers
  duckie_bot_sub_ = nh_.subscribe("flock_simulator/state", 10,
                                  &DuckViz::duckieBotSubCb, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "obst_avoid_viz/duckie_markers", 1);
}

DuckViz::~DuckViz() {}

/* \brief
 *  parse the flock_simulator::FlockState msg to rviz markers
 */
void DuckViz::duckieBotSubCb(const flock_simulator::FlockState &msg) {
  visualization_msgs::MarkerArray marker_array;

  for (size_t i = 0; i < msg.duckie_states.size(); i++) {
    // fill a single marker of whole marker array
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obst_avoid_viz";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg.duckie_states[i].pose.x;
    marker.pose.position.y = msg.duckie_states[i].pose.y;
    marker.pose.position.z = 0.05;

    // create a quaternion from theta angle
    tf2::Quaternion q;
    q.setRPY(0, 0, msg.duckie_states[i].pose.theta);

    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    marker.scale.x = 0.2;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.lifetime = ros::Duration();

    if (i == 0) {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    } else {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
    }
    // push filled marker to array
    marker_array.markers.push_back(marker);
  }

  // publish markers
  marker_pub_.publish(marker_array);
}

} // namespace obst_avoid_viz
