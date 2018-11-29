#include <flock_simulator/FlockState.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace obst_avoid_viz {

class DuckViz {
public:
  DuckViz(ros::NodeHandle &nh);
  ~DuckViz();

private:
  ros::NodeHandle &nh_;
  ros::Subscriber duckie_bot_sub_;
  ros::Publisher marker_pub_;
  void duckieBotSubCb(const flock_simulator::FlockState &msg);
};
} // namespace obst_avoid_viz
