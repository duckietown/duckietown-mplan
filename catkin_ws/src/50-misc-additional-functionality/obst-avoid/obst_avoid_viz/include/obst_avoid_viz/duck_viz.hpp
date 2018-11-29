#include <flock_simulator/FlockState.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace obst_avoid_viz {

class DuckViz {
public:
  /* \brief constructor
   * @param nh: the nodehandle used for the subscribers and publishers
   */
  DuckViz(ros::NodeHandle &nh);
  ~DuckViz();

private:
  ros::NodeHandle &nh_;
  ros::Subscriber duckie_bot_sub_;
  ros::Publisher marker_pub_;

  /* \brief callback method from duckie_bot_sub_
   *
   * This parses the flock_simulator::FlockState message and converts it to a
   * visualization_msgs::MarkerArray
   *
   * @param msg: the message as received by the subscriber
   */
  void duckieBotSubCb(const flock_simulator::FlockState &msg);
};

} // namespace obst_avoid_viz
