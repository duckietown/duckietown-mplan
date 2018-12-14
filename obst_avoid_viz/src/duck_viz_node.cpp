
#include <obst_avoid_viz/duck_viz.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "duck_viz");
  ros::NodeHandle nh;

  // create duck viz msg publisher and let it run
  obst_avoid_viz::DuckViz duck_viz(nh);
  ros::spin();
}
