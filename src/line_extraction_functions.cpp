#include "line_extraction_functions.hpp"

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
  if(msg->ranges.empty()){
    ROS_INFO("URG data is empty");
    return;
  } 
 
  const double DIS_THRESHOLD = 0.3;/*要検証*/
  std::vector<int> dp{0};
  double width;
  ConvexHull convexHull(msg,0,1);
  for(int i=2;i<msg->ranges.size()-1;i++){
    convexHull.add();
    width = convexHull.calcWidth();

    if(width > DIS_THRESHOLD){
      dp.push_back(i);
      convexHull.renew(i,i+1);
    } 
  }
}
