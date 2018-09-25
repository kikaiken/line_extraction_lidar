#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "line_extraction_functions.hpp"

/*
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
  if(msg->ranges.empty()){
    ROS_INFO("URG data is empty");
    return;
  } 
 
  const double DIS_THRESHOLD;
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
*/

int main(int argc,char **argv){
  ros::init(argc,argv,"line_extraction_lidar");
  ros::NodeHandle nh;

  
  ros::Subscriber lidar = nh.subscribe("scan",1,lidar_callback);
  ros::Rate loop_rate(1);
  
  
  
  while(ros::ok()){
    //ROS_INFO("hello world");
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
  return 0;
}

