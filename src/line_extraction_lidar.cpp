#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "line_extraction_functions.hpp"

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

