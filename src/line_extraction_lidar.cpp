#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include "convexHull.hpp"
#include "hokuyo_functions.hpp"


void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

ros::NodeHandle nh;
ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

int main(int argc,char **argv){
  ros::init(argc,argv,"line_extraction_lidar");
  
  ros::Subscriber lidar = nh.subscribe("scan",1,lidar_callback);
  ros::Rate loop_rate(1);
  
  while(ros::ok()){
    //ROS_INFO("hello world");
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}


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
  dp.push_back(1079);/*最後の点を追加*/

  
  /*ここから描画*/
  /*LiDARから得た点群を描画*/
  
  visualization_msgs::Marker points;
  points.header.frame_id = "/my_frame";
  points.header.stamp = ros::Time::now();
  points.ns = "lidar_data";
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  //points are blue
  points.color.b = 1.0;
  points.color.a = 1.0;
  
  for(int i=0;i<1080;i++){
    geometry_msgs::Point p;
    p.x = msg->ranges[i]*hokuyoCos(i);
    p.y = msg->ranges[i]*hokuyoSin(i);
    p.z = 0;
    points.points.push_back(p);
  }
  marker_pub.publish(points);
  
}
