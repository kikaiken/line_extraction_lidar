#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <list>
#include <visualization_msgs/Marker.h>
#include "convexHull.hpp"
#include "hokuyo_functions.hpp"
#include "line.hpp"

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

void mergeLines(std::list<Line>& lines);

void merge(Line& line1,Line& line2);


int main(int argc,char **argv){

  ros::init(argc,argv,"line_extraction_lidar");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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

  for(int i=0;i<msg->ranges.size();i++){
    std::cout << msg->ranges[i] << ",";
  }
  std::cout << std::endl;

  
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  const double DIS_THRESHOLD = 0.05;/*要検証*/
  //const double EPSILON_THETA = 0.05,EPSILON_Y_INTERCEPT = 0.05;

  double a,b;
  std::vector<int> dp{0};
  double width;
  
  std::list<Line> lines;/*各線分を格納する*/
  int startIndex = 0;
  ConvexHull convexHull(msg,0,1);
  for(int i=2;i<msg->ranges.size()-1;i++){
    convexHull.add();
    width = convexHull.calcWidth();
    convexHull.leastSquaresMethod(&a,&b);
    
    if(width > DIS_THRESHOLD){
      //std::cout << dp.size() <<"," << a << "," << b << std::endl;
      lines.push_back(Line(startIndex,i-1,a,b,msg->ranges));
      //std:: cout << i << "," << lines.end()->dis << std::endl;
      startIndex = i;
      dp.push_back(i);
      convexHull.renew(i,i+1);
    } 
  }

  if(dp.back()!=(msg->ranges.size()-1)){
    dp.push_back(1079);/*最後の点を追加*/
    lines.push_back(Line(startIndex,msg->ranges.size()-1,a,b,msg->ranges));
  }

  /*直線をmerge*/
  mergeLines(lines);
  
  ROS_INFO("get");

  /*ここから描画*/  
  /*LiDARから得た点群を描画*/ 
  visualization_msgs::Marker points;
  points.header.frame_id = "/my_frame";
  points.header.stamp = ros::Time::now();
  points.ns = "lidar_data";
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.005;
  points.scale.y = 0.005;
  points.lifetime = ros::Duration(0);/*再描画するまで残る*/
  //points are blue
  points.color.b = 1.0;
  points.color.a = 1.0;

  geometry_msgs::Point p;
  p.z = 0;
  for(int i=0;i<1080;i++){
    p.x = msg->ranges[i]*hokuyoCos(i);
    p.y = msg->ranges[i]*hokuyoSin(i);
    points.points.push_back(p);
  }
  marker_pub.publish(points);
  
  /*凸包の頂点の描画*/
  visualization_msgs::Marker points2;/*直線が切り替わった点*/
  points2.header.frame_id = "/my_frame";
  points2.header.stamp = ros::Time::now();
  points2.ns = "segments";
  points2.id = 1;
  points2.type = visualization_msgs::Marker::POINTS;
  points2.action = visualization_msgs::Marker::ADD;
  points2.pose.orientation.w = 1.0;
  points2.scale.x = 0.03;
  points2.scale.y = 0.03;
  points2.lifetime = ros::Duration(0);/*再描画するまで残る*/
  //points are yellow
  points2.color.r = 1.0;
  points2.color.g = 1.0;
  points2.color.a = 1.0;

  for(int i=0;i<dp.size();i++){
    p.x = msg->ranges[dp[i]]*hokuyoCos(dp[i]);
    p.y = msg->ranges[dp[i]]*hokuyoSin(dp[i]);
    points2.points.push_back(p);
    //ROS_INFO_STREAM(p.x << "," << p.y );
  }
  
  marker_pub.publish(points2);

  /*merge後の頂点を描画*/
  visualization_msgs::Marker points3;
  points3.header.frame_id = "/my_frame";
  points3.header.stamp = ros::Time::now();
  points3.ns = "segments2";
  points3.id = 1;
  points3.type = visualization_msgs::Marker::POINTS;
  points3.action = visualization_msgs::Marker::ADD;
  points3.pose.orientation.w = 1.0;
  points3.scale.x = 0.04;
  points3.scale.y = 0.04;
  points3.lifetime = ros::Duration(0);/*再描画するまで残る*/
  //points are green
  points3.color.g = 1.0;
  points3.color.a = 1.0;


  for(auto itr=lines.begin();itr!=lines.end();itr++){
    p.x = itr->startX;
    p.y = itr->startY;
    points3.points.push_back(p);
  }
  
  ROS_INFO_STREAM("dp size" << dp.size() << "segments size" << lines.size());

}

void merge(Line& line1,Line& line2){
  line2.merge(line1);
}

void mergeLines(std::list<Line>& lines){
  const static double THETA = 0.2;
  const static double Y_INTERCEPT = 0.3;

  for(auto itr=lines.begin();itr!=std::prev(lines.end(),1);){
    if(std::abs(itr->dis - std::next(itr,1)->dis) < THETA && std::abs(itr->dis - std::next(itr,1)->dis) < Y_INTERCEPT){
      merge(*itr,*std::next(itr,1));
      itr = lines.erase(itr);
      continue;
    }
    itr++;
  }
    
}
