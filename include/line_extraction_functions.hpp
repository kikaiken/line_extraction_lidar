#ifndef INCLUDED_LINE_EXTRACTION_FUNCTIONS
#define INCLUDED_LINE_EXTRACTION_FUNCTIONS

#include "sensor_msgs/LaserScan.h"
#include <vector>
#include "convexHull.hpp"
#include "hokuyo_functions.hpp"
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

#endif
