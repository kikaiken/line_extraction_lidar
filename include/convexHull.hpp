#ifndef INCLUDED_CONVEXHULL
#define INCLUDED_CONVEXHULL

#define _USE_MATH_DEFINES
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cmath>

class ConvexHull{

  /*lidarに関する定数*/
  const double PI = 3.141592;
  const float ANGLE_MIN =-2.35619,ANGLE_INCREMENT = 0.00436332;

  
  int startIndex,endIndex;
  sensor_msgs::LaserScan::ConstPtr msg;
  //最小２乗法用の変数
  double xSquared = 0,xy = 0,sumX = 0,sumY = 0;
  int tmp;//最小2乗法で前回どこまで計算したか保存する

public:
  ConvexHull(const sensor_msgs::LaserScan::ConstPtr &msg,int startindex,int endIndex);

public:
  //y = tan(a)*x + bで近似 aは[rad]
  void leastSquaresMethod(double* a,double* b);

  double calcWidth();

  void add();

  void renew(int startindex,int endIndex);

  double hokuyoCos(int index);

  double hokuyoSin(int index);
};

#endif
