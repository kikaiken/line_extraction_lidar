#include "convexHull.hpp"

ConvexHull::ConvexHull(const sensor_msgs::LaserScan::ConstPtr &msg,int startindex,int endIndex){
  ConvexHull::msg = msg;
  ConvexHull::startIndex = startindex;
  ConvexHull::endIndex = endIndex;
  tmp = startindex;
};


/*最小2乗法でy = tan(a)*x + bに近似　aは[rad] */
void ConvexHull::leastSquaresMethod(double* a,double* b){
  for(;tmp<=endIndex;tmp++){
    xy += msg->ranges[tmp]*msg->ranges[tmp]*hokuyoCos(tmp)*hokuyoSin(tmp);
    sumX += msg->ranges[tmp]*hokuyoCos(tmp);
    sumY += msg->ranges[tmp]*hokuyoSin(tmp);
    xSquared += msg->ranges[tmp]*msg->ranges[tmp]*hokuyoCos(tmp)*hokuyoCos(tmp);
  }

  int n = endIndex - startIndex + 1;
  *a = std::atan((n*xy - sumX*sumY)/(n*xSquared - sumX*sumX));
  *b = (xSquared*sumY - xy*sumX)/(n*xSquared - sumX*sumX);
}


double ConvexHull::calcWidth(){
  int n = endIndex - startIndex + 1;
  double dis[n];
  double theta,yIntercept;//y = tan(theta)*x + yInterceptに近似
  leastSquaresMethod(&theta,&yIntercept);  
  for(int i=0;i<n;i++){
     dis[i] = std::abs(msg->ranges[i]*(hokuyoSin(i) - hokuyoCos(i)*std::tan(theta)))/std::sqrt(1 + std::tan(theta)*std::tan(theta));//直線と点との距離
  }

  std::sort(&dis[0],&dis[n-1]);
  return dis[n-1] - dis[0];      
}


void ConvexHull::add(){
  ++endIndex;
}

void ConvexHull::renew(int startIndex,int endIndex){
  ConvexHull::startIndex = startIndex;
  ConvexHull::endIndex = endIndex;
  xSquared = 0;
  xy = 0;
  sumX = 0;
  sumY = 0;
  tmp = startIndex;
}

double ConvexHull::hokuyoCos(int index){
  return std::cos(M_PI/2 - (ANGLE_MIN + ANGLE_INCREMENT*index)); 
}
  
double ConvexHull::hokuyoSin(int index){
  return std::sin(M_PI/2 - (ANGLE_MIN + ANGLE_INCREMENT*index));
}
