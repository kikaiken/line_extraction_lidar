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
  //*a = std::atan2((n*xy - sumX*sumY),(n*xSquared - sumX*sumX));
  *b = (xSquared*sumY - xy*sumX)/(n*xSquared - sumX*sumX);
}


double ConvexHull::calcWidth(){
  std::vector<double> dis;
  int n= endIndex - startIndex + 1;
  double theta,yIntercept;//y = tan(theta)*x + yInterceptに近似
  leastSquaresMethod(&theta,&yIntercept);
  double tmp2 = std::sqrt(1 + std::tan(theta)*std::tan(theta));
  for(int i=0;i<n;i++){
    dis.push_back(std::abs(msg->ranges[i]*(hokuyoSin(i) - hokuyoCos(i)*std::tan(theta)))/tmp2);//直線と点との距離
  }
  
  return *std::max_element(dis.begin(),dis.end()) - *std::min_element(dis.begin(),dis.end());      
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

