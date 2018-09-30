#ifndef INCLUDED_LINE
#define INCLUDED_LINE

#include "hokuyo_functions.hpp"
#include <cmath>
#include <vector>
#include <iostream>

class Line{

public:
  double theta;/*傾き[rad]*/
  double yIntercept;/*y切片*/
  double dis;/*LiDARからの距離*/
  int startIndex,endIndex;
  double startX,startY,endX,endY;
  std::vector<float> ranges;
  
  Line(int startIndex,int endIndex,double theta,double yIntercept,const std::vector<float>& ranges);

  void merge(Line line);
};
#endif
