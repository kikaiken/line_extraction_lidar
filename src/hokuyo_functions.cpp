#include "hokuyo_functions.hpp"


double hokuyoCos(int index){
  return std::cos(M_PI/2 - (ANGLE_MAX - ANGLE_INCREMENT*index)); 
}
  
double hokuyoSin(int index){
  return std::sin(M_PI/2 - (ANGLE_MAX - ANGLE_INCREMENT*index));
}

