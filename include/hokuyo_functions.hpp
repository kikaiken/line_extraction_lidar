#ifndef INCLUDED_HOKUYO_FUNCTIONS
#define INCLUDED_HOKUYO_FUNCTIONS
#define _USE_MATH_DEFINES
#include<cmath>


/*lidarに関する定数*/
const float ANGLE_MIN = -2.35619,ANGLE_INCREMENT = 0.00436332,ANGLE_MAX = 2.35619;

double hokuyoCos(int index);

double hokuyoSin(int index);

#endif
