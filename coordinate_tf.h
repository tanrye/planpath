#ifndef COORDINATE_TF_H
#define COORDINATE_TF_H

#include <cmath>
#include <iostream>

using namespace std;

class Longlat_xy {
public:
   double* Longlat_tf_xy(const double &longgitude,const double &lattitude);
};

 class xy_tf_Longlat{
 public:
     double* MillierConvertion1(const double &x,const double &y);

};

 class Rotate{
 public:
     Rotate(const double &angle,const double &x,const double &y);
     double* getrotate(const double &Offset_x,const double &Offset_y);
 private:
     double goal_x,goal_y,tem_x,tem_y;
     double* result;
 };

#endif // COORDINATE_TF_H
