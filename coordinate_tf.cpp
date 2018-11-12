#include "coordinate_tf.h"

//经纬度与平面坐的转换用的是米勒投影方法
//经纬度转XY
double* Longlat_xy::Longlat_tf_xy(const double &longgitude,const double &lattitude)
{
    double L = 6381372 * M_PI * 2;//地球周长
    double W = L;// 平面展开后，x轴等于周长
    double H = L / 2;// y轴约等于周长一半
    double mill = 2.3;// 米勒投影中的一个常数，范围大约在正负2.3之间
    double x = longgitude * M_PI / 180;// 将经度从度数转换为弧度
    double y = lattitude * M_PI / 180;// 将纬度从度数转换为弧度
    y = 1.25 * log(tan(0.25 * M_PI + 0.4 * y));// 米勒投影的转换
    // 弧度转为实际距离
    x = (W / 2) + (W / (2 * M_PI)) * x;
    y = (H / 2) - (H / (2 * mill)) * y;
    double* result = new double[2];
    result[0] = x;
    result[1] = y;
    return result;
}

//XY转经纬度
double* xy_tf_Longlat::MillierConvertion1(const double &x,const double &y)
{
    double L = 6381372 * M_PI * 2;
    double W = L;
    double H = L / 2;
    double mill = 2.3;
    double lat; lat = ((H / 2 - y) * 2 * mill) / (1.25 * H);
    lat = ((atan(exp(lat)) - 0.25 * M_PI) * 180) / (0.4 * M_PI);
    double lon; lon = (x - W / 2) * 360 / W;
    double* result = new double[2];
    result[0] = lon;
    result[1] = lat;
    return result;
}

//坐旋转
 Rotate::Rotate(const double &angle,const double &x,const double &y)
{
    tem_x = x*cos(angle*M_PI/180.0)-y*sin(angle*M_PI/180.0);
    tem_y = x*sin(angle*M_PI/180.0)+y*cos(angle*M_PI/180.0);
}
 double* Rotate::getrotate(const double &Offset_x,const double &Offset_y)
{
    goal_x = tem_x+Offset_x;
    goal_y = tem_y+Offset_y;
    result = new double[2];
    result[0] = goal_x;
    result[1] = goal_y;
    return  result;
}
