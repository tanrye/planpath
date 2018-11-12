#ifndef LEAST_SQUARES_H
#define LEAST_SQUARES_H

#include <iostream>
#include <cmath>
#include <vector>
#define N 1e-13

using namespace std;

class Plan_straight{
public:
    Plan_straight(const vector<double>& x, const vector<double>& y);
    double getLoss(vector<double>& x, vector<double>& y);
    double getY(const double x) const;
    void printResult() const;

private:
    double B0,B1;

};

class Plan_curve{
public:
    Plan_curve(const vector<double>& x, const vector<double>& y);
    double getLoss(vector<double>& x, vector<double>& y); //计算误差，loss=(f(x)-y)^2求和取平均
    double getY(const double x) const;
    void printResult();

private:
    double S2 = 0, S1 = 0, S0 = 0, m1, m2, m3, z1, z2, z3;
    double sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0, sumY = 0, sumXY = 0, sumX2Y = 0;

};

#endif // LEAST_SQUARES_H
