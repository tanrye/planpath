#include "least_squares.h"

Plan_straight::Plan_straight(const vector<double>& x, const vector<double>& y)
{
    double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    for (int i = 0; i<x.size(); ++i)
    {
        t1 += x[i] * x[i];
        t2 += x[i];
        t3 += x[i] * y[i];
        t4 += y[i];
    }
    B1 = (t3*x.size() - t2*t4) / (t1*x.size() - t2*t2);
    B0 = (t1*t4 - t2*t3) / (t1*x.size() - t2*t2);
}

double Plan_straight::getY(const double x) const
{
    return B1*x + B0;
}

double Plan_straight::getLoss(vector<double>& x, vector<double>& y)
{
    double loss = 0;
    for (int i = 0; i < x.size(); i++)
    {
        loss += pow((B1*x[i] + B0 - y[i]), 2);
    }
    loss /= x.size();
    return loss;
}

void Plan_straight::printResult() const
{
    cout << "参数为： a=" << B1 << " , b=" << B0 << endl;
    cout << "直线拟合方程为： " << "y = " << B1 << "*x + " << B0 << endl;
}

Plan_curve::Plan_curve(const vector<double>& x, const vector<double>& y)
{
    for (int i = 0; i<x.size(); i++)
    {
        sumX += x[i]; sumY += y[i];
        sumX2 += pow(x[i], 2); sumXY += x[i] * y[i];
        sumX3 += pow(x[i], 3); sumX2Y += pow(x[i], 2)*y[i];
        sumX4 += pow(x[i], 4);
    }

    //int count = 0;
    do
    {
        //count++;
        m1 = S2; S2 = (sumX2Y - sumX3*S1 - sumX2*S0) / sumX4; z1 = (S2 - m1)*(S2 - m1);
        m2 = S1; S1 = (sumXY - sumX*S0 - sumX3*S2) / sumX2; z2 = (S1 - m2)*(S1 - m2);
        m3 = S0; S0 = (sumY - sumX2*S2 - sumX*S1) / 42; z3 = (S0 - m3)*(S0 - m3);
    } while ((z1>N) || (z2>N) || (z3>N));

}

double Plan_curve::getY(const double x) const
{
    return S2*x*x + S1*x + S0;
}

double Plan_curve::getLoss(vector<double>& x, vector<double>& y) //计算误差，loss=(f(x)-y)^2求和取平均
{
    double loss = 0;
    for (int i = 0; i < x.size(); i++)
    {
        loss += pow((S2*x[i] * x[i] + S1*x[i] + S0 - y[i]), 2);
    }
    loss /= x.size();
    return loss;
}

void Plan_curve::printResult()
{
    cout << "参数为： a= " << S2 << "    b=" << S1 << "   c=" << S0 << endl;
    cout << "弧线拟合方程为: " << "y=" << S2 << "x^2+" << S1 << "x+" << S0 << endl;
}
