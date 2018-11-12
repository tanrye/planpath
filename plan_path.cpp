#include "ros/ros.h"
#include "local_planning.h"
#include "path_data.h"
#include "ins_p2.h"
#include "lane_point.h"
#include "spline.h"
#include "trackpoint.h"
#include "least_squares.h"
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>

unsigned int Path_data=20;
extern unsigned int counter;
extern vector<double> Left_point_x,Left_point_y,Right_point_x,Right_point_y,Center_point_x,Center_point_y;
extern vector<int> Left_line_x,Left_line_y,Right_line_x,Right_line_y,Center_line_x,Center_line_y;
extern ros::Publisher pub_centerline_data;
extern ros::Publisher pub_point_data;
extern ros::Publisher pub_state;

extern void point_toControl(CameraMsg::TrackPoint *input_msg,vector<int> data1_x,vector<int> data1_y,vector<int> data2_x,vector<int> data2_y);//,vector<double> data3_x,vector<double> data3_y
extern void parameters_info(plan_path::local_planning *input_msg ,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y);
extern void lane_toControl(plan_path::path_data *gj_msg,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y);
extern void pup_points_vis(visualization_msgs::Marker points,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y);

unsigned int Max(const unsigned int &data1,const unsigned int &data2)
{
    if(data1>data2)
        return data1;
    if(data2>data1)
        return data2;
    if(data1==data2)
        return data1;
}
unsigned int Min(const unsigned int &data1,const unsigned int &data2)
{
    if(data1>data2)
        return data2;
    if(data2>data1)
        return data1;
    if(data1==data2)
        return data1;
}
double xielv(double &y1,double &y2,double &x1,double &x2)
{
    double k=0.0;
    k=(y1-y2)/(x1-x2);
    return k;
}

double Path_fitting(const vector<double> &left_point_x,const vector<double> &left_point_y,const vector<double> &right_point_x,const vector<double> &right_point_y)
{
    counter++;

    //左右边界点初划分存储
    if(left_point_x.size()!=0)
        {
            for(unsigned int i=0;i<left_point_x.size();i++)
            {
                Left_point_x.push_back(left_point_x[i]);
                Left_point_y.push_back(left_point_y[i]);
            }

            /* if(Left_point_x.size()>=2)
             {
                 for(unsigned int i=3;i<Left_point_x.size();i++)
                 {
                     Left_point_x.pop_back();
                     Left_point_y.pop_back();
                 }
             }*/

             if(left_point_x.size()<2)
             {
                 for(unsigned int i=0;i<2;i++)
                 {
                     Left_point_x.push_back(Left_point_x[i]);
                     Left_point_y.push_back(Left_point_y[i]+300);
                 }
             }

        }

        if(left_point_x.size()==0&&counter)
        {
            unsigned int Ldiu_point=0;
            for(unsigned int i=0;i<5;i++)
            {
                Ldiu_point++;
                Left_point_x.push_back(150);
                Left_point_y.push_back(Ldiu_point+100);
            }
        }

    if(Left_point_x.size()!=0)
    {
        Plan_curve line_left(Left_point_x,Left_point_y);

          for(unsigned int i=0;i<left_point_x.size();i++)
          {
              for(int t=-10;t<10;t++)
              {
                  Left_line_y.push_back(line_left.getY(Left_point_x[i]-(Left_point_x[i]-Left_point_x[i+1])*t/10));
                  Left_line_x.push_back(Left_point_x[i]-(Left_point_x[i]-Left_point_x[i+1])*t/10);
              }
          }
          if(Left_line_x.size()>15)//为了不受远处不稳定锥桶的影响，将远处跳动大的点剔除
          {
              for(unsigned int i=16;i<=Left_line_x.size();i++)
              {
                  Left_line_x.pop_back();
                  Left_line_y.pop_back();
              }
          }
     }

    if(right_point_x.size()!=0)
    {
        for(unsigned int j=0;j<right_point_x.size();j++)
        {
            Right_point_x.push_back(right_point_x[j]);
            Right_point_y.push_back(right_point_y[j]);
        }
        /*if(Right_point_x.size()>=2)
        {
            for(unsigned int i=3;i<Right_point_x.size();i++)
            {
                Right_point_x.pop_back();
                Right_point_y.pop_back();
            }
        }*/

        if(right_point_x.size()<2)
        {
            for(unsigned int i=0;i<2;i++)
            {
                Right_point_x.push_back(Right_point_x[i]);
                Right_point_y.push_back(Right_point_y[i]+200);
            }
        }
    }

    if(right_point_x.size()==0&&counter)
    {
        unsigned int Rdiu_point=0;
        for(unsigned int i=0;i<5;i++)
        {
            Rdiu_point++;
            Right_point_x.push_back(-150);
            Right_point_y.push_back(Rdiu_point+100);
        }
    }

    if(Right_point_x.size()!=0)
    {
        Plan_curve line_right(Right_point_x,Right_point_y);

        for(unsigned int i=0;i<right_point_x.size();i++)
          {
              for(int t=-10;t<10;t++)
              {
                  Right_line_y.push_back(line_right.getY(Right_point_x[i]-abs((Right_point_x[i+1]-Right_point_x[i])*t/10)));
                  Right_line_x.push_back(Right_point_x[i]-abs((Right_point_x[i+1]-Right_point_x[i])*t/10));
              }
          }
        if(Right_line_x.size()>15)
        {
            for(unsigned int i=16;i<=Right_line_x.size();i++)
            {
                Right_line_x.pop_back();
                Right_line_y.pop_back();
            }
        }
    }
    if(Left_line_x.size())
        cout<<"left_line_size: "<<Left_line_x.size()<<endl;
    if(Right_line_x.size())
        cout<<"right_line_x: "<<Right_line_x.size()<<endl;
    /*------center_point------center_line-----------------------------------------------------------------*/
    if(Left_line_x.size()>=1&&Right_line_x.size()>=1)
    {
        /*vector<double> tem_x,tem_y;
        if(abs(Left_line_y[0]-Right_line_y[0])>100)
           {
            for(unsigned int i=0;i<10;i++)
                    {
                        tem_x.push_back((Left_line_x[i]+Right_line_x[i])/2);
                        tem_y.push_back((Left_line_x[i]+Right_line_y[i])/2);
                    }
            Plan_straight plan_center(tem_x,tem_y);
            for(unsigned int i=0;i<tem_x.size();i++)
            {
                for(int t=-10;t<10;t++)
                {
                    Center_line_y.push_back(plan_center.getY(tem_x[i]-abs((tem_x[i+1]-tem_x[i])*t/10)));
                    Center_line_x.push_back(tem_x[i]-abs((tem_x[i+1]-tem_x[i])*t/10));
                }
            }
           }
        if(abs(Left_line_y[0]-Right_line_y[0])<100)
        {
           //Plan_curve plan_center(Left_);
            if(Left_point_x.size()&&Right_point_x.size())
            {
                for(unsigned int i=0;i<ComputerLR_point(Left_point_x.size(),Right_point_x.size());i++)
                {
                    Center_point_x.push_back((Left_point_x[i]+Right_point_x[i])/2);
                    Center_point_y.push_back((Left_point_y[i]+Right_point_y[i])/2);
                }
            }
            if(Center_point_x.size())
            {
                Plan_curve plan_center(Center_point_x,Center_point_y);
                for(unsigned int i=0;i<Center_point_x.size();i++)
                {
                    for(int t=-5;t<10;t++)
                    {
                        Center_line_y.push_back(plan_center.getY(Center_point_x[i]-abs((Center_point_x[i+1]-Center_point_x[i])*t/10)));
                        Center_line_x.push_back(Center_point_x[i]-abs((Center_point_x[i+1]-Center_point_x[i])*t/10));
                    }
                }
            }

        }*/

        for(unsigned int i=0;i<2;i++)//<15
         {
                Center_point_x.push_back((Left_point_x[i]+Right_point_x[i])/2);//采用的是用两边实际的点计算出实际中点，由得到的中点拟合出中线
                Center_point_y.push_back((Left_point_y[i]+Right_point_y[i])/2);
         }
        /*if(Center_point_x.size()>15)//Left_line_x[i]+Right_line_x[i])/2
        {
            for(unsigned int i=15;i<Center_point_x.size();i++)
            {
                Center_point_x.pop_back();
                Center_point_y.pop_back();
            }
        }*/

        Plan_curve plan_center(Center_point_x,Center_point_y);
        for(unsigned int i=0;i<Center_point_x.size();i++)
        {
            for(int t=-10;t<10;t++)
            {
                Center_line_y.push_back(plan_center.getY(Center_point_x[i]-abs((Center_point_x[i+1]-Center_point_x[i])*t/10)));
                Center_line_x.push_back(Center_point_x[i]-abs((Center_point_x[i+1]-Center_point_x[i])*t/10));
            }
        }
    }

   /*-------curve--------------------------------------------------------------------------------------------------------*/
/*
    for(unsigned int i=0;i<Min(Left_point_x.size(),Right_point_x.size());i++)
    {
        //前次的位置可以在这里，做一个条件
        if((Left_point_x[0] > Left_point_x[i+1]&&Right_line_x[0]>Right_point_x[i+1])||(Left_point_x[0]<Left_point_x[i+1]&&Right_point_x[0]<Right_point_x[i+1]))
        {
            if((Left_point_x[0] > Left_point_x[i+1] && Right_line_x[0]>Right_point_x[i+1]))
            {
              Plan_curve curve_left(Left_point_x,Left_point_y);

                for(int i=0;i<a;i++)
                {
                    Left_line_y[i] = curve_left.getY(Left_point_x[i]);
                    Left_line_x[i] = Left_point_x[i];
                }
            }
            if((Left_point_x[0]<Left_point_x[i+1] && Right_point_x[0]<Right_point_x[i+1]))
            {
              Plan_curve curve_right(Right_point_x,Right_point_y);

                for(unsigned int i=0;i<b;i++)
                {
                    Right_line_y[i] = curve_right.getY(Right_point_x[i]);
                    Right_line_x[i] = Right_point_x[i];
                }
            }
        }
    }
*/
    /*for(unsigned int i=0;i<Max(e,f);i++)
    {
        Center_point_x[i] = ((int)Left_point_x[i]+(int)Right_point_x[i])/2;//采用的是用两边实际的点计算出实际中点，由得到的中点拟合出中线
        Center_point_y[i] = ((int)Left_point_y[i]+(int)Right_point_y[i])/2;
    }

    tk::spline s;
    s.set_points(Center_point_x,Center_point_y);//得到条插值参数m_a,m_b,m_c
    for(unsigned int i=0;i<Center_point_x.size();i++)
    {
        Center_line_y.push_back(s(Center_point_x[i]));
        Center_line_x.push_back(Center_point_x[i]);
    }*/

   /* plan_path::path_data center_msg;//发给自己看的数据/
    lane_toControl(&center_msg,Left_line_x,Left_line_y,Right_line_x,Right_line_x,Center_line_x,Center_line_y);
    pub_centerline_data.publish(center_msg);*/

   CameraMsg::TrackPoint point_msg;
    point_toControl(&point_msg,Left_line_x,Left_line_y,Right_line_x,Right_line_y);//,Center_line_x,Center_line_y    pub_point_data.publish(point_msg);
    pub_point_data.publish(point_msg);

 /*   plan_path::local_planning state_parameters;
    parameters_info(&state_parameters,Left_line_x,Left_line_y,Right_line_x,Right_line_y,Center_line_x,Center_line_y);
    pub_state.publish(state_parameters);
*/
/*
    visualization_msgs::Marker rviz_msg;
    pup_points_vis(rviz_msg,Left_line_x,Left_line_y,Right_line_x,Right_line_y,Center_line_x,Center_line_y);
*/
    if(counter>=1)
    {
    Left_point_x.clear();
    Left_point_y.clear();
    Right_point_x.clear();
    Right_point_y.clear();
    Center_point_x.clear();
    Center_point_y.clear();

    Left_line_x.clear();
    Left_line_y.clear();
    Right_line_x.clear();
    Right_line_y.clear();
    Center_line_x.clear();
    Center_line_y.clear();
    counter=0;
    }
}

