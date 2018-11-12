#include "ros/ros.h"
#include "rosmessage.h"
#include "coordinate_tf.h"
#include "ins_p2.h"

#define PI 3.14159265
#define N 1e-13
#define TranslationIMU 0.0  //IMU到车体坐系原点的距离
#define TranslationIMU_z 0.0
#define GPS_LX 0.0
#define GPS_LY 0.0
#define GPS_RX 0.0
#define GPS_RY 0.0
#define TranslationCamera_x 0  //camera到车体坐系原点的距离
#define TranslationCamera_y 0
bool number=false;
extern unsigned long a,b,c,d;
extern unsigned int accept_counter ,save_counter;
extern float Start_longitude,Start_lattitude,yaw,longitude_x,latitude_y,x_longitude,y_latitude,left_long_x,left_lat_y,right_long_x,right_lat_y;
extern float Start_x,Start_y,Start_left_x,Start_left_y,Start_right_x,Start_right_y,deta_x,deta_y;
extern vector<double> blue_point_x,red_point_x,blue_point_y,red_point_y,big_point_x,big_point_y;

extern double Path_fitting(const vector<double> &left_point_x,const vector<double> &left_point_y,const vector<double> &right_point_x,const vector<double> &right_point_y);

typedef struct world_point{
    double x;
    double y;
}World_point;

unsigned int ins_count=0;
void Ins_p2Callback(const plan_path::ins_p2 msg)
{
    ins_count++;
    float longitude,latitude,yaw_tem;
     longitude = (double)msg.Lon;
     latitude = (double)msg.Lat;
     yaw_tem = (double)msg.Heading;

    double* result_xy;
    Longlat_xy GPS;
    if(ins_count==0)
    {
        ins_count=0;
    }
    if(ins_count&&ins_count<2&&longitude&&latitude)
    {
        result_xy = GPS.Longlat_tf_xy(longitude,latitude);
        Start_x = result_xy[0];
        Start_y = result_xy[1];
        Start_left_x = result_xy[0]+GPS_LX;//
        Start_right_x = result_xy[0]+GPS_LY;
        Start_left_y = result_xy[1]+GPS_RX;
        Start_right_y = result_xy[1]+GPS_RY;
    }
    cout<<"world_xy("<<result_xy[0]<<","<<result_xy[1]<<")"<<endl;

    Longlat_xy IMU_GPS;
    double* result_XY;
    if(ins_count)
    {
        x_longitude = (double)msg.Lon;
        y_latitude = (double)msg.Lat;
        yaw = (double)msg.Heading;

       result_XY = IMU_GPS.Longlat_tf_xy(x_longitude,y_latitude);
        longitude_x = result_XY[0];
        latitude_y = result_XY[1];

        double *retu_rotate;
        Rotate IMU_thita(yaw,(-1)*longitude_x,latitude_y);
        retu_rotate = IMU_thita.getrotate(0,0);
        longitude_x = retu_rotate[0];
        latitude_y = retu_rotate[1];

        deta_x = abs(Start_x-longitude_x);
        deta_y = abs(Start_x-latitude_y);
        //
        if(longitude_x<Start_x)  //&&deta<GPS_RX;
        {
            left_long_x = longitude_x + GPS_LX + deta_x;
            right_long_x = longitude_x + GPS_RX - deta_x;
        }
        if(longitude_x>Start_x)  //&&deta<GPS_LX;
        {
            left_long_x = longitude_x + GPS_LX - deta_x;
            right_long_x = longitude_x + GPS_RX + deta_x;
        }
       left_lat_y = latitude_y + GPS_LY + deta_y;
       right_lat_y = latitude_y + GPS_RY + deta_y;
       //???
       //abs(longitude_x-Start_x)>??
    }
}

/*----------------------subscribe-camera-messages--and-deal-------------------------------------------*/

void customMsgCallback(const plan_path::RosMessage msg)
{
    //   这里读取入的桶的我数量值给a,b,c
     a=msg.Blue_x.size();
     b=msg.Red_x.size();
     c=msg.Big_x.size();
     d=msg.Yellow_x.size();
     accept_counter++;
     save_counter++;
     cout<<"帧数: "<<accept_counter<<" a: "<<a<<" b: "<<b<<" c: "<<c<<" d: "<<d<<endl;
     ROS_INFO("---receive-cameraInfo------");

       if(a)
       {
           for(unsigned int i=0;i<a;i++)
           {
               blue_point_x.push_back(msg.Blue_x[i]);
               blue_point_y.push_back(msg.Blue_y[i]);
           }

           for(unsigned int k=0;k<a;k++)
           {
               for(unsigned int j=0;j<a-1-k;j++)
               {
                   if(blue_point_y[j] > blue_point_y[j+1])
                   {
                       double tem_blue_y,tem_bule_x;
                       tem_blue_y = blue_point_y[j+1];
                       tem_bule_x = blue_point_x[j+1];
                       blue_point_y[j+1] = blue_point_y[j];
                       blue_point_x[j+1] = blue_point_x[j];
                       blue_point_y[j] = tem_blue_y;
                       blue_point_x[j] = tem_bule_x;

                   }
               }

           }
           a=0;
       }

           if(b)
           {
               for(unsigned int j= 0;j<b;j++)
               {
                   red_point_x.push_back(msg.Red_x[j]);
                   red_point_y.push_back(msg.Red_y[j]);
               }

               for(unsigned int k=0;k<b;k++)
               {
                   for(unsigned int j=0;j<b-1-k;j++)
                   {
                       if(red_point_y[j] > red_point_y[j+1])
                       {
                           double tem_red_y,tem_red_x;
                           tem_red_y = red_point_y[j+1];
                           tem_red_x = red_point_x[j+1];
                           red_point_y[j+1] = red_point_y[j];
                           red_point_x[j+1] = red_point_x[j];
                           red_point_y[j] = tem_red_y;
                           red_point_x[j] = tem_red_x;

                       }
                   }

               }
               b=0;
           }

           if(c)
           {
               for(unsigned int k =0;k<c;k++)
               {
                   big_point_x.push_back(msg.Big_x[k]);
                   big_point_y.push_back(msg.Big_y[k]);
               }
               c=0;
           }

           vector<double> new_blue_point_x,new_blue_point_y,new_red_point_x,new_red_point_y;
           vector<double> blue_pointx1,blue_pointy1,blue_pointx2,blue_pointy2,blue_pointx3,blue_pointy3,
                   red_pointx1,red_pointy1,red_pointx2,red_pointy2,red_pointx3,red_pointy3;
               if(blue_point_x.size())
               {
                   for(unsigned int i=0;i<blue_point_x.size();i++)
                   {

                       blue_pointy1.push_back(blue_point_y[i]);
                       //blue_pointx1.push_back(blue_point_x[i]*sin(180*PI/180.0));
                       blue_pointx1.push_back(blue_point_x[i]);

                       /*blue_pointy2.push_back(blue_pointy1[i]*cos(-180*PI/180.0)-blue_pointx1[i]*sin(-180*PI/180.0));
                       blue_pointx2.push_back(blue_pointx1[i]*cos(-180*PI/180.0)+blue_pointy1[i]*sin(-180*PI/180.0));
                      */
                       blue_pointy3.push_back(blue_pointy1[i]+TranslationCamera_x);
                       blue_pointx3.push_back(blue_pointx1[i]+TranslationCamera_y);

                       new_blue_point_x.push_back(blue_pointx3[i]);//暂时还不能划入到左右边界，再加一个左右相对位置的条件
                       new_blue_point_y.push_back(blue_pointy3[i]);

                   }
               }

           if(red_point_x.size())
           {
               for(unsigned int i=0;i<red_point_x.size();i++)
               {
           /*        red_pointx1.push_back( red_point_x[i]);
                   red_pointy1.push_back(red_point_y[i]*sin(90*PI/180.0));

                   red_pointx2.push_back( red_pointx1[i]*cos(-180*PI/180.0)-red_pointy1[i]*sin(-180*PI/180.0));
                   red_pointy2.push_back( red_pointy1[i]*cos(-180*PI/180.0)+red_pointx1[i]*sin(-180*PI/180.0));
           */
                   red_pointx1.push_back(red_point_x[i]);
                   red_pointy1.push_back(red_point_y[i]);

                   red_pointx3.push_back(red_pointx1[i]+TranslationCamera_x);
                   red_pointy3.push_back(red_pointy1[i]+TranslationCamera_y);

                   new_red_point_x.push_back(red_pointx3[i]);
                   new_red_point_y.push_back(red_pointy3[i]);
               }
           }

/*
    vector<double> long_left_x,lat_left_y,long_right_x,lat_right_y;

    for(unsigned int i=0;i<new_blue_point_x.size();i++)
    {
        long_left_x.push_back(new_blue_point_x[i]+left_long_x);//车体坐系下的桶变到世界坐系下,世界坐系原点参考的是惯导的初始位置
        lat_left_y.push_back(new_blue_point_y[i]+left_lat_y);
    }
    for(unsigned int i=0;i<new_red_point_x.size();i++)
    {
        long_right_x.push_back(new_red_point_x[i]+right_long_x);
        lat_right_y.push_back(new_red_point_y[i]+right_lat_y);
    }
*/
 /*   World_point save_left;
    vector< vector<int> > array_lx(save_counter,vector<int> (new_blue_point_x.size()) );
    vector< vector<int> > array_ly(save_counter,vector<int> (new_blue_point_x.size()) );
    vector< vector<int> > array_rx(save_counter,vector<int> (new_red_point_x.size()) );
    vector< vector<int> > array_ry(save_counter,vector<int> (new_red_point_x.size()) );

    vector< vector<int> > new_array_lx(save_counter,vector<int> (new_blue_point_x.size()) );
    vector< vector<int> > new_array_ly(save_counter,vector<int> (new_blue_point_x.size()) );
    vector< vector<int> > new_array_rx(save_counter,vector<int> (new_red_point_x.size()) );
    vector< vector<int> > new_array_ry(save_counter,vector<int> (new_red_point_x.size()) );
    vector<int>::iterator it;
    vector< vector<int> >::iterator iter;
    vector<int> vec_tem;/**/
    //vector<int> it;
    //vector< vector<int> >::iterator iter;
    //vector<int> vec_tem;

  /*  int array_lx[][];
    int array_ly[][];
    int array_rx[][];
    int array_ry[][];
    for(unsigned int i=0;i<save_counter;i++)
    {
        for(unsigned int j=0;j<new_blue_point_x.size();j++)
        {
            array_lx[i][j] =
        }
    }*/
  /*  if(save_counter)
    {

            for(unsigned int j=0;j<long_left_x.size();j++)
            {
                array_lx[save_counter].push_back(long_left_x[j]);
                array_ly[save_counter].push_back(lat_left_y[j]);
            }
            for(unsigned int k=0;k<long_right_x.size();k++)
            {
                array_rx[save_counter].push_back(long_right_x[k]);
                array_ry[save_counter].push_back(lat_right_y[k]);
            }

    if(!array_lx.empty()&&save_counter>=8)  //array_lx.size()>8
     {
         /*for(iter=array_lx.begin();iter!=array_lx.end();iter++)
         {
             vec_tem = *iter;
             for(it=vec_tem.begin();it!=vec_tem.end();it++)
             {
                 if(abs(array_lx[**iter][*it]-array_lx[**(iter+1)][*it])>5)
                 {

                 }
             }
         }*/
       /* for(unsigned int i=0;i<array_lx.size();i++)
        {
            for(unsigned int j=0;j<long_left_x.size();j++)
            {
                if(abs(array_lx[i][j]-array_lx[i+1][j])<5&&abs(array_lx[i][j]-array_lx[i+2][j])<10&&abs(array_lx[i][j]-array_lx[i+4][j])>20)
                {
                    //new_array_lx[]
                }
            }
        }
    }
    }*/
//Rotate p();

if(accept_counter>=1)
    {
        blue_point_x.clear();
        blue_point_y.clear();
        red_point_x.clear();
        red_point_y.clear();
        big_point_x.clear();
        big_point_y.clear();
        accept_counter = 0;
    }
cout<<"new_point  "<<new_blue_point_x.size()<<endl;
//if(number==false)
  Path_fitting(new_blue_point_x,new_blue_point_y,new_red_point_x,new_red_point_y);
//if(number==true)

}

