#include "ros/ros.h"
#include "local_planning.h"
#include "path_data.h"
#include "lane_point.h"
#include "rosmessage.h"
#include "trackpoint.h"
#include "ins_p2.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <vector>
/*---------------------------------------------------------------------------------------------*/
using namespace std;
unsigned long a=0,b=0,c=0,d=0;//a,b,c读取入的红蓝黄的桶的数量
unsigned int counter=0,save_counter=0;
unsigned int accept_counter = 0 ;
float Start_longitude=0.0,Start_lattitude=0.0,yaw=0.0,longitude_x=0.0,latitude_y=0.0,x_longitude=0.0,y_latitude=0.0,left_long_x=0.0,left_lat_y=0.0,right_long_x=0.0,right_lat_y=0.0;
float Start_x=0.0,Start_y=0.0,Start_left_x=0.0,Start_left_y=0.0,Start_right_x=0.0,Start_right_y=0.0,deta_x=0,deta_y=0;

ros::Publisher pub_centerline_data;
ros::Publisher pub_point_data;
ros::Publisher pub_state;
ros::Publisher pub_status_information;
ros::Publisher path_pub;//visualiza

vector<double> Left_point_x,Left_point_y,Right_point_x,Right_point_y,Center_point_x,Center_point_y;
vector<int> Left_line_x,Left_line_y,Right_line_x,Right_line_y,Center_line_x,Center_line_y;
vector<double> blue_point_x,red_point_x,blue_point_y,red_point_y,big_point_x,big_point_y;

extern void Ins_p2Callback(const plan_path::ins_p2 msg);
extern void customMsgCallback(const plan_path::RosMessage msg);
/*-------------消息转化----------------------------------------------------*/

void point_toControl(CameraMsg::TrackPoint *input_msg,vector<int> data1_x,vector<int> data1_y,vector<int> data2_x,vector<int> data2_y)//,vector<double> data3_x,vector<double> data3_y
{

    for(unsigned int i=0;i<data1_x.size();i++)
    {
        input_msg->x.push_back( data1_x[i]);
        input_msg->y.push_back( data1_y[i]);
    }
    for(unsigned int i=0;i<data2_x.size();i++)
    {
        input_msg->x.push_back( data2_x[i]);
        input_msg->y.push_back( data2_y[i]);
    }
    /*for(unsigned int i=0;i<data3_x.size();i++)
    {
        input_msg->x.push_back( data3_x[i]);
        input_msg->y.push_back( data3_y[i]);
    }*/

}

void parameters_info(plan_path::local_planning *input_msg ,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y)
{
    static unsigned int num=0;
 /*   num++;
    input_msg->heading=0.0;
    input_msg->id=num;
    input_msg->vehicle_speed=10;
    input_msg->time;//ros::Time::now();
    input_msg->path_data;
    input_msg->path_num=1;
    input_msg->path_info=0;
*/
    for(unsigned int i=0;i<left_x.size();i++)
    {
        input_msg->left_x.push_back(left_x[i]);
        input_msg->left_y.push_back(left_y[i]);
    }
    for(unsigned int i=0;i<right_x.size();i++)
    {
        input_msg->right_x.push_back(right_x[i]);
        input_msg->right_y.push_back(right_y[i]);
    }
    for(unsigned int i=0;i<center_x.size();i++)
    {
        input_msg->path_x.push_back(center_x[i]);
        input_msg->path_y.push_back(center_y[i]);
    }

}
//发给自己看的数据
/*void lane_toControl(plan_path::path_data *gj_msg,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y)
{
    for(unsigned int i=0;i<left_x.size();i++)
    {
       gj_msg->left.x = left_x[i];
       gj_msg->left.y = left_y[i];
    }
    for(unsigned int i=0;i<center_x.size();i++)
    {
        gj_msg->path.x = center_x[i];
        gj_msg->path.y = center_y[i];
    }
    for(unsigned int i=0;i<right_x.size();i++)
    {
        gj_msg->right.x = right_x[i];
        gj_msg->right.y = right_y[i];
    }
//gj_msg.left.x
}*/

void pup_points_vis(visualization_msgs::Marker points,vector<int> left_x,vector<int> left_y,vector<int> right_x,vector<int> right_y,vector<int> center_x,vector<int> center_y)
{
  vector<int> m;
  vector<int> q;
  vector<int> e;
  vector<int> f;
  vector<int> j;
  vector<int> g;
  for(unsigned int i=0;i<left_x.size();i++)
  {
      m.push_back(left_x[i]);
      q.push_back(left_y[i]);
  }
  for(unsigned int i=0;i<right_x.size();i++)
  {
      e.push_back(right_x[i]);
      f.push_back(right_y[i]);
  }
  for(unsigned int i=0;i<center_x.size();i++)
  {
      j.push_back(center_x[i]);
      g.push_back(center_y[i]);
  }
  float v = 0.0;
   // ros::Rate r(30);
  //while (ros::ok())
 // {
   // visualization_msgs::Marker points, line_strip, line_list;

    points.header.frame_id = "/my_frame";

    points.header.stamp = ros::Time::now();

    points.ns = "points_and_lines";

    points.action = visualization_msgs::Marker::ADD;

    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    points.color.g = 1.0f;
    points.color.a = 1.0;


    // Create the vertices for the points and lines
    for (unsigned int i = 0; i < m.size(); ++i)
    {
      float lx = m[i];
      float ly = q[i];

      geometry_msgs::Point l;
      l.x = (int32_t)lx;
      l.y = ly;
      l.z = 0;

      points.points.push_back(l);
    }
    for (unsigned int i = 0; i < e.size(); ++i)
    {
      float rx = e[i];
      float ry = f[i];

      geometry_msgs::Point r;
      r.x = (int32_t)rx;
      r.y = ry;
      r.z = 0;

      points.points.push_back(r);

    }
    for (unsigned int i = 0; i < j.size(); ++i)
    {
      float cx = j[i];
      float cy = g[i];

      geometry_msgs::Point c;
      c.x = (int32_t)cx;
      c.y = cy;
      c.z = 0;

      points.points.push_back(c);
    }

    path_pub.publish(points);
    /*m.clear();
    q.clear();
    e.clear();
    f.clear();
    j.clear();
    g.clear();*/
    //r.sleep();

    v += 0.04;
  //}
}

/*--------------------------------------------------------------------------------------------------------------*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "msgs_listen_talker");
    ros::NodeHandle nh;

    pub_centerline_data =nh.advertise<plan_path::path_data>("path_data",2);
    pub_status_information = nh.advertise<plan_path::local_planning>("status_information",2);

     path_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1, true);
    //uint32_t shape = visualization_msgs::Marker::CUBE;

     pub_point_data =nh.advertise<CameraMsg::TrackPoint>("pointdata",2);

    ros::Subscriber sub_string = nh.subscribe("CameraInfo", 2, customMsgCallback);
    ros::Subscriber sub_IMUmsg = nh.subscribe("ins_p2Msg",2,Ins_p2Callback);

    /*for(;;)
      {
        pup_points_vis();
      }*/
    ros::spin();

    return 0;
}
