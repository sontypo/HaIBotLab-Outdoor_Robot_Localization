#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


// vi tri cua mang button
#define nhan 0
#define tron 1
#define tam_giac 2
#define vuong 3
#define trai_tren 4
#define phai_tren 5
#define trai_duoi 6
#define phai_duoi 7
#define select 8
#define start 9
#define connect 10
#define analog_trai 11
#define analog_phai 12
#define up 13
#define down 14
#define left 15
#define right 16
// vi tri cua mang axes
#define trai_sang 0
#define trai_len 1
#define nut_duoi_trai 2
#define phai_sang 3
#define phai_len 4
#define nut_duoi_phai 5


ros::Publisher pub;
geometry_msgs::Twist data;
double linear_x = 0, angular_z = 0;


std::string command_vel;

void joyTwistCallback(const sensor_msgs::JoyConstPtr &joy)
{
    if(joy->axes[trai_len]>0.5)//tien
    {
        if(joy->buttons[trai_tren])
        {
            data.angular.z=angular_z/3;
        }
        else  if(joy->buttons[phai_tren])
        {
            data.angular.z=-angular_z/3;
        }
        else
        {
            data.angular.z=0;
        }
        data.linear.x=linear_x;
        
    }
    else if(joy->axes[trai_len]<-0.5)//lui
    {
        if(joy->buttons[trai_tren])
        {
            data.angular.z=-angular_z/3;
        }
        else  if(joy->buttons[phai_tren])
        {
            data.angular.z=angular_z/3;
        }
        else
        {
            data.angular.z=0;
        }
        data.linear.x=-linear_x;
    }
    else
    {
        if(joy->buttons[trai_tren])
        {
            data.linear.x=0;
            data.angular.z=angular_z;
        }
        else if(joy->buttons[phai_tren])
        {
            data.linear.x=0;
            data.angular.z=-angular_z;
        }
        else
        {
            data.linear.x=0;
            data.angular.z=0;
        }
        
    }
    
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "ps3_node");
 ros::NodeHandle node_handle;
 ros::NodeHandle private_node_handle("~");//node param
 ros::Subscriber joy_subsriber = node_handle.subscribe<sensor_msgs::Joy>("joy",10,joyTwistCallback) ;
 private_node_handle.param<std::string>("command_vel", command_vel, "cmd_vel");//cong usb
 private_node_handle.param<double>("linear_x", linear_x, 0);
 private_node_handle.param<double>("angular_z", angular_z, 0);
 pub = node_handle.advertise<geometry_msgs::Twist>(command_vel, 2);
 ROS_INFO("Start ");
 ros::Rate loop_rate(50);//khai bong vong lap 10hz
 while(ros::ok())
 {
    pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
 }
 return 0;
}