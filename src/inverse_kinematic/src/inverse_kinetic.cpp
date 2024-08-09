#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16MultiArray.h"

std::string topic_v;
std::string cmd_vel;
ros::Publisher  Pub_v;
std_msgs::Int16MultiArray msg;
int left_wheel_velocity,right_wheel_velocity;

double wheel_radius=0.075; 
double wheels_distance=0.45;
double pi = 3.141592653589793238462643;
double omega_left = 0, omega_right = 0; 
double linear_vel = 0, angular_vel = 0;
double steering_factor=19.2;
int v_max_motor=14000,pwm_max=255;

double true_linear_vel;
double t_acceleration=1;
double t_decreasing=0.5;
double acc,speed_decreasing_const;
double realtime_t;
double dt;
uint8_t up_cnt,down_cnt;
ros::Time measurement_time;
ros::Time pre_measurement_time;


int16_t convert_pwm(double wheel_vel)
{
    int16_t pwm;
    pwm=wheel_vel*steering_factor;
    pwm=((double)pwm/v_max_motor)*(pwm_max);
    return pwm;
}


void IK(double linear_vel,double angular_vel)
{

    omega_left = (1.0/(2.0*wheel_radius))*(2.0*linear_vel - wheels_distance*angular_vel);
    omega_right = (1.0/(2.0*wheel_radius))*(2.0*linear_vel + wheels_distance*angular_vel);
    left_wheel_velocity = convert_pwm(omega_left*(60/(2*pi))); // m/s to rpm
    right_wheel_velocity = convert_pwm(omega_right*(60/(2*pi)));
}


void velocity_handling()
{
    msg.data.resize(0);
    msg.data.push_back(left_wheel_velocity);
    msg.data.push_back(right_wheel_velocity);
    Pub_v.publish(msg);
}


void decreasing_acceleration(double linear_vel)
{
    if(true_linear_vel!=linear_vel&&linear_vel!=0)
    {
        down_cnt=0;
        measurement_time=ros::Time::now();
        dt=(measurement_time-pre_measurement_time).toSec();
        if(up_cnt<2)
        {
            realtime_t=0;
            acc=linear_vel/t_acceleration;
            up_cnt++;
        }
        if(up_cnt>1)
        {
            realtime_t=realtime_t+dt;
        }
        if(realtime_t>t_acceleration)
         realtime_t=t_acceleration;
       
        true_linear_vel=(acc*realtime_t);
        pre_measurement_time=measurement_time;
    }
    else if(true_linear_vel!=linear_vel&&linear_vel==0)
    {
        up_cnt=0;
        measurement_time=ros::Time::now();
        dt=(measurement_time-pre_measurement_time).toSec();
        if(down_cnt<2)
        {
            realtime_t=0;
            speed_decreasing_const=true_linear_vel/t_decreasing;
            down_cnt++;
        }
        if(down_cnt>1)
        {
            realtime_t=realtime_t+dt;
        }
        if(realtime_t>t_decreasing)
        {
            realtime_t=t_decreasing;
            true_linear_vel=0;
        }
        else
            true_linear_vel=speed_decreasing_const*t_decreasing-(speed_decreasing_const*realtime_t);
        pre_measurement_time=measurement_time;
    }
    else
    {
        dt=0;
        realtime_t=0;
        up_cnt=0;
        down_cnt=0;
    }
}


void control(double linear_vel,double angular_vel)
{
    decreasing_acceleration(linear_vel);
    IK(true_linear_vel,angular_vel);
    velocity_handling();
}


void velocities_callback(const geometry_msgs::Twist &robot_vel)
{
     ROS_INFO("%f,%f",robot_vel.angular.z,robot_vel.linear.x);
     control(robot_vel.linear.x,robot_vel.angular.z);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "inverse_kinetic");
    ros::NodeHandle node_handling;
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle n;
    private_node_handle.param<std::string>("topic_velocities", topic_v, "topic_velocities");
    private_node_handle.param<std::string>("cmd_vel", cmd_vel, "Twist");
    private_node_handle.param<int>("left_wheel_velocity", left_wheel_velocity, 0);
    private_node_handle.param<int>("right_wheel_velocity", right_wheel_velocity, 0);
    private_node_handle.param<double>("steering_factor", steering_factor, 71.2);
    private_node_handle.param<double>("linear_vel", linear_vel, 0);
    private_node_handle.param<double>("angular_vel", angular_vel, 0);
    private_node_handle.param<double>("wheels_distance", wheels_distance, 0.38);
    private_node_handle.param<double>("t_acceleration", t_acceleration, 1);
    private_node_handle.param<double>("t_decreasing", t_decreasing, 0.);
    private_node_handle.param<int>("v_max_motor", v_max_motor, 10000);
    private_node_handle.param<int>("pwm_max", pwm_max, 255);
    Pub_v = n.advertise<std_msgs::Int16MultiArray>(topic_v, 10);
    ros::Subscriber sub = n.subscribe(cmd_vel, 1, velocities_callback);
    ros::spin();
}