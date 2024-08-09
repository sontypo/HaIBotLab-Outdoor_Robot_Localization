#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int16MultiArray.h"
#include <sstream>

std::string port;
serial::Serial ser;
std::string topic_v;
std::string string_to_stm;

uint8_t dir_1, dir_2, dir_3, dir_4;
uint8_t wheel_1, wheel_2, wheel_3, wheel_4;
uint8_t rec_data,count;
uint8_t buff_data[8];
int16_t angle,en1,en2,pre_angle,en3,en4;
uint8_t test='e';

double delta_theta_front=0, delta_s_front=0, omega_right_front=0, omega_left_front=0;
double delta_theta_rear=0, delta_s_rear=0, omega_right_rear=0, omega_left_rear=0;
double pi = 3.141592653589793238462643;
double rad_to_dr = 180/3.141592653589793238462643;
double rad=3.14159265359/1800;
double theta_front=0, x_front=0, y_front=0;
double pre_theta_front=0, pre_x_front=0, pre_y_front=0; 
double theta_rear=0, x_rear=0, y_rear=0;
double pre_theta_rear=0, pre_x_rear=0, pre_y_rear=0; 
double pulses_per_revolution=500;
double odom_angle_front, odom_angle_rear;
double wheel_radius=0.075, wheels_distance=0.45;
double scaling_factor, dt;

std::string topic_odom_front, topic_imu, topic_odom_rear;
ros::Time measurement_time_1, measurement_time_2, measurement_time_imu;
std::string fixed_frame_odom_front, fixed_frame_odom_rear;
ros::Time pre_measurement_time_1, pre_measurement_time_2, pre_measurement_time_imu;
ros::Publisher odom_front_pub; //node Publisher
ros::Publisher odom_rear_pub;
sensor_msgs::Imu imu;
ros::Publisher imu_pub;
bool _check = false;


// double delta_calculation(int16_t left_en, int16_t right_en)
// {
//     scaling_factor = (2.0 * pi * wheel_radius) / pulses_per_revolution;

//     double delta_theta, delta_s, omega_right, omega_left;

// 	omega_right = (right_en) * scaling_factor;
// 	omega_left = (left_en) * scaling_factor;
    
// 	delta_theta = (omega_right + omega_left) / wheels_distance;
// 	delta_s = (omega_right - omega_left) / 2.0;

//     return delta_theta, delta_s;
// }

void delta_front(void)
{
    scaling_factor = (2.0 * pi * wheel_radius) / pulses_per_revolution;

	omega_right_front = (en2) * scaling_factor;
	omega_left_front = (en1) * scaling_factor;
    
	delta_theta_front = (omega_right_front - omega_left_front) / wheels_distance;
	delta_s_front = (omega_right_front + omega_left_front) / 2.0;
}


void delta_rear(void)
{
    scaling_factor = (2.0 * pi * wheel_radius) / pulses_per_revolution;

	omega_right_rear = (en4) * scaling_factor;
	omega_left_rear = (en3) * scaling_factor;
    
	delta_theta_rear = (omega_right_rear - omega_left_rear) / wheels_distance;
	delta_s_rear = (omega_right_rear + omega_left_rear) / 2.0;
}


// double odometry_calculation(double pre_theta, double delta_theta, double delta_s, double pre_x, double pre_y)
// {
// 	double theta = pre_theta + delta_theta;
// 	double odom_angle = theta * rad_to_dr * 10;

// 	double x = pre_x + (cos(theta) * delta_s);
// 	double y = pre_y + (sin(theta) * delta_s);
// 	// pre_theta = theta; 
// 	// pre_x = x; 
// 	// pre_y = y;
//     return theta, odom_angle, x, y;
// }


void odometry_front_calculation()
{
	theta_front = pre_theta_front + delta_theta_front;
	odom_angle_front = theta_front * rad_to_dr * 10;

	x_front = pre_x_front + (cos(theta_front) * delta_s_front);
	y_front = pre_y_front + (sin(theta_front) * delta_s_front);

	pre_theta_front = theta_front; 
	pre_x_front = x_front; 
	pre_y_front = y_front;
}


void odometry_rear_calculation()
{
	theta_rear = pre_theta_rear + delta_theta_rear;
	odom_angle_rear = theta_rear * rad_to_dr * 10;

	x_rear = pre_x_rear + (cos(theta_rear) * delta_s_rear);
	y_rear = pre_y_rear + (sin(theta_rear) * delta_s_rear);
    
	pre_theta_rear = theta_rear; 
	pre_x_rear = x_rear; 
	pre_y_rear = y_rear;
}


void imu_data()
{
    measurement_time_imu = ros::Time::now();
    dt = (measurement_time_imu - pre_measurement_time_imu).toSec();
    geometry_msgs::Quaternion imu_quad= tf::createQuaternionMsgFromYaw(angle*rad);
    imu.header.stamp = measurement_time_imu;
    imu.header.frame_id = "base_link";
    imu.orientation = imu_quad;
    imu.angular_velocity.z = ((angle-pre_angle) * rad) / dt;
    imu_pub.publish(imu);
    pre_measurement_time_imu = measurement_time_imu;
    pre_angle = angle;
}


void odom_front_publish()
{
    measurement_time_1 = ros::Time::now(); 
    // delta_theta_front, delta_s_front = delta_calculation(en1, en2);
    delta_front();
    odometry_front_calculation();
    dt = (measurement_time_1 - pre_measurement_time_1).toSec();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_angle_front * rad);

    nav_msgs::Odometry odom_front;
    odom_front.header.stamp = measurement_time_1;
    odom_front.header.frame_id = fixed_frame_odom_front;

    odom_front.pose.pose.position.x = x_front;
    odom_front.pose.pose.position.y = y_front;
    odom_front.pose.pose.position.z = 0.0;
    odom_front.pose.pose.orientation = odom_quat;
    
    odom_front.twist.twist.linear.y = 0;
    odom_front.twist.twist.angular.z = delta_theta_front / dt;
    if((en1>0 && en2>0)||(en1<0 && en2<0))
    {
        odom_front.twist.twist.linear.x = delta_s_front / dt;
    }
    else
    {
        odom_front.twist.twist.linear.x = 0;
    }
    
    odom_front.child_frame_id = "base_link";

    odom_front_pub.publish(odom_front);
   
    pre_measurement_time_1 = measurement_time_1;
}


void odom_rear_publish()
{
    measurement_time_2 = ros::Time::now(); 
    // delta_theta_rear, delta_s_rear = delta_calculation(en3, en4);
    delta_rear();
    // theta_rear, odom_angle_rear, x_rear, y_rear = odometry_calculation(pre_theta_rear, delta_theta_rear, delta_s_rear, pre_x_rear, pre_y_rear);
    // pre_theta_rear = theta_rear; 
	// pre_x_rear = x_rear; 
	// pre_y_rear = y_rear;
    odometry_rear_calculation();
    dt = (measurement_time_2 - pre_measurement_time_2).toSec();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_angle_rear * rad);

    nav_msgs::Odometry odom_rear;
    odom_rear.header.stamp = measurement_time_2;
    odom_rear.header.frame_id = fixed_frame_odom_rear;

    odom_rear.pose.pose.position.x = x_rear;
    odom_rear.pose.pose.position.y = y_rear;
    odom_rear.pose.pose.position.z = 0.0;
    odom_rear.pose.pose.orientation = odom_quat;
    
    odom_rear.twist.twist.linear.y = 0;
    odom_rear.twist.twist.angular.z = delta_theta_rear / dt;
    if((en3>0 && en4>0)||(en3<0 && en4<0))
    {
        odom_rear.twist.twist.linear.x = delta_s_rear / dt;
    }
    else
    {
        odom_rear.twist.twist.linear.x = 0;
    }
    
    odom_rear.child_frame_id = "base_link";

    odom_rear_pub.publish(odom_rear);
   
    pre_measurement_time_2 = measurement_time_2;
}


void connect_serial()
{
    try
    {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
        ros::Duration(5).sleep();
    }
    if (ser.isOpen())
    {
        ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
    }
}


void read_serial()
{
        while (ser.available())
        {
            ser.read(&rec_data,1);
            if(count==0 && rec_data!=0x04)
                break;
            if(count==11 && rec_data!=0x05)
                break;
            if(count==11 && rec_data==0x05)
            {
                angle = buff_data[1]<<8|buff_data[2];
                en1 = buff_data[3]<<8|buff_data[4];
                en2 = buff_data[5]<<8|buff_data[6];
                en3 = buff_data[7]<<8|buff_data[8];
                en4 = buff_data[9]<<8|buff_data[10];
                count = 0;
                if(_check == false){
                    if(angle != 0){
                        break;
                    }
                    else{
                        _check = true;
                    }
                }
                odom_front_publish();
                odom_rear_publish();
                imu_data();
                // ROS_INFO("en1 = %d, en2 = %d, en3 = %d, en4 = %d, angle = %d",en1, en2, en3, en4, angle);
            }
            else
            {
                buff_data[count] = rec_data;
                count++;
            }
        }  
}


void write_serial(int16_t byte_reset, int16_t left_speed,int16_t right_speed)
{
    
    if(left_speed<0)
    {
        dir_1 = 1;
        dir_3 = 1;
        left_speed = -left_speed;
    }
    else if(left_speed>0)
    {
        dir_1 = 0;
        dir_3 = 0;
    }
    if(left_speed>255)
        left_speed = 255;

    if(right_speed<0)
    {
        dir_2 = 1;
        dir_4 = 1;
        right_speed = -right_speed;
    }
    else if(right_speed>0)
    {
        dir_2 = 0;
        dir_4 = 0;
    }
    if(right_speed>255)
        right_speed = 255;

    wheel_1 = (dir_1<<7)|0x01;
    wheel_2 = (dir_2<<7)|0x02;
    wheel_3 = (dir_3<<7)|0x03;
    wheel_4 = (dir_4<<7)|0x04;

    int16_t wheel_1_dir = wheel_1;
    int16_t wheel_2_dir = wheel_2;
    int16_t wheel_3_dir = wheel_3;
    int16_t wheel_4_dir = wheel_4;
    std::stringstream ss;
    ss << ", " << byte_reset<<" "<<wheel_1_dir<<" "<<wheel_3_dir<<" "<<left_speed<<" "<<wheel_2_dir<<" "<<wheel_4_dir<<" "<<right_speed<<".";
    string_to_stm=ss.str();
    ser.write(string_to_stm);
}


void velocities_callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    write_serial(0, msg->data[0], msg->data[1]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "connect_node");
    ros::NodeHandle node_xu_ly;
    ros::NodeHandle private_node_handle("~");
    ros::NodeHandle n;
    
    private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node_handle.param<std::string>("topic_odom_front", topic_odom_front, "odom_encoder/front");
    private_node_handle.param<std::string>("topic_odom_rear", topic_odom_rear, "odom_encoder/rear");
    private_node_handle.param<std::string>("topic_imu", topic_imu, "IMU");
    private_node_handle.param<std::string>("fixed_frame_odom_front", fixed_frame_odom_front, "odom_front");
    private_node_handle.param<std::string>("fixed_frame_odom_front", fixed_frame_odom_rear, "odom_rear");

    private_node_handle.param<double>("wheel_radius", wheel_radius, 0.075);
    private_node_handle.param<double>("wheels_distance", wheels_distance, 0.45);
    private_node_handle.param<double>("pulses_per_revolution", pulses_per_revolution, 2000.0);
    private_node_handle.param<std::string>("topic_velocities", topic_v, "topic_velocities");

    odom_front_pub = n.advertise<nav_msgs::Odometry>(topic_odom_front, 10);
    odom_rear_pub = n.advertise<nav_msgs::Odometry>(topic_odom_rear, 10);
    imu_pub = n.advertise<sensor_msgs::Imu>(topic_imu, 10);
    ros::Subscriber sub = n.subscribe(topic_v, 1, velocities_callback);

    connect_serial();
    for(int i = 0; i< 2; i++)
        write_serial(1,0,0);
    for(int i = 0; i< 5; i++)
        write_serial(0,0,0);
    
    ros::Rate rate_sleep(100);
  
    while (ros::ok())
    {
        read_serial();
        // write_driver(0,-69,69);
        rate_sleep.sleep();
        ros::spinOnce();
    }
    ser.close();
}