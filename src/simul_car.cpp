#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

ros::Publisher car_position_pub;
ros::Publisher rviz_car_pub;

std::pair<double, double> car_position = {0.0, 0.0};

float car_velocity = 0.0;
float car_steering_rad = 0.0;

double deg2rad = M_PI/180;
double rad2deg = 180/M_PI;
double yaw_gain_deg, car_yaw_rad = 0.0;

void Contorl_Callback(const geometry_msgs::Twist::ConstPtr &control_msg)
{
    car_velocity = control_msg->linear.x;
    car_steering_rad = control_msg->angular.z;
}

void Rviz_car()
{
    visualization_msgs::Marker ego_car;

    ego_car.header.frame_id = "edu_frame";
    ego_car.header.stamp = ros::Time::now();
    ego_car.ns = "ego_car";
    ego_car.id = 0;
    ego_car.type = visualization_msgs::Marker::CUBE;
    ego_car.action = visualization_msgs::Marker::ADD;
    ego_car.lifetime = ros::Duration(0.2);

    ego_car.pose.position.x = car_position.first;
    ego_car.pose.position.y = car_position.second;
    ego_car.pose.position.z = 0;

    tf2::Quaternion quaternion;

    quaternion.setRPY(0, 0, car_yaw_rad);

    ego_car.pose.orientation = tf2::toMsg(quaternion);

    ego_car.scale.x = 1.04;
    ego_car.scale.y = 0.75;
    ego_car.scale.z = 0.1;

    ego_car.color.r = 0.0;      
    ego_car.color.g = 0.0;     
    ego_car.color.b = 1.0;
    
    ego_car.color.a = 0.75;

    rviz_car_pub.publish(ego_car);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simul_car_node");
	ros::NodeHandle nh;

    ros::Rate loop(8);

    ros::Subscriber car_control_sub = nh.subscribe<geometry_msgs::Twist>("/control_value", 10, &Contorl_Callback);

    car_position_pub = nh.advertise<geometry_msgs::Point> ("/car_position", 10);
    rviz_car_pub = nh.advertise<visualization_msgs::Marker> ("/rviz_car_position", 10);

    ROS_INFO("Simulation Running...");

    geometry_msgs::Point car_pos;

	while(ros::ok())
    {	
        ros::Time ros_stamp = ros::Time::now();
        double stamp = ros_stamp.toSec();	

        static double last_stamp = stamp;

        double dt_run = stamp - last_stamp;

        double car_yaw_steering_rad = car_steering_rad - car_yaw_rad;    
    
        double yaw_rate_rad = car_velocity/1.04 * car_yaw_steering_rad;
        yaw_gain_deg = yaw_rate_rad*rad2deg*dt_run;
        car_yaw_rad = car_yaw_rad + yaw_gain_deg*deg2rad;

        double dist = car_velocity*dt_run;

        car_position.first = car_position.first + dist*cos(car_steering_rad);
        car_position.second = car_position.second + dist*sin(car_steering_rad);

        car_pos.x = car_position.first;
        car_pos.y = car_position.second;
        car_pos.z = 0.0;

        car_position_pub.publish(car_pos);
        
        Rviz_car();
        
        car_velocity = 0.0;

        loop.sleep();

        last_stamp = stamp;

		ros::spinOnce();
    }

    return 0;
}
