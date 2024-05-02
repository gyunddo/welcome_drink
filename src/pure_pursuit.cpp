#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher rviz_car_pub;
ros::Publisher rviz_target_point_pub;

double deg2rad = M_PI/180;
double rad2deg = 180/M_PI;
double yaw_gain_deg, car_yaw_rad = 0.0;

std::pair<double, double> car_position = {0.0, 0.0};

//////////////////////////////////////////////////////////
std::pair<double ,double> target_point = {5.0, 2.76756905};
        
double car_velocity = 0.3; 
double look_distance = 3.0;
/////////////////////////////////////////////////////////

float Calculate_steering()
{
    float steering_rad;

    double dx = (target_point.first - car_position.first);
    double dy = (target_point.second - car_position.second);

    double dist = sqrt(dx*dx + dy*dy);

    double alpha = atan2(dy, dx);

    steering_rad = atan2((2*1.04*sin(alpha)), look_distance);

    return steering_rad;
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

void Rviz_target_point()
{
    visualization_msgs::Marker t_point;

    t_point.header.frame_id = "edu_frame";
    t_point.header.stamp = ros::Time::now();
    t_point.ns = "target_point";
    t_point.id = 0;
    t_point.type = visualization_msgs::Marker::SPHERE;
    t_point.action = visualization_msgs::Marker::ADD;
    t_point.lifetime = ros::Duration(0.2);

    t_point.pose.position.x = target_point.first;
    t_point.pose.position.y = target_point.second;
    t_point.pose.position.z = 0;

    t_point.scale.x = 0.3;
    t_point.scale.y = 0.3;
    t_point.scale.z = 0.0;

    t_point.color.r = 0.0;      
    t_point.color.g = 1.0;     
    t_point.color.b = 0.0;
    
    t_point.color.a = 1.0;

    rviz_target_point_pub.publish(t_point);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "edu_node");
	ros::NodeHandle nh;

    ros::Rate loop(10);

    rviz_car_pub = nh.advertise<visualization_msgs::Marker> ("/rviz_car_position", 10);
    rviz_target_point_pub = nh.advertise<visualization_msgs::Marker> ("/rviz_target_point", 10);

	while(ros::ok())
    {	
        ros::Time ros_stamp = ros::Time::now();
        double stamp = ros_stamp.toSec();	

        static double last_stamp = stamp;

        double dt_run = stamp - last_stamp;

        float car_steering_rad = Calculate_steering();
        float car_yaw_steering_rad = car_steering_rad - car_yaw_rad;

        double yaw_rate_rad = car_velocity/1.04 * car_yaw_steering_rad;
        yaw_gain_deg = yaw_rate_rad*rad2deg*dt_run;
        car_yaw_rad = car_yaw_rad + yaw_gain_deg*deg2rad;

        double dist = car_velocity*dt_run;

        car_position.first = car_position.first + dist*cos(car_steering_rad);
        car_position.second = car_position.second + dist*sin(car_steering_rad);
        
        Rviz_car();
        Rviz_target_point();

        std::cout << "Look distance : " << look_distance << std::endl;
        std::cout << " " << std::endl;
        
        std::cout << "Target point position(X, Y)[m]" << std::endl;
        std::cout << target_point.first << ", " << target_point.second << std::endl;
        std::cout << " " << std::endl;
	
        std::cout << "Car velocity[m/s] : " << car_velocity << std::endl;
        std::cout << "Car steering[deg] : " << car_steering_rad*rad2deg << std::endl;
        std::cout << "Car Yaw steering[deg] : " << car_yaw_steering_rad*rad2deg << std::endl;
        std::cout << "Car yaw[deg] : " << car_yaw_rad*rad2deg << std::endl;
        std::cout << " " << std::endl;

        std::cout << "Car position(X, Y)[m]" << std::endl;
        std::cout << car_position.first << ", " << car_position.second << std::endl;
        std::cout << " " << std::endl;

        std::cout << "****************************" << std::endl;
        std::cout << " " << std::endl;

        loop.sleep();

        last_stamp = stamp;

		ros::spinOnce();
	}
	ros::spin();
	
    return 0;
}
