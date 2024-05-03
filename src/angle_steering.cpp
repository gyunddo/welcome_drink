#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

using namespace std;

ros::Publisher car_control_pub;
ros::Publisher rviz_target_point_pub;

float car_steering_rad = 0.0;
double deg2rad = M_PI/180;
double rad2deg = 180/M_PI;
double car_pos_x, car_pos_y = 0.0;

//////////////////////////////////////////////////////////
std::pair<double ,double> target_point = {5.0, 2.76756905};     //{X, Y}  unit : [m]
        
double car_velocity = 0.3;                                      //unit : [m/s]
/////////////////////////////////////////////////////////

float Calculate_steering()
{
    double dx = (target_point.first - car_pos_x);
    double dy = (target_point.second - car_pos_y);

    double dist = sqrt(dx*dx + dy*dy);

    double alpha = atan2(dy, dx);

    float steering_rad = alpha;
    
    return steering_rad;
}

void Car_pos_Callback(const geometry_msgs::Point::ConstPtr &car_pos)
{
    car_pos_x = car_pos->x;
    car_pos_y = car_pos->y; 
}

void Publish_control_value()
{
    geometry_msgs::Twist car_control;

    car_control.linear.x = car_velocity;
    car_control.angular.z = car_steering_rad;

    car_control_pub.publish(car_control);
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
	ros::init(argc, argv, "edu_angle_steering_node");
	ros::NodeHandle nh;

    ros::Rate loop(8);

    ros::Subscriber car_position_sub = nh.subscribe<geometry_msgs::Point>("/car_position", 10, &Car_pos_Callback);

    car_control_pub = nh.advertise<geometry_msgs::Twist> ("/control_value", 10);
    rviz_target_point_pub = nh.advertise<visualization_msgs::Marker> ("/rviz_target_point", 10);

    geometry_msgs::Twist car_control;

	while(ros::ok())
    {	
        car_steering_rad = Calculate_steering();

        Publish_control_value();

        Rviz_target_point();
		
        cout << "Target point position(X, Y)[m]" << endl;
        cout << target_point.first << ", " << target_point.second << endl;
        cout << " " << endl;
	
        cout << "Car velocity[m/s] : " << car_velocity << endl;
        cout << "Car steering[deg] : " << car_steering_rad*rad2deg << endl;
        cout << " " << endl;

        cout << "Car position(X, Y)[m]" << endl;
        cout << car_pos_x << ", " << car_pos_y << endl;
        cout << " " << endl;

        cout << "****************************" << endl;
        cout << " " << endl;

        loop.sleep();

		ros::spinOnce();
	}
	
    return 0;
}
