#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>	

using namespace std;

int sum_point = 0;
double roi_distance = 1.5;	 	//unit : m		

double Rad2Deg(double rad)
{
	double deg = rad * 180/M_PI;
	return deg;
}

double Deg2Rad(double deg)
{
	double rad = deg * M_PI/180;
	return rad;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
{
    int count = scan->scan_time / scan->time_increment;

    sum_point = 0;

    for(int i=0; i<count; i++) 
	{
		double point_rad = scan->angle_min + scan->angle_increment * i;
        double point_deg = Rad2Deg(point_rad);

        if((point_deg >= -1*30)&&(point_deg <= 30)) 
        {
			if((scan->ranges[i] <= roi_distance)&&(scan->ranges[i] >= scan->range_min))
			{
				sum_point++;
			}
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lidar_detect_node");
	ros::NodeHandle nh;

	ros::Rate rate(8);

	ros::Subscriber lidar_sub = nh.subscribe("/scan", 100, &scanCallback);
	
	while(ros::ok())
	{
		if(sum_point > 10)
		{
			ROS_INFO("-----Object Detect!-----");
			ROS_INFO("Sum : %d", sum_point);
			cout << " " << endl;
		}
		else
		{
			ROS_INFO("-------------------------");
		}
		
		ros::spinOnce();
		
		rate.sleep();
	}
	
	ros::spin();

	return 0;
}