#include "ros/ros.h"
#include "gnc_functions.hpp"
#include "sensor_msgs/LaserScan.h"

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan current_2D_scan;
	current_2D_scan = *msg;
	float avoidance_vector_x = 0;
	float avoidance_vector_y = 0;
	bool avoid = false;
	float d0 = 3;
	float k = 0.5;

	for (int i = 1; i < current_2D_scan.ranges.size(); ++i)
	{
		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > 0.3)
		{
			avoid = true;
			float X = cos(current_2D_scan.angle_increment*i);
			float Y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + X*U;
			avoidance_vector_y = avoidance_vector_y + Y*U;
		}
	}

	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	
	if(avoid)
	{
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);	
	}


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_avoidance");
	ros::NodeHandle nh;

	ros::Subscriber collision_sub = nh.subscribe<sensor_msgs::LaserScan>("spur/laser/scan", 1, scan_cb );

	init_publisher_subscriber(nh);

	wait4connect();

	wait4start();

	initialize_local_frame();

	takeoff(2);

	set_destination(0,0,2,0);

	ros::Rate rate(2.0);
	int counter = 0;

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}