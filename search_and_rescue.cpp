#include "ros/ros.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "gnc_functions.hpp"

int mode_g = 0;

void sr_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for (int i = 0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());
		if(msg->bounding_boxes[i].Class == "person")
		{
			mode_g = 1;
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "search_and_rescue");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, sr_cb);

	init_publisher_subscriber(nh);

	wait4connect();

	wait4start();

	initialize_local_frame();

	takeoff(10);

	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWaypoint;
	float rows = 5;
	float row;
	int spacing = 10;
	int range = 50;

	for (int i = 0; i < rows; i++)
	{
		row = i*2;
		nextWaypoint.x = row*spacing;
		nextWaypoint.y = 0;
		nextWaypoint.z = 10;
		nextWaypoint.psi = 0;
		waypointList.push_back(nextWaypoint);

		nextWaypoint.x = row*spacing;
		nextWaypoint.y = range;
		nextWaypoint.z = 10;
		nextWaypoint.psi = 0;
		waypointList.push_back(nextWaypoint);

		nextWaypoint.x = (row+1)*spacing;
		nextWaypoint.y = range;
		nextWaypoint.z = 10;
		nextWaypoint.psi = 0;
		waypointList.push_back(nextWaypoint);

		nextWaypoint.x = (row+1)*spacing;
		nextWaypoint.y = 0;
		nextWaypoint.z = 10;
		nextWaypoint.psi = 0;
		waypointList.push_back(nextWaypoint);
		
	}

	ros::Rate rate(2.0);
	int counter;

	while(ros::ok())
	{
		if (mode_g == 0)
		{
			ros::spinOnce();
			rate.sleep();
		
			if (check_waypoint_reached(0.3))
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
					counter++;
				}
				else
				{
					land();
				}
			}
		}
		if (mode_g == 1)
		{
			land();
			ROS_INFO("Rescue operation started");
		}

	}

	return 0;
}