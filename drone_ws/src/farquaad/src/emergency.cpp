#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"


int main(int argc, char **argv)
{
	
  	ros::init(argc, argv, "reset");
  	ros::NodeHandle n;
	ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1000);
	reset.publish(std_msgs::Empty());
	return 0;
}
