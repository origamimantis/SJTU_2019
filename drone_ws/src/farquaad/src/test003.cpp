#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"

int st = 0;

void checkState(ardrone_autonomy::Navdata msg)
{
	st = msg.state;
  	std::cout << "Current State: " << st << std::endl;
}

int main(int argc, char **argv)
{
	
  	ros::init(argc, argv, "battery_read");
  	ros::NodeHandle n;
	ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1000);
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
	ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
	ros::Subscriber state = n.subscribe("ardrone/navdata", 1000, checkState);
	ros::Rate loop_rate(0.5);
	
	while (ros::ok())
	{
		if(st <= 2)
			takeoff.publish(std_msgs::Empty());
		else
			land.publish(std_msgs::Empty());
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
