#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"


void dataCB( const ardrone_autonomy::Navdata& msg)
{
  ROS_INFO( "Battery: %f, State: %d, Alt: %d" , msg.batteryPercent, msg.state, msg.altd);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "info");
  ros::NodeHandle n;
  
  ros::Subscriber s = n.subscribe("ardrone/navdata", 100, dataCB);

  ros::spin();

}


