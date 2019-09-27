#include <iostream>

#include "ros/ros.h"
#include "std_msgs/UInt16.h"


void dataCB( const std_msgs::UInt16& msg)
{
  ROS_INFO( "%d\n" , msg.data);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyinfo");
  ros::NodeHandle n;
  
  ros::Subscriber s = n.subscribe("farquaad/keyInput", 100, dataCB);

  ros::spin();

}


