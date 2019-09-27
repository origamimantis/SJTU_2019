#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"



int main(int argc, char** argv)
{
  int state = 0;
  ros::init(argc, argv, "ud");
  ros::NodeHandle n;
  
  ros::Publisher toff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
  ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land", 1000);

  ros::Rate l_r(0.5);

  while (ros::ok())
  {
    if (state == 0)
    {
      toff.publish(std_msgs::Empty());
      state = 1 - state;
    }
    else if ( state == 1 )
    {
      land.publish(std_msgs::Empty());
      state = 1 - state;
    }
    ros::spinOnce();
  }

}


