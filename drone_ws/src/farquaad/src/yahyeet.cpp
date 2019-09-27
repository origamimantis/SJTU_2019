//Collect keyboard input and drive drone
#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"

int getch(void);
void powerCtrl(void); //Set to Takeoff, Land, or Emergency
void checkAlt(ardrone_autonomy::Navdata msg); //check Altitude (mm)
void droneCtrl(void); //move drone (after takeoff)
ros::Publisher reset, takeoff, land, control;
ros::Subscriber alt;
int c = 0; //character input
float offset = 0;
geometry_msgs::Twist data; //data->linear.x or something

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
	takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	land = n.advertise<std_msgs::Empty>("ardrone/land", 1);
	control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	alt = n.subscribe("ardrone/navdata", 1, checkAlt);
	ros::Rate loop_rate(100);
	
	
	while (ros::ok())
	{
		
		c=getch();
		
		powerCtrl();
		droneCtrl();
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void checkAlt(ardrone_autonomy::Navdata msg)
{
  	std::cout << "Current Altitude: " << msg.altd  << "mm" << std::endl;
}

void powerCtrl()
{
	if(c == 32)//SPACE
		reset.publish(std_msgs::Empty());
	else if(c == 49)//1
		takeoff.publish(std_msgs::Empty());
	else if(c == 48)//0
		land.publish(std_msgs::Empty());
}

void droneCtrl()
{
	if(c == 119)//w
	{
		offset = 1.0;
		data.linear.x = offset;
		control.publish(data);
	}
}
