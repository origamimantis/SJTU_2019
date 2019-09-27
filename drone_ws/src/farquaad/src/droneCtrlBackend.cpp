//Collect keyboard input and drive drone
#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"

int getch(void);
void powerCtrl(void); //Set to Takeoff, Land, or Emergency
void droneCtrl(void); //move drone (after takeoff)
void ctrlPub(char, float);
void publish(void);
ros::Publisher reset, takeoff, land, control;
int c = 0; //character input
float dx = 0;
float dy = 0;
float dz = 0;
float da = 0;
bool keyDown = false;
geometry_msgs::Twist data; //data->linear.x or something

void dataCB( const std_msgs::UInt16& msg)
{
	c = msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
	takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	land = n.advertise<std_msgs::Empty>("ardrone/land", 1);
	control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber input = n.subscribe("farquaad/keyInput", 1, dataCB);
	ros::Rate loop_rate(20);
	
	
	while (ros::ok())
	{
		
		powerCtrl();
		droneCtrl();
		publish();
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
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
  if(c == 119) { dx += 1; } // w - forward
  if(c == 115) { dx -= 1; } // s -backward

  if(c == 97) dy += 1; //left a
  if(c == 100) dy -= 1; //right d

  if(c == 105) dz += 1; //up i
  if(c == 107) dz -= 1; //down k

  if(c == 106) da += 1;// up j
  if(c == 108) da -= 1;// down l
}

void publish()
{
  ctrlPub('x', dx);
  dx = 0;
  ctrlPub('y', dy);
  dy = 0;
  ctrlPub('z', dz);
  dz = 0;
  ctrlPub('a', da);
  da = 0;
  c = 0;
}


void ctrlPub(char type, float offset)
{
	if(type == 120)
	{
		data.linear.x = offset;
		control.publish(data);
	}
	else if(type == 121)
	{
		data.linear.y = offset;
		control.publish(data);
	}
	else if(type == 122)
	{
		data.linear.z = offset;
		control.publish(data);
	}
	else if(type == 97)
	{
		data.angular.z = offset;
		control.publish(data);
	}
}
