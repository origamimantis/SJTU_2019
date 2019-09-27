#include <sstream>
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/video.hpp>
#include <vector>
#include <opencv2/videoio.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Empty.h"
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt16.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
float xpos, ypos;
ros::ServiceClient toggle, set;
ros::Publisher control;
std_srvs::Empty zer;
geometry_msgs::Twist data;
void getX(std_msgs::Float64 msg);
void getY(std_msgs::Float64  msg);
void getBoolFollow(std_msgs::Bool msg);
void ctrlPub(char type, float offset);
void moveTrack(void);
int w = 640;
int h = 410;
bool followMode = false;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Follow");
  	ros::NodeHandle n;
	set = n.serviceClient<std_srvs::Empty>("ardrone/setcamchannel"); 
	toggle = n.serviceClient<std_srvs::Empty>("/ardrone/togglecam"); //toggle between cam
	control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	set.call(zer); //set to front cam
	toggle.call(zer); //toggle to bottom camera
	while(ros::ok())
	{
		ros::Subscriber center_x = n.subscribe("centerX", 1, getX);
		ros::Subscriber center_y = n.subscribe("centerY", 1, getY);
		ros::Subscriber followMode = n.subscribe("follow", 1, getBoolFollow);
		if(followMode)
			moveTrack();
		ros::spinOnce();
	}
	
}

void getX(std_msgs::Float64 msg)
{
	xpos = msg.data;	
}

void getY(std_msgs::Float64 msg)
{
	ypos = msg.data;	
}

void moveTrack()
{
	if(xpos < w/2 && xpos != 0)
		ctrlPub('y', -1.0);
	else if(xpos > w/2)
		ctrlPub('y', 1.0);
	else
		ctrlPub('y', 0.0);
	
	if(ypos < h/2 && ypos != 0)
		ctrlPub('x', -1.0);
	else if(ypos > h/2)
		ctrlPub('x', 1.0);
	else
		ctrlPub('x', 0.0);
	
}

void getBoolFollow(std_msgs::Bool msg)
{
	followMode = msg.data;
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
