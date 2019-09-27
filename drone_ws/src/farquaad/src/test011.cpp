//Collect keyboard input and drive drone
#include <ros/ros.h>
#include <termios.h>
//#include <ncurses.h>

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt16.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"


//int getch(void);
void takeInput(const std_msgs::UInt16& msg);
void powerCtrl(void); //Set to Takeoff, Land, or Emergency
void droneCtrl(void); //move drone (after takeoff)
void forwardBack(void); //move forward and backwards
void LandR(void); //move left and right
void UandD(void); //move up and down
void ctrlPub(char type, float offset);
void rotat(void); //angular angle
void hover(void);
ros::Publisher reset, takeoff, land, control;
ros::Subscriber input;
unsigned int c = 0; //character input
geometry_msgs::Twist data; //data->linear.x or something


int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_control");
	ros::NodeHandle n;
	reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
	takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	land = n.advertise<std_msgs::Empty>("ardrone/land", 1);
	control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	input = n.subscribe("farquaad/keyInput", 1, takeInput);
	
	ros::Rate loop_rate(30);
	
	
	while (ros::ok())
	{
		//hover();
		//c = getch();
		
		powerCtrl();
		droneCtrl();
		c = 0;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


/*int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}*/

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
	forwardBack();
	LandR();
	UandD();
	rotat();
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

void forwardBack()
{
	if(c == 119)//w
		ctrlPub('x', 1.0); //forward
	
	else if(c == 115)//s
		ctrlPub('x', -1.0); //backward
	
	else
		ctrlPub('x', 0.0); //stay
}

void LandR()
{
	if(c == 97)//a
		ctrlPub('y', 1.0); //left
	
	else if(c == 100)//d
		ctrlPub('y', -1.0); //right
	
	else
		ctrlPub('y', 0.0); //stay
}

void UandD()
{
	if(c == 105)//i
		ctrlPub('z', 1.0); //up
	
	else if(c == 107)//k
		ctrlPub('z', -1.0); //down
	
	else
		ctrlPub('z', 0.0); //stay
}

void rotat()
{
	if(c == 106)//j
		ctrlPub('a', 1.0); //up
	
	else if(c == 108)//l
		ctrlPub('a', -1.0); //down
	
	else
		ctrlPub('a', 0.0); //stay
}

void hover()
{
	ctrlPub('a', 0.0);
	ctrlPub('x', 0.0);
	ctrlPub('y', 0.0);
	ctrlPub('z', 0.0);
	
}

void takeInput(const std_msgs::UInt16& msg)
{
  	c = msg.data;
}
