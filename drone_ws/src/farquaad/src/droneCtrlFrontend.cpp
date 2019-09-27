//Collect keyboard input and drive drone
#include <ros/ros.h>
#include <termios.h>
#include <sstream>
#include "std_msgs/UInt16.h"

int getch(void);
ros::Publisher typewriter;
int c = 0; //character input


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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FrontEnd");
	ros::NodeHandle n;
	typewriter = n.advertise<std_msgs::UInt16>("farquaad/keyInput", 1);
	ros::Rate loop_rate(100);
	
	
	while (ros::ok())
	{
		
		c=getch();
		std_msgs::UInt16 s;
		s.data = c;
		typewriter.publish(s);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
