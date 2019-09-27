//Collect keyboard input and control drone
#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
int getch(void);

void checkAlt(ardrone_autonomy::Navdata msg)
{
  	std::cout << "Current Altitude: " << msg.altd << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_reset");
	ros::NodeHandle n;
	ros::Publisher reset = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
	ros::Publisher takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	ros::Publisher land = n.advertise<std_msgs::Empty>("ardrone/land", 1);
	ros::Subscriber alt = n.subscribe("ardrone/navdata_altitude", 1, checkAlt);
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		int c = 0;
		c=getch();
		//printf("%c \n",c); //what key is entered
		if(c == 32)//SPACE
			reset.publish(std_msgs::Empty());
		else if(c == 119)//w
			takeoff.publish(std_msgs::Empty());
		else if(c == 115)//s
			land.publish(std_msgs::Empty());
		
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

