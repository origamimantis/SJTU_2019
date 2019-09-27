#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <ardrone_autonomy/Navdata.h>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>



void updateImg( const sensor_msgs::ImageConstPtr& msg )
{
  cv::imshow("frontcam", cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
  cv::waitKey(1);
}


int main(int argc, char** argv)
{
  int state = 0;
  ros::init(argc, argv, "frontcamera");
  ros::NodeHandle n;

  // create image transporting handle from the node handle
  image_transport::ImageTransport i(n);
  
  image_transport::Subscriber camData = i.subscribe("ardrone/front/image_raw", 1000, updateImg);
  //ros::Subscriber camData = n.subscribe("ardrone/front/image_raw", 1000, updateImg);
  cv::namedWindow("frontcam");
  ros::Rate l_r(0.5);

  while (ros::ok())
  {
    ros::spin();
  }

}

