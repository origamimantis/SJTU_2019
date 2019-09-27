#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>


int main( int ac, char** av)
{
	std::string path = "oops.png";
	if (ac > 1)
	{
		path = av[1];
	}
	cv::Mat thePicture = cv::imread( path.c_str() , cv::IMREAD_COLOR );
	if (thePicture.empty())
	{
		std::cout << "Ya hecked up!" << std::endl;
		return -1;
	}
	
	cv::namedWindow( path.c_str(), cv::WINDOW_AUTOSIZE );
	cv::imshow( path.c_str(), thePicture );

	cv::waitKey(0);

	return 0;
}

