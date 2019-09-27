#include <iostream>
#include <string>

#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int ac, char** av)
{
	std::cout << ac;
	std::string name;
	cv::Mat curImg;
	cv::VideoCapture vc;
	
	vc.open(0);
	name = "Camera";
	
	if (! vc.isOpened())
	{
		std::cout << "Ya hecked up!" << std::endl;
		return -1;
	}
	vc  >> curImg;
	//cv::VideoWriter theVid("CameraOutput.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(vc.get(cv::CAP_PROP_FRAME_WIDTH), vc.get(cv::CAP_PROP_FRAME_HEIGHT)));
	cv::VideoWriter theVid;
	theVid.open("CameraOutput.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, curImg.size(), 1);
		// change extension, letters in fourcc, framerate
	if (!theVid.isOpened()) { 
		std::cout <<"heck";
		return -1; }
	
	while (1)
	{
		if (!vc.read(curImg)) {break;}

		theVid.write(curImg);

		cv::imshow( name , curImg );

		if (cv::waitKey(1) >= 0) {break;}
	}
	return 0;
}
