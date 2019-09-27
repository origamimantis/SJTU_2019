#include <iostream>
#include <string>

#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat curImg, modImg;
int trackg = 0;
int trackb = 0;
int trackr = 0;
int r_err = 40;
int g_err = 40;
int b_err = 40;




int main(int ac, char** av)
{
	std::cout << ac;
	std::string name;
	cv::VideoCapture vc;
	
	vc.open(0);
	
	if (! vc.isOpened())
	{
		std::cout << "Ya hecked up, turn on your webcam!" << std::endl;
		return -1;
	}


	vc  >> curImg;
                
	cv::namedWindow("Camera");
	cv::createTrackbar(  "Red        ","Camera", &trackr, 255);
	cv::createTrackbar(  "Green      ","Camera", &trackg, 255);
	cv::createTrackbar(  "Blue       ","Camera", &trackb, 255);
	cv::createTrackbar(  "Red Error  ","Camera", &r_err, 255);
	cv::createTrackbar(  "Green Error","Camera", &g_err, 255);
	cv::createTrackbar(  "Blue Error ","Camera", &b_err, 255);

	while (1)
	{
		if (!vc.read(curImg))
		{
			std::cout << "Something broke, image not readable" << std::endl;
			break;
		}

		cv::inRange( curImg, 
				cv::Scalar(trackb-b_err, trackg-g_err, trackr-r_err),
				cv::Scalar(trackb+b_err, trackg+g_err, trackr+r_err),
				modImg);
		
		cv::imshow( "Camera" , curImg );
		cv::imshow( "Filtered" , modImg );

		if (cv::waitKey(1) == 27)
		{
			std::cout << "You left..." << std::endl;
			break;
		}
	}
	return 0;
}
