#include <iostream>
#include <string>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Matrices (images) to store the current frame and the frame after filtering
cv::Mat curImg, modImg;
// Keep track of which camera to use
int THECAMERA = 0;

// Window size
int width;
int height;

// Keep track of the central color to filter for
int trackg = 0;
int trackb = 0;
int trackr = 0;

// the amount of error allowed, out of 255
int r_err = 40;
int g_err = 40;
int b_err = 40;
	
// create a VideoCapture object
cv::VideoCapture vc;

// Global var for the tracking rectangle, to be defined later.
cv::Rect tracker;

// Used in CamShift
cv::TermCriteria term_crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);

// Creates some windows
void setUpWindows ()
{
	cv::namedWindow("Camera");
	cv::namedWindow("Filtered");
}

// Creates trackbars
void setUpTrackbars ()
{
	cv::createTrackbar(  "Red        ","Camera", &trackr, 255);
	cv::createTrackbar(  "Green      ","Camera", &trackg, 255);
	cv::createTrackbar(  "Blue       ","Camera", &trackb, 255);
	cv::createTrackbar(  "Red Error  ","Camera", &r_err, 255);
	cv::createTrackbar(  "Green Error","Camera", &g_err, 255);
	cv::createTrackbar(  "Blue Error ","Camera", &b_err, 255);
}

// Updates the video in each window
void updateWindows ( )
{
	cv::imshow( "Camera" , curImg );
	cv::imshow( "Filtered" , modImg );
}

// Reads an image and applies CamShift algorithm
void processImg ( )
{
	// Read one frame from camera input
	vc.read(curImg);

	// Find all pixels that lie in the bounds given by global ints and write them to modImg
	cv::inRange( curImg,                                                // Input img
		     cv::Scalar(trackb-b_err, trackg-g_err, trackr-r_err),  // lower bound calculation
		     cv::Scalar(trackb+b_err, trackg+g_err, trackr+r_err),  // upper bound
		     modImg);                                               // Output img
	
	// Use CamShift - detection algorithm
	cv::RotatedRect rot_rect = cv::CamShift(modImg, tracker, term_crit);
	
	// Convert modImg to bgr (colored) so the tracking box shows up
	cv::cvtColor( modImg, modImg, cv::COLOR_GRAY2BGR);
	
	// Store the vertices of the detection frame
	cv::Point2f points[4];
	rot_rect.points(points);
	
	// Connect the dots
	for (int i = 0; i < 4; i++)
	{
		// Use negative color on normal video for contrast, constant color on filtered
		cv::line(curImg, points[i], points[(i+1)%4], cv::Scalar(255-trackb,255-trackg,255-trackr), 2);	
		cv::line(modImg, points[i], points[(i+1)%4], cv::Scalar(0,255,0), 2);	
	}
	// center of tracker box
	cv::circle(curImg, rot_rect.center, 1, cv::Scalar(255-trackb,255-trackg,255-trackr), 1);	
	cv::circle(modImg, rot_rect.center, 1, cv::Scalar(0,155,0), 2);	
}	


int main(int ac, char** av)
{
	// If command line arg "1", use webcam.
	if (ac >= 2) { std::string  opt =  av[1] ; THECAMERA = 1 ? (opt == "1") : 0;}
	
	// Connect to the camera
	vc.open(THECAMERA);
	
	// Make sure the camera is on
	if (! vc.isOpened()) { return -1; }

	// read one frame to get te size of the image - used to set size of tracking window
	vc >> curImg;
	height = curImg.size().height;
	width = curImg.size().width;
	
	// Create the windows
	setUpWindows();

	// Create the scrolly bars
	setUpTrackbars();
	
	// create the box for showing the track
	tracker = cv::Rect(0,0,width,height);

	// Initialize exit condition
	int exiter = 0;

	while (exiter != 27)
	{
		// Read the next frame from camera and process it
		processImg();

		// Add the current color being detected as a "border" at the bottom
		cv::copyMakeBorder( curImg, curImg,                      // images to edit
				    0, 50, 0, 0,                         // only add 50 pixels at the bottom
				    cv::BORDER_CONSTANT,                 // Border is a solid color
				    cv::Scalar(trackb,trackg,trackr) );  // Color to use
		// Update the video
		updateWindows();

		// Update the exit condition
		exiter = cv::waitKey(1);
	}
	std::cout << "You left..." << std::endl;
	return 0;
}
