#include <iostream>
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/photo.hpp>
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
int trackr = 0;
int trackg = 0;
int trackb = 0;

// the amount of error allowed, out of 255
int r_err = 40;
int g_err = 40;
int b_err = 40;

// Constant for lower bound of gray threshold
int grayb = 150;

// Stores previous points in history
std::vector<cv::Point2f> ptHist;
int QMaxed = 0;     // Acts as a bool
int trackhl = 50;   // How many frames to remember

// Toggles for drawing tracking shapes
int trackon = 1;
int trackbon = 1;
	
// create a VideoCapture object
cv::VideoCapture vc;

// Declare global var for the tracking rectangle, to be defined later.
cv::Rect tracker;

// Used in CamShift
cv::TermCriteria term_crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);

// Creates some windows
void setUpWindows ()
{
	cv::namedWindow("Camera");
	cv::namedWindow("Bars");
	cv::namedWindow("Filtered");
}

// Creates trackbars
void setUpTrackbars ()
{
	cv::createTrackbar(  "Red "            ,"Bars", &trackr  , 255);
	cv::createTrackbar(  "Green"           ,"Bars", &trackg  , 255);
	cv::createTrackbar(  "Blue"            ,"Bars", &trackb  , 255);
	cv::createTrackbar(  "Red Error"       ,"Bars", &r_err   , 255);
	cv::createTrackbar(  "Green Error"     ,"Bars", &g_err   , 255);
	cv::createTrackbar(  "Blue Error"      ,"Bars", &b_err   , 255);
	cv::createTrackbar(  "TrackBox Enable" ,"Bars", &trackbon, 1  );
	cv::createTrackbar(  "MoveTrack Enable","Bars", &trackon , 1  );
	cv::createTrackbar(  "MoveTrack Amount","Bars", &trackhl , 150);
	cv::createTrackbar(  "Noise Reduction ","Bars", &grayb   , 255);
}

// Updates the video in each window
void updateWindows ( )
{
	cv::imshow( "Camera" , curImg );
	cv::imshow( "Filtered" , modImg );
}


void updateQ( cv::Point2f pt )
{
	// Make sure the tail is only as long as specified
	while (ptHist.size() > trackhl) { ptHist.erase(ptHist.begin()); }

	// If the center is on screen, add to history
	if (pt.x + pt.y) { QMaxed = 0; ptHist.push_back(pt); }

	// if the vector is too long, remove the element that was there earliest (at the beginning)
	if (ptHist.size() > trackhl ) { QMaxed = 1; }

	// if the vector is very shor, disable removing items
	else if (ptHist.size() <= 1 ) { QMaxed = 0; }

	// if the thing isn't being tracked, start deleting points
	else if (!(pt.x+pt.y)) { QMaxed = 1; }

	// Delete points
	if (QMaxed) {ptHist.erase(ptHist.begin()); }
}


void drawLines()
{
	int s = ptHist.size();   // Store, used many times
	for (int i = 0 ; i < s-1;  i++)
	{
		// scale the color being shown on the tail
		float pc = 0.4 + 0.4*(1-(s-i)/(float)s);

		cv::line(curImg,                                              // Draw on the unfiltered image
			ptHist[i], ptHist[i+1],                       // Line is between consequetive pts
			cv::Scalar((255-trackb)*pc,(255-trackg)*pc,(255-trackr)*pc), // Still negative color, but
				2);                                                  // gets darker as time passes
		cv::line(modImg,
			ptHist[i], ptHist[i+1],
			cv::Scalar( 0 ,(255)*pc, 0 ),
				2);
	}
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
	
	// Denoise, but very slow
	//cv::fastNlMeansDenoising(modImg, modImg ,3);
	/*
	int t = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS,
			cv::Size( 2*t+1, 2*t+1 ),
			cv::Point( t, t ) );
	cv::erode( modImg, modImg, element );
	cv::dilate( modImg, modImg, element );
	*/

	for (int i = 0; i < 2; i++)
	{
		cv::blur( modImg, modImg, cv::Size( 10, 10 ), cv::Point(-1,-1) );

		// Re-threshold using gray as bounds
		cv::inRange( modImg,
			     cv::Scalar( grayb , grayb , grayb ),    // Grayish lower bound
			     cv::Scalar( 260 , 260 , 260 ),    // upper bound is just white
			     modImg);                          // Output img
	}
	int t = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS,
			cv::Size( 2*t+1, 2*t+1 ),
			cv::Point( t, t ) );
	cv::erode( modImg, modImg, element );
	// Use CamShift - detection algorithm
	cv::RotatedRect rot_rect = cv::CamShift(modImg, tracker, term_crit);
	
	// Convert modImg to bgr (colored) so the tracking box shows up
	cv::cvtColor( modImg, modImg, cv::COLOR_GRAY2BGR);
	
	if (rot_rect.size.width*rot_rect.size.height <= 0.007*modImg.size().width*modImg.size().height)
	{
		rot_rect = cv::RotatedRect();
	}
	
	// Store the vertices of the detection frame
	cv::Point2f points[4];
	rot_rect.points(points);
	
	if (trackbon)
	{
		// Connect the dots
		for (int i = 0; i < 4; i++)
		{
			// Use negative color on normal video for contrast, constant color on filtered
			cv::line(curImg,points[i],points[(i+1)%4],cv::Scalar(255-trackb,255-trackg,255-trackr),2);
			cv::line(modImg,points[i],points[(i+1)%4],cv::Scalar(0,255,0), 2);  // Color in BGR
		}
	}
	

	// Add center point to history
	updateQ( rot_rect.center );

	// Trace history if enabled
	if (trackon)
	{
		// Draw center of tracker box
		cv::circle(modImg, rot_rect.center, 1, cv::Scalar(0,155,0), 2);	
		cv::circle(curImg, rot_rect.center, 1, cv::Scalar(255-trackb,255-trackg,255-trackr), 2);
		drawLines(); 
	}
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
	std::cout << "You left... ;-;" << std::endl;
	return 0;
}
