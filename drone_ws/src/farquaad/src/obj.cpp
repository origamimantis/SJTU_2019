#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

// Matrices (images) to store the current frame and the frame after filtering
cv::Mat curImg, modImg, shpImg;
cv::Mat basImg;
std::vector<std::vector<cv::Point> > baseCntr;
std::string defaultImg = "circle.png";

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

// for lower bound of gray threshold
int grayb = 150;

// for similarity of shape checks
int trackContourError = 10;
int trackMinArea = 1000;

// Used in cv::erode
cv::Mat element = cv::getStructuringElement( cv::MORPH_CROSS,
		cv::Size( 3, 3 ),
		cv::Point( 1, 1 ) );


// Stores previous points in history
std::vector<cv::Point2f> ptHist;
int QMaxed = 0;     // Acts as a bool
int trackhl = 50;   // How many frames to remember

// Toggles for drawing tracking shapes
int trackon = 1;
int trackbon = 1;
	
// create a VideoCapture object
cv::VideoCapture vc;

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
	cv::createTrackbar(  "Shape det err(%)","Bars", &trackContourError , 50);
	cv::createTrackbar(  "Shape min area  ","Bars", &trackMinArea   , width*height/10);
}

// Updates the video in each window
void updateWindows ( )
{
	cv::imshow( "Camera" , curImg );
	cv::imshow( "Filtered" , modImg );
	cv::imshow( "Matched" , shpImg );
}


void updateQ( cv::Point2f pt )
{
	// Make sure the tail is only as long as specified
	while (ptHist.size() > trackhl) { ptHist.erase(ptHist.begin()); }
	
	// If the center is on screen, add to history
	if (pt.x>0 && pt.y>0) { QMaxed = 0; ptHist.push_back(pt); }

	// if the vector is too long, remove the element that was there earliest (at the beginning)
	if (ptHist.size() > trackhl ) { QMaxed = 1; }

	// if the vector is very short, disable removing items
	else if (ptHist.size() == 0 ) { QMaxed = 0; }

	// if the thing isn't being tracked, start deleting points
	else if (!(pt.x>0 && pt.y>0)) { QMaxed = 1; }

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
			cv::Scalar((255-trackg)*pc,(255-trackb)*pc,(255-trackr)*pc), // Still negative color, but
				2);                                                  // gets darker as time passes
		cv::line(shpImg,
			ptHist[i], ptHist[i+1],
			cv::Scalar( 0 ,(255)*pc, 0 ),
				2);
	}
}

void deNoise( cv::Mat* mat )
{
	
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
		cv::blur( *mat, *mat, cv::Size( 10, 10 ), cv::Point(-1,-1) );

		// Re-threshold using gray as bounds
		cv::inRange( *mat,
			     cv::Scalar( grayb , grayb , grayb ),    // Grayish lower bound
			     cv::Scalar( 260 , 260 , 260 ),    // upper bound is just white
			     *mat);                          // Output img
	}
	cv::erode( *mat, *mat, element );
	
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
	
	// denoise the image
	deNoise( &modImg );

	// to store contours (each contour is stored as a vector of endpoints (?) )
	std::vector<std::vector<cv::Point> > contrs;

	// Makes shpImg purely black, so the match is visible later
	cv::inRange( modImg,cv::Scalar( 260 , 260 , 260 ),cv::Scalar( 300 , 300 , 300 ),shpImg);

	// get rid of the small shapes left over - these easily match the input shape
	deNoise( &shpImg );
	
	// find contours, storing them into contrs
	cv::findContours(modImg, contrs, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_TC89_KCOS);

	// Filter out contours by similarity to input image (lower is closer)
	std::vector<std::vector<cv::Point> > matches;
	
	int bestMatch = 100;

	for (int i = 0; i < contrs.size(); i++)
	{
		float t = cv::matchShapes(baseCntr[0], contrs[i] ,1,0);
		// add contour to matches if error is less than specified thres, in %
		if (100*t <= trackContourError && cv::contourArea(contrs[i]) >= trackMinArea)
		{
			// put the best match (least error) in index 0, so we can track its movement
			if ( t < bestMatch )
			{
				bestMatch = t;
				matches.insert(matches.begin(), contrs[i]);
			}
			else
			{
				// otherwise, store it but don't care about position.
				matches.push_back( contrs[i] );
			}
		}
	}

	// Draw the matched shapes using the color selected
	cv::cvtColor( shpImg, shpImg, cv::COLOR_GRAY2BGR);

	// if no match (assume by default), make the center (0,0) so updateQ ignores it but still updates the animation
	cv::Point2f center(0,0);
	
	// only calculate center of mass if the match exists - avoids segfault when no match
	if (matches.size())
	{

		// find center of mass of shape
		cv::Moments m = cv::moments( matches[0] );
		center = cv::Point2f(m.m10 / m.m00,  m.m01 / m.m00);
	}

	// add center to the stored list of centers
	updateQ(center);
	
	// Always draw the filled in contours
	cv::drawContours(shpImg, matches, -1, cv::Scalar(255,255,255), cv::FILLED);

	// draw shape outline if enabled
	if (trackbon)
	{
		// 0 argument (index 2) makes it draw the best match (index 0 in the vector)
		cv::drawContours(shpImg, matches, 0, cv::Scalar(0,155,0), 2);
		cv::drawContours(curImg, matches, 0, cv::Scalar(255-trackg,255-trackb,255-trackr), 2);
	}

    // Trace history if enabled
    if (trackon)
    {
        // Draw center of shape outline
		if (center.x > 0 && center.y > 0)
		{
			cv::circle(shpImg, center, 1, cv::Scalar(0,155,0), 2);
			cv::circle(curImg, center, 1, cv::Scalar(255-trackg,255-trackb,255-trackr), 2);
		}
        drawLines();
    }
}	
int main(int ac, char** av)
{
	// If command line arg "1", use webcam. (Dropped in favor of selecting base image)
	//if (ac >= 2) { std::string  opt =  av[1] ; THECAMERA = 1 ? (opt == "1") : 0;}

	if (ac >= 2)
	{
		std::string imgToOpen = av[1];
		
		// read image as grayscale
		basImg = cv::imread(imgToOpen, cv::IMREAD_GRAYSCALE);
		if (basImg.empty())
		{
			std::cout << "Error loading image, shape = 'circle.png'"<<std::endl;
			basImg = cv::imread(defaultImg, cv::IMREAD_GRAYSCALE);
		}
	}
	else
	{
		std::cout << "No arg given, shape = 'circle.png'"<<std::endl;
		basImg = cv::imread(defaultImg, cv::IMREAD_GRAYSCALE);
	}
	
	// Connect to the camera
	vc.open(THECAMERA);
	
	// Make sure the camera is on
	if (! vc.isOpened()) { return -1; }

	// read one frame to get te size of the image - used to set size of tracking window
	vc >> curImg;
	height = curImg.size().height;
	width = curImg.size().width;

	// Detect contours on input image, for matching later
	cv::findContours(basImg, baseCntr, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_TC89_KCOS);

	// Show the contour
	cv::imshow("Shape", basImg);
	
	// Create the windows
	setUpWindows();

	// Create the scrolly bars
	setUpTrackbars();
	
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
