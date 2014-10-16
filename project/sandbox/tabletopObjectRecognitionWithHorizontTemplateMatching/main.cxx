#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;
using namespace std;

int stableFrameCounter = 0;
Point stablePoint;


// START: Template matching
/// Global Variables
Mat img; Mat templ; Mat result;
char* image_window = "Source Image";
char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;

Mat input;
Mat object;

/// Function Headers
void MatchingMethod( int, void* );
// END: Template Matching

// The rectangular with the detection
Rect change;

// create a mask for the table top
Mat createReferenceMask(Mat &reference, int threshold);

// detect an object on the table top
Rect detectChange(Mat &img, Mat &reference, Mat &mask, int diffThreshold);

// check if object moves or not
bool isStable(Point &center, int stableFrames, float maxDistance);

// detection
int detection( void );

// Image input
int input_cols = 640, input_rows = 480;


// The features
// Get the horizont of the image
float horizont = input_rows / 2;
// set the hight and width of the features
float feature_hight = 6;
float feature_width = 40;
// Horizont
Rect rHorizont(0, horizont - feature_hight / 2, input_cols-1, feature_hight);
// Get the location of the features
Rect rFeature_left, rFeature_right;
Mat mFeature_right, mFeature_left;

//
// these functions are just for testing
//
void processImage(Mat &input, Mat &reference, Mat &mask, int diffThreshold,
		float minWidthRatio, int stabelFrames);
int testImages(int regionGrowingThreshold, int colorDifferenceThreshold,
		float minWidthThreshold);
int testStream(int regionGrowingThreshold, int colorDifferenceThreshold,
		float minWidthThreshold, int stabelFrames);

int main(int argc, char **argv) {

	// PARAMETERS
	// the color value difference from the seed point
	int regionGrowingThreshold = 50;
	// the color value difference for considering a pixel as foreground
	int differenceImageThreshold = 30;
	// the minimum size of a found object relative to the image width
	float minWidthThreshold = 0.05;
	// number of frames an object must remain at the same location
	int stableFrames = 10;

	// Preview Windows
	namedWindow("reference mask", WINDOW_AUTOSIZE);
	namedWindow("result", WINDOW_AUTOSIZE);

	// load images from disc for testing
	// return testImages(regionGrowingThreshold, differenceImageThreshold,
	//		minWidthThreshold);

	// use camera stream for testing
	 return testStream(regionGrowingThreshold, differenceImageThreshold,
			minWidthThreshold, stableFrames);
}

/** @function main */
int detection( void )
{

  Mat img_object = object;
  Mat img_scene = input;
  
  namedWindow("input", WINDOW_AUTOSIZE);
	namedWindow("object", WINDOW_AUTOSIZE);
	
	imshow("input", input);
	imshow("object", object);
	// return 1;
  

  if( !img_object.data || !img_scene.data )
  { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 1000;
  SurfFeatureDetector detector( minHessian );
  //cv::FeatureDetector * detector = new cv::SIFT();

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );
  //delete(detector);

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;
  //cv::DescriptorExtractor * extractor = new cv::SIFT();

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );
  //delete(extractor);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );

  waitKey(0);
  return 0;
  }

int testStream(int regionGrowingThreshold, int colorDifferenceThreshold,
		float minWidthThreshold, int stableFrames) {

	// initialize capture
	VideoCapture cap(0);
	if (!cap.isOpened())
		return -1;

	cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);

	// get reference image
	cap >> input;
	Mat reference;
	input.copyTo(reference);
	
	Mat referenceMask = createReferenceMask(reference, regionGrowingThreshold);
	imshow("reference mask", referenceMask);

	while (true) {

		// get image to test
		cap >> input;
		
		int key = waitKey(30);

		if (key == 27) { // ESC
		  detection( );
	  }
		if(key == 113) { // pressed Q
		  cout << "Heading" << endl;
		  cout << "Width" << input.cols << endl;
		  cout << "Hight" << input.rows << endl;

          // Get a view of the horizont
          Mat mHorizont = input(rHorizont);
          
          // Get image and template
          img = mHorizont;
          templ = mFeature_left;
          
          // Correlate the features
          /// Create windows
		  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
		  namedWindow( result_window, CV_WINDOW_AUTOSIZE );

		  /// Create Trackbar
		  char* trackbar_label = "Method: \n 0: SQDIFF \n 1: SQDIFF NORMED \n 2: TM CCORR \n 3: TM CCORR NORMED \n 4: TM COEFF \n 5: TM COEFF NORMED";
		  createTrackbar( trackbar_label, image_window, &match_method, max_Trackbar, MatchingMethod );

		  MatchingMethod( 0, 0 );
          
		}
		if(key == 119) { // pressed W
		  cout << "Create new refernce mask" << endl;
		  input.copyTo(reference);
		  Mat referenceMask = createReferenceMask(reference, regionGrowingThreshold);
		  imshow("reference mask", referenceMask);
        }
		if(key == 101) {// pressed E
		  processImage(input, reference, referenceMask, colorDifferenceThreshold,
				minWidthThreshold, stableFrames);
		  input(change).copyTo(object);
			}
			
		}

	return 0;
}

/**
 * @function MatchingMethod
 * @brief Trackbar callback
 */
void MatchingMethod( int, void* )
{
  /// Source image to display
  Mat img_display;
  img.copyTo( img_display );

  /// Create the result matrix
  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_cols, result_rows, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( img, templ, result, match_method );
  normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  /// Localizing the best match with minMaxLoc
  double minVal; double maxVal; Point minLoc; Point maxLoc;
  Point matchLoc;

  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
  if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
  else
    { matchLoc = maxLoc; }

  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

  imshow( image_window, img_display );
  imshow( result_window, result );

  return;
}

void processImage(Mat &input, Mat &reference, Mat &mask, int diffThreshold,
		float minWidthRatio, int stabelFrames) {

	// detect largest object on table
	change = detectChange(input, reference, mask, diffThreshold);
	Point center = Point((change.x + change.width) / 2,
			(change.y + change.height) / 2);

	// check if object large enough and stable
	if (change.width >= reference.cols * minWidthRatio
			&& isStable(center, stabelFrames, input.cols * 0.01)) {

		cout << "Found object at " << center.x << "x" << center.y << endl;
		cout << "Top left: " << change.tl() << " and bottom right " << change.br() << endl;
		rectangle(input, change, Scalar(0, 255, 0), 2);

	} else {
		rectangle(input, change, Scalar(0, 0, 255), 2);
		cout << "No object found" << endl;
	}

	imshow("result", input);

}

bool isStable(Point &center, int stableFrames, float maxDistance) {
	float dx = center.x - stablePoint.x;
	float dy = center.y - stablePoint.y;
	float dist = sqrt(dx * dx + dy * dy);
	if (dist <= maxDistance) {
		stableFrameCounter++;
	} else {
		stableFrameCounter = 0;
	}
	stablePoint = center;
	return stableFrameCounter >= stableFrames;
}

Rect detectChange(Mat &img, Mat &reference, Mat &mask, int diffThreshold) {

	Mat diff(reference.rows, reference.cols, reference.type());

	// calculate difference image
	absdiff(reference, img, diff);

	// convert  difference image to gray scale
	cvtColor(diff, diff, CV_RGB2GRAY);

	// apply gray value threshold
	// all pixels with a higher difference than diffThreshold will be white,
	// all others black
	threshold(diff, diff, diffThreshold, 255, THRESH_BINARY);

	// apply the reference mask (only pixels on table remain)
	bitwise_and(diff, mask, diff);

	// find connected regions
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(diff, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// find largest region
	int max = 0;
	vector<Point> largestContour;
	for (int i = 0; i < contours.size(); i++) {
		if (max < contours[i].size()) {
			max = contours[i].size();
			largestContour = contours[i];
		}
	}

	// create bounding box from contour
	Rect rect;
	if (largestContour.size() > 0) {
		rect = boundingRect(largestContour);
	}

	return rect;
}

Mat createReferenceMask(Mat &reference, int threshold) {

	// smooth the image (mask relative to image size)
	int medianSize = int((float) reference.cols * 0.006);
	if (medianSize == 0 || medianSize % 2 != 1)
		medianSize++;
	medianBlur(reference, reference, medianSize);

	// create mask (one pixel larger than reference on every border)
	Mat mask(reference.rows + 2, reference.cols + 2, CV_8UC1, Scalar(0));

	// define the seed point (center of bottom image border)
	Point seedPoint = Point(reference.cols / 2, reference.rows - 2);

	// do floodfill
	uchar fillValue = 255;
	floodFill(reference, mask, seedPoint, Scalar(255), 0,
			Scalar(threshold, threshold, threshold),
			Scalar(threshold, threshold, threshold),
			4 | FLOODFILL_MASK_ONLY | (fillValue << 8) | FLOODFILL_FIXED_RANGE);

	// fill holes
	int erosion_size = 4;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));
	dilate(mask, mask, element);
	erode(mask, mask, element);

	// crop the additional border pixels
	Rect cropRect(1, 1, reference.cols, reference.rows);
	mask = mask(cropRect);

	return mask;
}

