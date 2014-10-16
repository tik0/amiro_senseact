//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : tabletop object recognition
//============================================================================

#include <cv.h>
#include <highgui.h>
#include <iostream>

using namespace std;
using namespace cv;

int stableFrameCounter = 0;
Point stablePoint;

const string WIN_NAME = "Camera View";
const int DELAY = 30;
const int KEY_ESC = 27;

int main() {

  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video0
  cam.open(0);
  // Allocate a frame object to store the picture
  cv::Mat frame;

  // Read the key
  int key = cv::waitKey(DELAY);

  // Process the cam as long as it is open
  for (; cam.isOpened();) {

    // Save the actual picture to the frame object
    cam >> frame;

    // Show the picture
    cv::imshow(WIN_NAME, frame);

    // Read a keystroke
    key = cv::waitKey(DELAY);

    // If the keystroke was ESC, then break
    if (KEY_ESC == key)
      break;
    if(key == 113) // pressed Q
      break;
    if(key == 119) // pressed W
      break;
    if(key == 101) // pressed E
      break;
  }

  // Free the cam
  cam.release();

  return 0;
}




// create a mask for the table top
Mat createReferenceMask(Mat &reference, int threshold);

// detect an object on the table top
Rect detectChange(Mat &img, Mat &reference, Mat &mask, int diffThreshold);

// check if object moves or not
bool isStable(Point &center, int stableFrames, float maxDistance);

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
	return testImages(regionGrowingThreshold, differenceImageThreshold,
			minWidthThreshold);

	// use camera stream for testing
	// return testStream(regionGrowingThreshold, differenceImageThreshold,
	//		minWidthThreshold, stableFrames);
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
	Mat input;
	cap >> input;
	Mat reference;
	input.copyTo(reference);

	Mat referenceMask = createReferenceMask(reference, regionGrowingThreshold);
	imshow("reference mask", referenceMask);

	while (true) {

		// get image to test
		cap >> input;

		processImage(input, reference, referenceMask, colorDifferenceThreshold,
				minWidthThreshold, stableFrames);

		// stop on button press
		if (waitKey(30) >= 0)
			break;
	}
	return 0;
}

int testImages(int regionGrowingThreshold, int colorDifferenceThreshold,
		float minWidthThreshold) {

	// read images from disc
	Mat reference = imread("./reference.jpg", CV_LOAD_IMAGE_COLOR);
	Mat ignore0 = imread("./ignore0.jpg", CV_LOAD_IMAGE_COLOR);
	Mat detect0 = imread("./detect0.jpg", CV_LOAD_IMAGE_COLOR);
	Mat detect1 = imread("./detect1.jpg", CV_LOAD_IMAGE_COLOR);
	Mat detect2 = imread("./detect2.jpg", CV_LOAD_IMAGE_COLOR);

	if (reference.cols == 0 || ignore0.cols == 0 || detect0.cols == 0
			|| detect1.cols == 0 || detect2.cols == 0) {
		cerr << "Error: Not able to load images!" << endl;
		return 1;
	}

	Mat referenceMask = createReferenceMask(reference, regionGrowingThreshold);
	imshow("reference mask", referenceMask);

	processImage(detect0, reference, referenceMask, colorDifferenceThreshold,
			minWidthThreshold, 0);

	// wait for button press before continuing
	waitKey(0);

	processImage(detect1, reference, referenceMask, colorDifferenceThreshold,
			minWidthThreshold, 0);

	// wait for button press before continuing
	waitKey(0);

	processImage(detect2, reference, referenceMask, colorDifferenceThreshold,
			minWidthThreshold, 0);

	// wait for button press before continuing
	waitKey(0);

	processImage(ignore0, reference, referenceMask, colorDifferenceThreshold,
			minWidthThreshold, 0);

	// wait for button press before continuing
	waitKey(0);

	return 0;
}

void processImage(Mat &input, Mat &reference, Mat &mask, int diffThreshold,
		float minWidthRatio, int stabelFrames) {

	// detect largest object on table
	Rect change = detectChange(input, reference, mask, diffThreshold);
	Point center = Point((change.x + change.width) / 2,
			(change.y + change.height) / 2);

	// check if object large enough and stable
	if (change.width >= reference.cols * minWidthRatio
			&& isStable(center, stabelFrames, input.cols * 0.01)) {

		cout << "Found object at " << center.x << "x" << center.y << endl;
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

