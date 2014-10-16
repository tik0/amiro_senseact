namespace tableTopDetection {

// Prototypes
bool isStable(Point &center, int stableFrames, float maxDistance);
cv::Rect detectChange(Mat &img, Mat &reference, Mat &mask, int diffThreshold);
cv::Mat createReferenceMask(Mat &reference, int threshold);
  
// Variables for checking if the object is stable over a few frames
Point stablePoint;
bool bIsStable = false;
int stableFrameCounter = 0;

void processImage(Mat &input, Mat &reference, Mat &mask, int diffThreshold,
		float minWidthRatio, int stabelFrames, cv::Rect &rectObject) {

	DEBUG_MSG( "Process Image" )
	// detect largest object on table
	rectObject = detectChange(input, reference, mask, diffThreshold);
	Point center = Point((rectObject.x + rectObject.width) / 2,
			(rectObject.y + rectObject.height) / 2);

#ifndef NO_SHOW_
	cv::Mat inputTmp = input.clone();
#endif
		
	// check if object large enough and stable
	if (rectObject.width >= reference.cols * minWidthRatio
			&& isStable(center, stabelFrames, input.cols * 0.01)) {

		DEBUG_MSG( "Found object at " << center.x << "x" << center.y )
		DEBUG_MSG( "Top left: " << rectObject.tl() << " and bottom right " << rectObject.br() )
#ifndef NO_SHOW_
		rectangle(inputTmp, rectObject, Scalar(0, 255, 0), 2);
#endif
		bIsStable = true;

	} else {
#ifndef NO_SHOW_
		rectangle(inputTmp, rectObject, Scalar(0, 0, 255), 2);
#endif
		DEBUG_MSG( "No object found" )
		bIsStable = false;
	}
#ifndef NO_SHOW_
	imshow("result", inputTmp);
#endif

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

cv::Rect detectChange(Mat &img, Mat &reference, Mat &mask, int diffThreshold) {

	cv::Mat diff(reference.rows, reference.cols, reference.type());

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
	
	cv::GaussianBlur(diff, diff, cv::Size(21, 21), 0,0);
	#ifndef NO_SHOW_
	imshow("diff", diff);
	#endif

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

  
	// Just a full white image
// 	Mat maskFullWhite(reference.rows, reference.cols, CV_8UC1, Scalar(255));
// 	return maskFullWhite;
  
	// Just return a two/third white image
	Mat maskTwoThirdWhite(reference.rows, reference.cols, CV_8UC1, Scalar(0));
	cv::rectangle(maskTwoThirdWhite, cv::Point(0,reference.rows / 2), cv::Point(reference.cols,reference.rows), cv::Scalar(255), reference.rows / 2);
	return maskTwoThirdWhite;	
  
  
	// Just return a half white image
	Point Seed = Point(reference.cols / 2 , reference.rows - 2 + reference.cols / 2);
	Mat maskHalfWhite(reference.rows, reference.cols, CV_8UC1, Scalar(0));
	cv::circle(maskHalfWhite, Seed, reference.rows , cv::Scalar(255), reference.rows / 2);
	return maskHalfWhite;
	
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
}