namespace templateMatching {
// Parameter of the feature
int horizontFeatureHight = 6;
int horizontFeatureWidth = 40;
Rect rFeature_left, rFeature_right;
Mat mFeature_right, mFeature_left;
  
// Prototypes
void templateMatching( int match_method, cv::Mat &mTemplate, cv::Mat &mImage  );
void horizontTemplateMatching( int match_method, cv::Mat &mImage  );
void horizontTemplateCreating( int x, float templateHight, float templateWidth, cv::Mat &mImage  );

void horizontTemplateCreating( int x, float templateHight, float templateWidth, cv::Mat &mImage  )
{
  horizontFeatureHight = (int)templateHight;
  horizontFeatureWidth = (int)templateWidth;
  
  // The feature setup for template detection
  // Get the horizont of the image
  int yHorizont = (int)(mImage.rows / 2.0f);
  
  // Get rectangle of template
  int halfTemplateWidth = (int)(horizontFeatureWidth/2.0f);
  int halfTemplateHight = (int)(horizontFeatureHight/2.0f);
  
  rFeature_left = cv::Rect(x - halfTemplateWidth, yHorizont - halfTemplateHight, x - halfTemplateWidth, horizontFeatureHight);
  
  // Get the template
  mImage(rFeature_left).copyTo(mFeature_left);
  
}

void horizontTemplateMatching( int match_method, cv::Mat &mImage  )
{
  // The feature setup for template detection
  // Get the horizont of the image
  int yHorizont = (int)(mImage.rows / 2.0f);
  
  // Get rectangle of template
  int halfTemplateWidth = (int)(horizontFeatureWidth/2.0f);
  int halfTemplateHight = (int)(horizontFeatureHight/2.0f);
  
  // Horizont
  Rect rHorizont(0, yHorizont - halfTemplateHight, mImage.cols-1, horizontFeatureHight);
  
  // Past the view of the horizont
  cv::Mat mHorizontImage = mImage(rHorizont);
  templateMatching( /*Method*/ match_method, mFeature_left, mHorizontImage ); 
}
  
void templateMatching( int match_method, cv::Mat &mTemplate, cv::Mat &mImage  )
{
  /// Create the result matrix
  int result_cols =  mImage.cols - mTemplate.cols + 1;
  int result_rows = mImage.rows - mTemplate.rows + 1;

  cv::Mat result( result_cols, result_rows, CV_32FC1 );

  /// Do the Matching and Normalize
  matchTemplate( mImage, mTemplate, result, match_method );
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

  
#ifndef NO_SHOW_  
  /// Source image to display
  Mat img_display;
  mImage.copyTo( img_display );
  
  /// Show me what you got
  rectangle( img_display, matchLoc, Point( matchLoc.x + mTemplate.cols , matchLoc.y + mTemplate.rows ), Scalar::all(0), 2, 8, 0 );
  rectangle( result, matchLoc, Point( matchLoc.x + mTemplate.cols , matchLoc.y + mTemplate.rows ), Scalar::all(0), 2, 8, 0 );

  // Show the result
  char* image_window = "Source Image";
  char* result_window = "Result window";
  namedWindow( image_window, CV_WINDOW_AUTOSIZE );
  namedWindow( result_window, CV_WINDOW_AUTOSIZE );
  imshow( image_window, img_display );
  imshow( result_window, result );
#endif

  return;
}
}