namespace objectDetection {
int detection( cv::Mat &img_object, cv::Mat &img_scene , cv::Point2f &mean, cv::Point2f &var)
{
  
#ifndef NO_SHOW_
  namedWindow("input", WINDOW_AUTOSIZE);
  namedWindow("object", WINDOW_AUTOSIZE);
  
  imshow("input", img_scene);
  imshow("object", img_object);
  // return 1;
#endif
  

  if( !img_object.data || !img_scene.data )
  { ERROR_MSG( " --(!) Error reading images " ) return -1; }

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

  INFO_MSG( "-- Max dist : " << max_dist )
  INFO_MSG( "-- Min dist : " << min_dist )

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

//   INFO_MSG( "--------- descriptors : " << descriptors_object.rows )
//   if (descriptors_object.rows < 4)
//     return 1;
  
  
  for( int i = 0; i < descriptors_object.rows; i++ )
  { 
//     INFO_MSG( "feature" <<  i )
    if( matches[i].distance < 3*min_dist )
     { good_matches.push_back( matches[i]); }
//      good_matches.push_back( matches[i]);
  }
//   INFO_MSG( "Got all good matches" )
  

#ifndef NO_SHOW_  
  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
#endif
  
  //-- Localize the object
//   INFO_MSG( "Allocate points" )
  
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

//   INFO_MSG( "Get the keypoints from the good matches")
  
  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  // Calculate the mean
//   Point2f mean(0.f, 0.f);
  for (int idx = 0; idx < scene.size(); ++idx) {
    mean.x = mean.x + scene.at(idx).x;
    mean.y = mean.y + scene.at(idx).y;
  }
  mean.x = mean.x / scene.size();
  mean.y = mean.y / scene.size();
  cv::circle(img_matches, mean + Point2f( img_object.cols, 0), /*diameter*/10 , cv::Scalar(0, 255, 0), /*linewidth*/ 5);
  
  // Calculate the variance
//   Point2f var(0.f, 0.f);
  for (int idx = 0; idx < scene.size(); ++idx) {
    var.x = var.x + pow(scene.at(idx).x -mean.x, 2);
    var.y = var.y + pow(scene.at(idx).y -mean.y, 2);
  }
  var.x = var.x / scene.size();
  var.y = var.y / scene.size();
  line( img_matches, mean + Point2f( -sqrt(var.x), 0) + Point2f( img_object.cols, 0),
	             mean + Point2f( sqrt(var.x), 0) + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, mean + Point2f( 0, -sqrt(var.y)) + Point2f( img_object.cols, 0),
	             mean + Point2f( 0, sqrt(var.y)) + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  
  
  
//   imshow( "Good Matches & Object detection", img_matches );
  // Dont find the the homography
  return 0;
  
  INFO_MSG( "find homography: START")
  Mat H = findHomography( obj, scene, CV_RANSAC );
  INFO_MSG( "find homography: END")

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  INFO_MSG( "perspective Transform: START" )
  perspectiveTransform( obj_corners, scene_corners, H);
  INFO_MSG( "perspective Transform: END" )

#ifndef NO_SHOW_  
  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

  //-- Show detected matches

  imshow( "Good Matches & Object detection", img_matches );
#endif
//   waitKey(0);
  return 0;
}
}