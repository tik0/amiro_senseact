#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

// For checking character pressed in the console
#include <kbhit.hpp>


// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_COMPARE = "COMP";
std::string COMMAND_ENDOFCOMPARE = "END_COMP";

#define maxObjects 10
#define maxObjectSites 4


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
using namespace rsb::converter;
using namespace cv;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

static std::string g_sOutScope = "/image";
static std::string g_sInScope = "/command";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;


std::string directory = "objectPics/";
std::string savingDir = "object_";
std::string fileend = ".jpg";

// init values for saving and comparing
int imageId = 0;
int compareId = 0;
int siteId = 0;
Mat objects[maxObjects][maxObjectSites][4];
int siteCount[maxObjectSites];
int orderedObjects[maxObjects][maxObjects];
float weightedObjects[maxObjects][maxObjects];
bool sendingPic = false;
bool saving = true;
bool newObject = true;

Mat frame, b_hist, g_hist, r_hist, gray_hist;



// calculates histograms
void calcHistograms() {
  cv::Mat gray_image;
  cv::cvtColor(frame, gray_image, CV_BGR2GRAY);

  /// Separate the image in 3 places ( B, G and R )
  vector<Mat> bgr_planes;
  split(frame, bgr_planes);

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 255 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  //Mat b_hist, g_hist, r_hist, gray_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &gray_image, 1, 0, Mat(), gray_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
  Mat histImageGray( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
  normalize(gray_hist, gray_hist, 0, histImageGray.rows, NORM_MINMAX, -1, Mat() );
}



// saving objects procedure
void savingObject(Mat blue, Mat green, Mat red, Mat gray, bool saveImg) {
  // Save histogramms
  Mat blue_s, green_s, red_s, gray_s;
  blue.copyTo(blue_s);
  green.copyTo(green_s);
  red.copyTo(red_s);
  gray.copyTo(gray_s);
  objects[imageId][siteId][0] = blue_s;
  objects[imageId][siteId][1] = green_s;
  objects[imageId][siteId][2] = red_s;
  objects[imageId][siteId][3] = gray_s;
  siteCount[imageId] = siteId+1;
}


// load objects
void loadObjects() {
  for (imageId=0; imageId<maxObjects; imageId++) {
    for (siteId=0; siteId<maxObjectSites; siteId++) {
      string filename = directory + savingDir + to_string(imageId) + "_" + to_string(siteId) + fileend;
      frame = imread(filename, 1);
      if (frame.rows > 0 && frame.cols > 0) {
        calcHistograms();
        savingObject(b_hist, g_hist, r_hist, gray_hist, false);
      } else {
        //printf("Couldn't load at object %i site %i.\n", imageId+1, siteId+1);
        break;
      }
    }
    if (siteId == 0) {
      break;
    } else {
      siteCount[imageId] = siteId;
    }
  }
  if (imageId > 0) {
    printf("%i objects loaded:\n", imageId);
    for (int i=0; i<imageId; i++) {
      printf(" - %i: %i sites\n", i+1, siteCount[i]);
    }
  } else {
    std::cout << "No objects could be loaded in relative directory '" << directory << "' with filename type '" << savingDir << "x_x" << fileend << "'!\n";
  }
}


// comparing saved objects to one snapshot
void comparing(Mat blue, Mat green, Mat red, Mat gray) {
  Mat nullMat = Mat::eye(1,1,CV_64F);
  for (int obj=0; obj < imageId; obj++) {
    float compShot = 0;
    for (int site=0; site < siteCount[obj]; site++) {
      float comp_b = (compareHist(blue, objects[obj][site][0], 0)+1)/2.0;
      float comp_g = (compareHist(green, objects[obj][site][1], 0)+1)/2.0;
      float comp_r = (compareHist(red, objects[obj][site][2], 0)+1)/2.0;
      float comp_gray = (compareHist(gray, objects[obj][site][3], 0)+1)/2.0;
      float comp_color = comp_b*comp_g*comp_r;
      float comp_x = comp_color * comp_gray;
      printf("Comparison with %i. object site %i: b=%f g=%f r=%f color=%f gray=%f x=%f\n", obj+1, site+1, comp_b, comp_g, comp_r, comp_color, comp_gray, comp_x);
      if (comp_x > compShot) {
        compShot = comp_x;
      }
    }

    orderedObjects[compareId][obj] = obj;
    weightedObjects[compareId][obj] = compShot;
  }

  for (int run=1; run < imageId; run++) {
    for (int comp=imageId-1; comp >= run; comp--) {
      float compWeight = weightedObjects[compareId][comp];
      int compId = orderedObjects[compareId][comp];
      if (weightedObjects[compareId][comp-1] < compWeight) {
        weightedObjects[compareId][comp] = weightedObjects[compareId][comp-1];
        orderedObjects[compareId][comp] = orderedObjects[compareId][comp-1];
        weightedObjects[compareId][comp-1] = compWeight;
        orderedObjects[compareId][comp-1] = compId;
      }
    }
  }

  printf("Order for object %i: ", compareId+1);
  for (int idx=0; idx<imageId; idx++) {
    if (idx < imageId-1) {
      printf("%i (%f), ", orderedObjects[compareId][idx]+1, weightedObjects[compareId][idx]);
    } else {
      printf("%i (%f)", orderedObjects[compareId][idx]+1, weightedObjects[compareId][idx]);
    }
  }
  printf("\n");
}


// sort compared objects
void sortObjects() {
  int order[imageId];
  bool ordered[imageId];
  bool allOrdered = false;
  for (int i=0; i<imageId; i++) {
    order[i] = -1;
    ordered[i] = false;
  }
  int levelChoose = 0;

    printf("First sort:\n");
  
    for (int obj=0; obj < imageId; obj++) {
      int shootId = -1;
      for (int shoot=0; shoot < compareId; shoot++) {
        if (orderedObjects[shoot][levelChoose] == obj && shootId < 0) {
          shootId = shoot;
        } else if (orderedObjects[shoot][levelChoose] == obj) {
          shootId = -1;
          break;
        }
      }
      if (shootId >= 0) {
        order[shootId] = obj;
        ordered[obj] = true;
        printf(" - Object %i belongs to pic %i.\n", obj+1, shootId+1);
      } else {
        printf(" - No clear selection for object %i.\n", obj+1);
      }
    }

    int orderedCount = 0;
    for (int i=0; i<imageId; i++) {
      if (ordered[i]) {
        orderedCount++;
      }
    }
    allOrdered = orderedCount == compareId;

  while (!allOrdered && levelChoose != imageId) {

    printf("Sort at level %i:\n", levelChoose+1);
  
    for (int obj=0; obj < imageId; obj++) {
      if (!ordered[obj]) {
        int shootIds[imageId];
        int count = 0;
        for (int shoot=0; shoot < compareId; shoot++) {
          if (orderedObjects[shoot][levelChoose] == obj && order[shoot] < 0) {
            shootIds[count] = shoot;
            count++;
          }
        }
        if (count == 1) {
          order[shootIds[0]] = obj;
          ordered[obj] = true;
          printf(" - Object %i belongs to pic %i.\n", obj+1, shootIds[0]+1);
        } else if (count > 1) {
          printf(" - Object %i found in pics ", obj+1);
          int idx = -1;
          float dist = 0;
          for (int shoot=0; shoot<count; shoot++) {
            printf("%i ", shootIds[shoot]+1);
            for (int comp=levelChoose+1; comp<imageId; comp++) {
              if (!ordered[orderedObjects[shootIds[shoot]][comp]]) {
                float newDist = weightedObjects[shootIds[shoot]][levelChoose]/weightedObjects[shootIds[shoot]][comp];
                if (shoot < count-1) {
                  printf("(%i, %f/%f=%f), ", weightedObjects[shootIds[shoot]][levelChoose], weightedObjects[shootIds[shoot]][comp], orderedObjects[shootIds[shoot]][comp]+1, newDist);
                } else {
                  printf("(%i, %f/%f=%f)", weightedObjects[shootIds[shoot]][levelChoose], weightedObjects[shootIds[shoot]][comp], orderedObjects[shootIds[shoot]][comp]+1, newDist);
                }
            
                if (newDist > dist) {
                  idx = shootIds[shoot];
                  dist = newDist;
                }
                break;
              }
            }
          }
          order[idx] = obj;
          ordered[obj] = true;
          printf(" - selected for pic %i (%f).\n", idx+1, dist); 
        } else {
          printf(" - Object %i not found.\n", obj+1);
        }
      }
    }

    levelChoose++;

    int orderedCount = 0;
    for (int i=0; i<compareId; i++) {
      if (ordered[i]) {
        orderedCount++;
      }
    }
    allOrdered = orderedCount == compareId;

  }

  printf("Final order: ");
  for (int i=0; i<compareId; i++) {
    printf("%i ", order[i]+1);
  }
  printf("\n");
}



// main function
int main(int argc, char **argv) {  
  
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving commands")
            ("loadingDirectory,l", po::value < std::string > (&directory),"Directory from where the data can be loaded.")
            ("device,d", po::value < int > (&g_iDevice),"Number of device")
            ("quality,q", po::value < unsigned int > (&g_uiQuality),"Quality of JPEG compression [0 .. 100]")
            ("sending,s", "Sends the taken snapshot over RSB.")
            ("printPic", "Prints a notice if a new picture has been taken.");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

  // first, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }

  sendingPic = vm.count("sending");
    
  // afterwards, let program options handle argument errors
  po::notify(vm);
    
  INFO_MSG( "Scope: " << g_sOutScope)
  INFO_MSG( "Commands: " << g_sInScope)
  INFO_MSG( "Device: " << g_iDevice)
  INFO_MSG( "JPEG Quality: " << g_uiQuality)

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create the informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

  // Create and start the command listener
  rsb::ListenerPtr listener = factory.createListener(g_sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(commandQueue)));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video<g_iDevice>
  if ( cam.open(g_iDevice) ) {
    // Buffer to store the compressed image

    loadObjects();
    if (imageId <= 0) {
      return -1;
    }
    
    // Process the cam forever
    for (; ;) {

      // Save the actual picture to the frame object
      cam >> frame;
      if (vm.count("printPic")) {
        printf("New image taken.\n");
      }
      
      if (sendingPic) {
        // Compress image
        vector<uchar> buf;
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(g_uiQuality);
      
        imencode(".jpg", frame, buf, compression_params);

        // Send the data.
        shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
        informer->publish(frameJpg);
      }

      calcHistograms();

      // Get command
      if (!commandQueue->empty()) {
        std::string command = *commandQueue->pop().get();
        // check for quit command
        if (command == COMMAND_QUIT) {
          INFO_MSG("Quit application.");
          break;
        // check for compare command
        } else if (command == COMMAND_COMPARE) {
          if (!newObject) {
            printf("Please create a new object first to finalize learning.\n");
          } else if (saving && imageId > 0) {
            saving = false;
            printf("Try to compare.\n");
          } else if (imageId > 0) {
            printf("Compare %i. object of %i:\n", compareId+1, imageId);
            comparing(b_hist, g_hist, r_hist, gray_hist);
            compareId++;
            if (compareId == imageId) {
              sortObjects();
              compareId = 0;
              saving = true;
              printf("Comparing finished.\n");
            }
          } else {
            printf("No objects saved for comparison.\n");
          }
        // check for compare end command
        } else if (command == COMMAND_ENDOFCOMPARE) {
          if (compareId > 0) {
            sortObjects();
            compareId = 0;
            saving = true;
            printf("Comparing finished.\n");
          }
        // otherwise it is an unknown command
        } else {
          INFO_MSG("Unknown command.");
        }
      }

    }
  }

  // Free the cam
  cam.release();

  return 0;

}
