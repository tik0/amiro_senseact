//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the proximity data directly from the device,
//               computes the occupancy grid map and send it over RSB to 
//               a foreign socket server
//============================================================================


 #define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <math.h>
#define PI 3.14159265


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <rst/navigation/OccupancyGrid2DInt.pb.h>
using namespace rst::navigation;
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

// Include own converter
#include <converter/vecIntConverter/main.hpp>
      
using namespace std;
using namespace muroxConverter;


int g_uiMapLocalHeight = 150;
int g_uiMapLocalWidth = 150;
float g_fMapLocalResolution_m = 0.0033214;
const float BeBotWidth_m = 0.10;
const float BeBotHeight_m = 0.10;
const int BeBotWidth = BeBotWidth_m / g_fMapLocalResolution_m; // Width in the local map
const int BeBotHeight = BeBotHeight_m / g_fMapLocalResolution_m;  // Height in the local map
const int BeBotNumSensor = 12;
const int AMiRoNumSensor = 8;

// The Origin of the the BeBot, which is in the middle of the BeBot,
// and in the middle of the local map
const cv::Point BeBotOrigin( g_uiMapLocalWidth/2, g_uiMapLocalHeight/2);

// If the sensor detects an obstacle under this range, the rest of the polygon will be
// drawn as occupied
float g_occStart_m = 0.10;
int g_occStart = g_occStart_m / g_fMapLocalResolution_m;


struct sensor {
    int   id;
    int    x;
    int    y;
    int    orientation;
    int    value;
  };
struct sensor BeBot[BeBotNumSensor];

// Prototypes
void sendLocalMap(rsb::EventPtr event, rsb::Informer< OccupancyGrid2DInt >::Ptr informerOccMap);
void createMap(boost::shared_ptr<OccupancyGrid2DInt> &map);
void drawSensorInformation(boost::shared_ptr<std::vector<int> > &messageIr, boost::shared_ptr<OccupancyGrid2DInt> &map);
int BeBotIrValue2localMap(int value);
void initBeBotSensorCoordinates();

std::string g_sOutScope_OGM = "/maps/ogm";
std::string g_sInScope_Prox = "/IR";
std::string g_sRemoteSocketServer = "192.168.0.200";
std::string g_sRemoteSocketPort = "55555";
// int g_iObstacleStart_cm = 10;
// int g_iMapResolution_um = 3321;


int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&g_sOutScope_OGM), "Scope for sending the grid maps.")
    ("inscope,i", po::value < std::string > (&g_sInScope_Prox), "Scope for receiving the proximity values.")
    ("remoteSocketServer,s", po::value < std::string > (&g_sRemoteSocketServer), "IP of remote socket server.")
    ("remoteSocketPort,p", po::value < std::string > (&g_sRemoteSocketPort), "Port of remote socket server.")
    ("obstacleStart", po::value < float > (&g_occStart_m), "Distance in cm, when an obstacle starts.")
    ("mapResolution", po::value < float > (&g_fMapLocalResolution_m), "Resolution of the map.")
    ("mapHeight", po::value < int > (&g_uiMapLocalHeight), "Height of the map.")
    ("mapWidth", po::value < int > (&g_uiMapLocalWidth), "Width of the map.");

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
  
  // afterwards, let program options handle argument errors
  po::notify(vm);


  INFO_MSG( "Outscope: " << g_sOutScope_OGM)
  INFO_MSG( "Inscope: " << g_sInScope_Prox)
  INFO_MSG( "Remote Server: " << g_sRemoteSocketServer)
  INFO_MSG( "Remote Port: " << g_sRemoteSocketPort)
  INFO_MSG( "Obstacle start: " << g_occStart_m)
  INFO_MSG( "mapResolution: " << g_fMapLocalResolution_m)
  INFO_MSG( "mapHeight: " << g_uiMapLocalHeight)
  INFO_MSG( "mapWidth: " << g_uiMapLocalWidth)

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
  ///////////////////////////////////////////////////////////////////////////////
      // Get the global participant config as a template
      rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
            {
              // Get the options for socket transport, because we want to change them
              rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();

              // enable socket transport
              std::string enabled = "1";
              tmpPropSocket["enabled"] = boost::any(enabled);

              // this node is the server, all other nodes clients server = 0 !!!
              std::string server = "0";
              tmpPropSocket["server"] = boost::any(server);

              // Change the config
              tmpPropSocket["host"] = boost::any(g_sRemoteSocketServer);

              // Change the Port
              tmpPropSocket["port"] = boost::any(g_sRemoteSocketPort);

              // Write the socket tranport properties back to the participant config
              tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
            }
  ///////////////////////////////////////////////////////////////////////////////

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  // Register new converter for occupancy grid maps
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<OccupancyGrid2DInt> >
      converter(new rsb::converter::ProtocolBufferConverter<OccupancyGrid2DInt>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader for the IR data
  ReaderPtr reader = factory.createReader(g_sInScope_Prox);

  // Prepare RSB informer for the occupancy grid
  rsb::Informer< OccupancyGrid2DInt >::Ptr informerOccMap = factory.createInformer< OccupancyGrid2DInt > (g_sOutScope_OGM, tmpPartConf);

  // Init the BeBot Sensors
  initBeBotSensorCoordinates();

  // Receive IR values and publish the map
  while (true) {
    sendLocalMap(reader->read(), informerOccMap);
  }

  return EXIT_SUCCESS;
}

void sendLocalMap(rsb::EventPtr event, rsb::Informer< OccupancyGrid2DInt >::Ptr informerOccMap) {

      // Get the message with the IR data
      boost::shared_ptr<std::vector<int> > messageIr = static_pointer_cast<std::vector<int> >(event->getData());

      // Create a map
      boost::shared_ptr<OccupancyGrid2DInt> map = boost::shared_ptr<OccupancyGrid2DInt>(new OccupancyGrid2DInt);
      createMap(map);

      // Draw the sensor information into the map
      if (messageIr->size() == BeBotNumSensor) {
        drawSensorInformation(messageIr, map);
      } else if (messageIr->size() == AMiRoNumSensor) {
        // TODO
      }

      // Send the map
      informerOccMap->publish(map);
}

int BeBotIrValue2localMap(int value) {

  // lower bound
  if (value < 16)
    value = 16;

  // Linearisation of the sensor values
  float b = 0.008135;
  float m = 1.03254;
  float x = (1/sqrt(value) - b) / m;

  DEBUG_MSG("Distance to Object: " << x << " m")
  // Return the value in local grid coordinates
  return x / g_fMapLocalResolution_m;
}

void drawSensorInformation(boost::shared_ptr<std::vector<int> > &messageIr, boost::shared_ptr<OccupancyGrid2DInt> &map) {

  // Copy map to an image
  cv::Mat mapImage(map->height(), map->width(), CV_8UC1);
  for(int i = 0; i < map->height(); i++)
    for(int j = 0; j < map->width(); j++)
      mapImage.at<uchar>(i,j)=map->map()[j+i*map->width()];

  // transform the sensordata into local grid coordinates
  for (int sensorIdx = 0; sensorIdx < BeBotNumSensor; sensorIdx++)
    BeBot[sensorIdx].value = BeBotIrValue2localMap(messageIr->at(sensorIdx));

  // First draw the occupied polygons, then the free ones
  for (int occFreeIdx = 0; occFreeIdx <= 1; ++occFreeIdx) {
    for (int sensorIdx = 0; sensorIdx < BeBotNumSensor; sensorIdx++){

  //        int sensorScale = BeBot[sensorIdx].value;      // The real sensor input
          int sensorScale = (occFreeIdx) ? BeBot[sensorIdx].value : g_occStart;

          cv::Point sensorOrigin = cv::Point( BeBot[sensorIdx].x, BeBot[sensorIdx].y) + BeBotOrigin;  // Origin of the sensor
          float orientation_rad = BeBot[sensorIdx].orientation * PI / 180.0 ;  // orientation of the sensor
          // Create the left polygon point
          cv::Point leftPolyOrigin(sensorScale,sensorScale); // Opening angle of 45 °
          cv::Point leftPoly;
          leftPoly.x = leftPolyOrigin.x*cos(orientation_rad) - leftPolyOrigin.y*sin(orientation_rad);
          leftPoly.y = leftPolyOrigin.x*sin(orientation_rad) + leftPolyOrigin.y*cos(orientation_rad);
          leftPoly += sensorOrigin;
          // Create the right Poly
          cv::Point rightPolyOrigin(sensorScale,-sensorScale); // Opening angle of -45 °
          cv::Point rightPoly;
          rightPoly.x = rightPolyOrigin.x*cos(orientation_rad) - rightPolyOrigin.y*sin(orientation_rad);
          rightPoly.y = rightPolyOrigin.x*sin(orientation_rad) + rightPolyOrigin.y*cos(orientation_rad);
          rightPoly += sensorOrigin;

            /** Create some points */
            cv::Point rook_points[1][3];
            rook_points[0][0] = sensorOrigin; // Sensor origin
            rook_points[0][1] = leftPoly;
            rook_points[0][2] = rightPoly;

            const cv::Point* ppt[1] = { rook_points[0] };
            int npt[] = { 3 };

            // Draw the polygon of the sensor
            // Note: If the occopied part is drawn, then fill the polygon with value 100, else 0
            cv::fillPoly( mapImage,
                      ppt,
                      npt,
                      1,
                      (occFreeIdx) ? cv::Scalar( 0.0 ) : cv::Scalar( 100.0 ),
                      0 );  // Linetype


    }
  }

  // Draw the BeBot itselfe
  cv::Point rook_points[1][4];
  rook_points[0][0] = BeBotOrigin + cv::Point(- BeBotWidth / 2, BeBotWidth / 2);
  rook_points[0][1] = BeBotOrigin + cv::Point(- BeBotWidth / 2, - BeBotWidth / 2);
  rook_points[0][2] = BeBotOrigin + cv::Point( BeBotWidth / 2, - BeBotWidth / 2);
  rook_points[0][3] = BeBotOrigin + cv::Point( BeBotWidth / 2, BeBotWidth / 2);
  const cv::Point* ppt[1] = { rook_points[0] };
  int npt[] = { 4 };
  // Draw the polygon of the sensor
  cv::fillPoly( mapImage,ppt,npt,1, cv::Scalar( 0.0 ),0 );

  // Show the map
#ifndef __arm__
//   cv::flip(mapImage,mapImage,0); // Flip the image, to display the correct coordinates bottom-left
//   cv::imshow("Map",mapImage);
//   cv::waitKey(1);
#endif

  // Copy image back to the map
  for(int i = 0; i < map->height(); i++)
    for(int j = 0; j < map->width(); j++)
      map->mutable_map()->at(j+i*map->width()) = mapImage.at<uchar>(i,j);

}

void createMap(boost::shared_ptr<OccupancyGrid2DInt> &map) {

  map->set_height(g_uiMapLocalHeight);
  map->set_width(g_uiMapLocalWidth);
  map->set_resolution(g_fMapLocalResolution_m); // Resolution per pixel in meter
  // set the origin in the middle of the local map
  map->mutable_origin()->mutable_translation()->set_x(map->width() / 2.0 * map->resolution());
  map->mutable_origin()->mutable_translation()->set_y(map->height() / 2.0 * map->resolution());
  map->mutable_origin()->mutable_translation()->set_z(double(0.0));
  map->mutable_origin()->mutable_rotation()->set_qw(double(0.0));
  map->mutable_origin()->mutable_rotation()->set_qx(double(0.0));
  map->mutable_origin()->mutable_rotation()->set_qy(double(0.0));
  map->mutable_origin()->mutable_rotation()->set_qz(double(0.0));

  // Fill the map with unknown (-1) values
  int idxWidth, idxHeight;
  for ( idxWidth = 0; idxWidth < map->width(); ++idxWidth)
    for ( idxHeight = 0; idxHeight < map->height(); ++idxHeight)
      map->mutable_map()->push_back(-1);
}

void initBeBotSensorCoordinates() {

  // Sensor coordinates in local grid space
  BeBot[0].id = 0;
  BeBot[0].orientation = 90;
  BeBot[0].x = - BeBotWidth / 4;
  BeBot[0].y = BeBotHeight / 2;
  BeBot[0].value = 10;
  BeBot[1].id = 1;
  BeBot[1].orientation = 90 + 45;
  BeBot[1].x = - BeBotWidth / 2;
  BeBot[1].y = BeBotHeight / 2;
  BeBot[1].value = 10;
  BeBot[2].id = 2;
  BeBot[2].orientation = 180;
  BeBot[2].x = - BeBotWidth / 2;
  BeBot[2].y = BeBotHeight / 4;
  BeBot[2].value = 10;
  BeBot[3].id = 3;
  BeBot[3].orientation = 180;
  BeBot[3].x = - BeBotWidth / 2;
  BeBot[3].y = - BeBotHeight / 4;
  BeBot[3].value = 10;
  BeBot[4].id = 4;
  BeBot[4].orientation = 180 + 45;
  BeBot[4].x = - BeBotWidth / 2;
  BeBot[4].y = - BeBotHeight / 2;
  BeBot[4].value = 10;
  BeBot[5].id = 5;
  BeBot[5].orientation = 180 + 90;
  BeBot[5].x = - BeBotWidth / 4;
  BeBot[5].y = - BeBotHeight / 2;
  BeBot[5].value = 10;
  BeBot[6].id = 6;
  BeBot[6].orientation = 180 + 90;
  BeBot[6].x = BeBotWidth / 4;
  BeBot[6].y = - BeBotHeight / 2;
  BeBot[6].value = 10;
  BeBot[7].id = 7;
  BeBot[7].orientation = 180 + 90 + 45;
  BeBot[7].x =  BeBotWidth / 2;
  BeBot[7].y = - BeBotHeight / 2;
  BeBot[7].value = 10;
  BeBot[8].id = 8;
  BeBot[8].orientation = 0;
  BeBot[8].x = BeBotWidth / 2;
  BeBot[8].y = - BeBotHeight / 4;
  BeBot[8].value = 10;
  BeBot[9].id = 9;
  BeBot[9].orientation = 0;
  BeBot[9].x = BeBotWidth / 2;
  BeBot[9].y = BeBotHeight / 4;
  BeBot[9].value = 10;
  BeBot[10].id = 10;
  BeBot[10].orientation = 45;
  BeBot[10].x = BeBotWidth / 2;
  BeBot[10].y = BeBotHeight / 2;
  BeBot[10].value = 10;
  BeBot[11].id = 11;
  BeBot[11].orientation = 90;
  BeBot[11].x = BeBotWidth / 4;
  BeBot[11].y = BeBotHeight / 2;
  BeBot[11].value = 10;
}
