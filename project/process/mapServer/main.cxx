// Message formating
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
 #define ERROR_MSG_
#include <MSG.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
// #include <rsc/threading/SynchronizedQueue.h>
// #include <rsb/util/QueuePushHandler.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/misc/SignalWaiter.h>

// RST Proto types
#include <rst/navigation/OccupancyGrid2DInt.pb.h>
#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
#include <types/MachineModel.pb.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // resize

// Stdandard libraries
#include <mutex>          // std::mutex
#include <math.h>
#include <string>
#include <iostream>

// Boost
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// mrpt
#include <mrpt/maps/CMultiMetricMap.h>
//#include <mrpt/maps.h>
//#include <mrpt/opengl.h>
//#include <mrpt/gui.h>

// Converting helpers
#include <Eigen/Geometry>

inline Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

#define NUM_MAPS 9
enum maps {
  stockDensity1,
  stockDensity2,
  stockDensity3,
  obstacleNonProcNonAcc,
  obstacleProcNonAcc,
  obstacleNonProcAcc,
  stockFlattened,
  stockEdge,
  processed  // Overwrites all other properties for visualization and other reasoning
};

static std::string mapSubScopes[NUM_MAPS] {
  "/stockDensity1",
  "/stockDensity2",
  "/stockDensity3",
  "/obstacleNonProcNonAcc",
  "/obstacleProcNonAcc",
  "/obstacleNonProcAcc",
  "/stockFlattened",
  "/stockEdge",
  "/processed"
};

// Concatenated map scopes with the correct prefix, which is set by program options
static std::string mapScopes[NUM_MAPS];

#define NUM_BGR_CHANNELS 3
unsigned char mapColorBGR[NUM_MAPS][NUM_BGR_CHANNELS] {
  {255,   0,   0}, // Light blue
  {200,   0,   0}, // Mid blue
  {127,   0,   0}, // Dark blue
  {  0,   0, 255}, // Dark red
  { 42,  42, 165}, // Brown
  { 30, 105, 210}, // Chocolat
  {200,   0, 200}, // Purple
  {  0, 255, 255}, // Yellow
  {  0, 255,   0}  // Green
};

// The map stack
static std::vector<mrpt::maps::COccupancyGridMap2D> mapStack;
static mrpt::maps::COccupancyGridMap2D::TMapDefinition def;

// Program options
  // Properties of a single occupancy grid map
  static float resolution = 0.1f;
  static float maxOccupancyUpdateCertainty = 0.8f;
  static float maxDistanceInsertion = 30.0f;
  static float max_x = 30.0f;
  static float min_x = 0.0f;
  static float max_y = 30.0f;
  static float min_y = 0.0f;
  static bool debug = false;
  static bool doTest = false;
  static std::string ismScopePrefix = "/ism";
  static std::string machineModelScope = "/plan/Fahrzeugdaten";
  // Marging for non present information: 0.45 .. 0.55 -> 0.1
  static float uncertaintyMargin = 0.1f;

// Global variables
static rst::claas::MachineModel_Odemetrie machineModel;
static bool doMapRefresh = false;

// For sending the localization
// rsb::Informer<rst::geometry::PoseEuler>::Ptr informerOdometry;

// Actual position
// rsb::Informer<rst::geometry::PoseEuler>::DataPtr odomData(new rst::geometry::PoseEuler);

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

std::mutex mapRefresh;
std::mutex mtxOdom;       // mutex for odometry messages

// Set all map tiles of all maps to the given value
void refreshMap(float value = 0.5f) {
  for (uint idx = 0; idx < mapStack.size(); ++idx) {
    mapStack[idx].fill (value);  // Fill every map with uncertainty
  }
}

// Add just some picture of each other
boost::shared_ptr<cv::Mat> colorMapCallback() {
  const int sizeX = (def.max_x - def.min_x) / def.resolution;
  const int sizeY = (def.max_y - def.min_y) / def.resolution;

  boost::shared_ptr<cv::Mat> dst(new cv::Mat(sizeX, sizeY, CV_8UC3));
  dst->setTo(cv::Scalar(255,255,255)); // to set all values to 255

  const float validCellValue = 0.5 + uncertaintyMargin;
  for (int mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    for (int idy = 0; idy < sizeY; ++idy) {
      for (int idx = 0; idx < sizeX; ++idx) {
        if (mapStack[mapIdx].getCell(idx, idy) > validCellValue) {
          dst->at<cv::Vec3b>(idy,idx)[0] = mapColorBGR[mapIdx][0];
          dst->at<cv::Vec3b>(idy,idx)[1] = mapColorBGR[mapIdx][1];
          dst->at<cv::Vec3b>(idy,idx)[2] = mapColorBGR[mapIdx][2];
        }
      }
    }
  }

  INFO_MSG("Server returns map")
  cv::flip(*dst, *dst, 0);  // horizontal flip
  return dst;
};

rst::claas::MachineModel_Odemetrie getMachineModel() {
  mtxOdom.lock();
  rst::claas::MachineModel_Odemetrie model(machineModel);
  mtxOdom.unlock();
  return model;
}

// TODO Check if we need a que lenght of 1, to remove depricated odom messages
// Store the current machine model (odometry)
void storeMachineModel(rsb::EventPtr event) {
  mtxOdom.lock();
  // Get the ism out of the event
  boost::shared_ptr<rst::claas::MachineModel_Odemetrie> update
    = boost::static_pointer_cast<rst::claas::MachineModel_Odemetrie>(event->getData());

  machineModel.set_phi_kon      (update->phi_kon());
  machineModel.set_y_dis_roi    (update->y_dis_roi());
  machineModel.set_x_dis_roi    (update->x_dis_roi());
  machineModel.set_center_dresch(update->center_dresch());
  machineModel.set_x_dis_glo    (update->x_dis_glo());
  machineModel.set_y_dis_glo    (update->y_dis_glo());
  machineModel.set_x_kon_roi    (update->x_kon_roi());
  machineModel.set_x_kon_glo    (update->x_kon_glo());
  machineModel.set_y_kon_roi    (update->y_kon_roi());
  machineModel.set_y_kon_glo    (update->y_kon_glo());

  if (debug) {
    DEBUG_MSG("phi_kon      : " << machineModel.phi_kon      ())
    DEBUG_MSG("y_dis_roi    : " << machineModel.y_dis_roi    ())
    DEBUG_MSG("x_dis_roi    : " << machineModel.x_dis_roi    ())
    DEBUG_MSG("center_dresch: " << machineModel.center_dresch())
    DEBUG_MSG("x_dis_glo    : " << machineModel.x_dis_glo    ())
    DEBUG_MSG("y_dis_glo    : " << machineModel.y_dis_glo    ())
    DEBUG_MSG("x_kon_roi    : " << machineModel.x_kon_roi    ())
    DEBUG_MSG("x_kon_glo    : " << machineModel.x_kon_glo    ())
    DEBUG_MSG("y_kon_roi    : " << machineModel.y_kon_roi    ())
    DEBUG_MSG("y_kon_glo    : " << machineModel.y_kon_glo    ())
  }
  mtxOdom.unlock();
}

void doIsmFusion(rsb::EventPtr event) {

  // Get the map index
  int mapIdx = 0;
  bool isValidScope = false;
  for (; mapIdx < NUM_MAPS; ++mapIdx) {
    if (event->getScope() == mapScopes[mapIdx]) {
      isValidScope = true;
      break;
    }
  }

  // Invalid scope handling
  if (!isValidScope) {
    ERROR_MSG("Received invalid scope: " << event->getScope())
    return;
  }

  // Get the ism out of the event
  boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> update
    = boost::static_pointer_cast<rst::navigation::OccupancyGrid2DInt>(event->getData());

  const rst::geometry::Pose origin = update->origin();

  const float max_x = update->height() * update->resolution();
  const float min_x = 0.0f;
  const float max_y = update->width() * update->resolution();;
  const float min_y = 0.0f;

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  // Rotation is not allowed
  const float transX = origin.translation().x();
  const float transY = origin.translation().y();

  const int mapOffsetXStart = transX / def.resolution;
  const int mapOffsetXEnd   = (transX + max_x - min_x) / def.resolution;
  const int mapOffsetYStart = transY / def.resolution;
  const int mapOffsetYEnd   = (transY + max_y - min_y) / def.resolution;

  // TODO Store the pruned map, and shift it in the current view
  // Refresh the map if we have to
  mtxOdom.lock();
  if (machineModel.center_dresch() < 0)
    doMapRefresh = true;
  mtxOdom.unlock();
  if (doMapRefresh) {
    refreshMap();
    doMapRefresh = false;
  }

  // Do a pointwise sensor fusion
  for (int idy = mapOffsetYStart; idy < mapOffsetYEnd ; ++idy) {
    for (int idx = mapOffsetXStart; idx < mapOffsetXEnd ; ++idx) {
      // Get the index to get the correct cell out of the update
      const int y = idx-mapOffsetXStart;
      const int x = idy-mapOffsetYStart;
      const int index = x + (y * update->width());
      // Update the cell
      // There are only int values in the update:
      // -1 (unknown), 0 - 100 (occupancyness in percent)
      if (update->map().at(index) >= 0)
        mapStack[mapIdx].updateCell(idx,idy,float(update->map().at(index)) / 100.0f);
    }
  }

  if (debug) {
    boost::shared_ptr<cv::Mat> image(colorMapCallback());
    imshow( "Current View", *image );                    // Show our image inside it.
    cv::waitKey(1);                                      // Update the window
  }

}

// RSB Server function for the mapping server which replies with a 8bit map of the environment
class mapServer: public rsb::patterns::LocalServer::Callback<void, cv::Mat> {
public:
  boost::shared_ptr<cv::Mat> call(const std::string& /*methodName*/) {
    return colorMapCallback();
  }
};

void fillMapTest() {

  // Define the map as a sensor update
  mrpt::maps::COccupancyGridMap2D::TMapDefinition def;
  def.resolution = resolution;
  def.insertionOpts.maxOccupancyUpdateCertainty = maxOccupancyUpdateCertainty;
  def.insertionOpts.maxDistanceInsertion = maxDistanceInsertion;
  def.max_x = 4.0f;
  def.min_x = 0.0f;
  def.max_y = 3.0f;
  def.min_y = 0.0f;

  // Create the map from definition
  mrpt::maps::COccupancyGridMap2D sensorUpdate = *mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def);

  // Fill it with some occupied value
  sensorUpdate.fill (0.8);

  // Update a sensor scan:
  // The transformation already points to the lower left edge of the sensor scan
  const float transX = 2.0f;
  const float transY = 5.0f;

  int mapOffsetXStart = transX / def.resolution;
  int mapOffsetXEnd   = (transX + def.max_x - def.min_x) / def.resolution;
  int mapOffsetYStart = transY / def.resolution;
  int mapOffsetYEnd   = (transY + def.max_y - def.min_y) / def.resolution;

  // Iterate through every map layer
  for (int mapIdx = 0; mapIdx < NUM_MAPS ; ++mapIdx) {
    // Do a pointwise sensor fusion for every layer
    int mapOffsetXStart = transX / def.resolution * (1 + mapIdx);
    int mapOffsetXEnd   = (transX + def.max_x - def.min_x) / def.resolution * (1 + mapIdx);
    for (int idy = mapOffsetYStart; idy < mapOffsetYEnd ; ++idy) {
      for (int idx = mapOffsetXStart; idx < mapOffsetXEnd ; ++idx) {
        mapStack[mapIdx].updateCell(idx,idy,sensorUpdate.getCell(idx-mapOffsetXStart, idy-mapOffsetYStart));
      }
    }
  }
}

void handleProgramOptions(int argc, const char **argv) {

  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("debug", po::bool_switch(&debug)->default_value(false), "Enable debug outputs")
    ("test", po::bool_switch(&doTest)->default_value(false), "Enable testing")
    ("resolution", po::value<float>(&resolution), "Resolution of map in meter/cell")
    ("maxOccupancyUpdateCertainty", po::value<float>(&maxOccupancyUpdateCertainty), "Maximum update uncertainty")
    ("maxDistanceInsertion", po::value<float>(&maxDistanceInsertion), "Maximum distance insertion")
    ("max_x", po::value<float>(&max_x), "Maxmium value of x in meter")
    ("min_x", po::value<float>(&min_x), "Minimum value of x in meter")
    ("max_y", po::value<float>(&max_y), "Maxmium value of y in meter")
    ("min_y", po::value<float>(&min_y), "Minimum value of y in meter")
    ("uncertaintyMargin", po::value<float>(&uncertaintyMargin), "Minimum divitation from 0.5 (unknown), to be displayed")
    ("machineModelScope", po::value<std::string>(&machineModelScope), "Scope for receiving the machine models (odometry)")
    ("ismScopePrefix", po::value<std::string>(&ismScopePrefix), "Scope prefix for the inverse sensor models");


  INFO_MSG("Resolution of map in meter/cell: " << resolution                   )
  INFO_MSG("Maximum update uncertainty: "      << maxOccupancyUpdateCertainty  )
  INFO_MSG("Maximum distance insertion: "      << maxDistanceInsertion         )
  INFO_MSG("Maxmium value of x in meter: "     << max_x                        )
  INFO_MSG("Minimum value of x in meter: "     << min_x                        )
  INFO_MSG("Maxmium value of y in meter: "     << max_y                        )
  INFO_MSG("Minimum value of y in meter: "     << min_y                        )
  INFO_MSG("Minimum divitation from 0.5 (unknown), to be displayed: "     << uncertaintyMargin)
  INFO_MSG("Scope for receiving the machine models (odometry): " << machineModelScope    )
  INFO_MSG("Scope prefix for the inverse sensor models: " << ismScopePrefix    )
  for (int idx = 0; idx < NUM_MAPS; ++idx) {
    mapScopes[idx] = std::string(ismScopePrefix).append(mapSubScopes[idx]);
    INFO_MSG("Scope for " << mapSubScopes[idx] << ": " << mapScopes[idx])
  }

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }
  // afterwards, let program options handle argument errors
  po::notify(vm);
}

int main(int argc, const char **argv){

  handleProgramOptions(argc, argv);

  // Define the properties of one occupancy map
  def.resolution = resolution;
  def.insertionOpts.maxOccupancyUpdateCertainty = maxOccupancyUpdateCertainty;
  def.insertionOpts.maxDistanceInsertion = maxDistanceInsertion;
  def.max_x = max_x;
  def.min_x = min_x;
  def.max_y = max_y;
  def.min_y = min_y;

  // Create the maps
  for (unsigned char mapIdx = 0; mapIdx < NUM_MAPS; ++mapIdx) {
    mapStack.push_back(*mrpt::maps::COccupancyGridMap2D::CreateFromMapDefinition(def));
  }
  refreshMap();


  if (doTest) {
    INFO_MSG("Test the mapstructure by printing rectangulars in every layer an printing them")
    // Fill all the maps with small rectangles which are overlapping a bit
    fillMapTest();
    // Get the map as image
    boost::shared_ptr<cv::Mat> image(colorMapCallback());
    imshow( "Display window", *image );                  // Show our image inside it.
    cv::waitKey(0);                                      // Wait for a keystroke in the window
    INFO_MSG("Exit")
    return 0;
  }


// Get the RSB factory

#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif

  // Register converters
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::navigation::OccupancyGrid2DInt > > ismConverter(new rsb::converter::ProtocolBufferConverter<rst::navigation::OccupancyGrid2DInt >());
  rsb::converter::converterRepository<std::string>()->registerConverter(ismConverter);
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie > > odomConverter(new rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie >());
  rsb::converter::converterRepository<std::string>()->registerConverter(odomConverter);

  // Prepare RSB listener for all OGM scopes
  rsb::ListenerPtr ismListener = factory.createListener(ismScopePrefix);
//  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::navigation::OccupancyGrid2DInt>>>ismQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::navigation::OccupancyGrid2DInt>>(1));
//  ismListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::navigation::OccupancyGrid2DInt>(ismQueue)));
  ismListener->addHandler(HandlerPtr(new EventFunctionHandler(&doIsmFusion)));

  // Prepare RSB listener for odometry and map messages (MachineModel)
  rsb::ListenerPtr machineModelListener = factory.createListener(machineModelScope);
  machineModelListener->addHandler(HandlerPtr(new EventFunctionHandler(&storeMachineModel)));

  // As events are received asynchronously we have to wait here for
  // them.
  rsc::misc::initSignalWaiter();
  return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
}

