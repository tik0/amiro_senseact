/**
 * Author: Jonas D. Homburg
 */

// ROS
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
// ROS - OpenCV Bridge
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
//#include <rst/vision/Image.pb.h>
#include <rst/generic/Value.pb.h>

//OpenCv
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "Constants.hpp"

using namespace std;

// RSB Listener Scope
//const string rsbListenerScope = constants::rsbImageScope;

// ROS Publish Topic
//const string rosPublishTopic = constants::rosImageTopic;
// ros::Publisher rosPublisher;
ros::Publisher floorProxPub;
//image_transport::Publisher imagePublisher;

// program name
const string programName = "rstFloorProximityValueToRosIntArray";

// frame id
//const string frameID = "";

// Rsb event type
//const string rsbEventType = "rst::vision::Image";

void processValueArray(rsb::EventPtr event) {
  if (event->getType() != "rst::generic::Value"){
    return;
  }

  boost::shared_ptr<rst::generic::Value> value = boost::static_pointer_cast<rst::generic::Value>(event->getData());
  if (value->type() != rst::generic::Value::ARRAY){
    return;
  }

  int size = value->array_size();
  std::vector<uint16_t> data (size, 0);
  rst::generic::Value entry;
  for (int i=0; i<size; i++){
    entry=value->array(i);
    if (entry.type()!=rst::generic::Value::INT){
      return;
    }
    data[i]=entry.int_();
    cout << data[i];
  }
  cout << endl;
  std_msgs::UInt16MultiArray floorProxMsg;
  floorProxMsg.data=data;
  floorProxPub.publish(floorProxMsg);
}

int main(int argc, char *argv[]) {
  cout << "Start: " << programName << endl;

	// Init ROS
	ros::init(argc, argv, programName);
  ros::NodeHandle node;
  floorProxPub = node.advertise<std_msgs::UInt16MultiArray>(constants::rosFloorProxTopic, 10);
  // imagePublisher = node.advertise(rosPublishTopic, 1);
  //image_transport::ImageTransport imageTransport(node);
	//imagePublisher = imageTransport.advertise(rosPublishTopic, 1);

	// Create RSB factory
  //#if RSB_VERSION_NUMERIC<1200
//    rsb::Factory& factory = rsb::Factory::getInstance();
//  #else
    rsb::Factory& factory = rsb::getFactory();
//  #endif

	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
		converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
		rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr floorProxListener = factory.createListener(constants::rsbFloorProxScope);
  floorProxListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processValueArray)));

	ros::spin();

 	return 0;
}
