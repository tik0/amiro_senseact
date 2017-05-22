// ============================================================================
// Name        : rst_value_array_to_ros_int_array.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : Recieve int32_t array as Value via rsb and publish them via ros.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sai_msgs/Int32MultiArrayStamped.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/generic/Value.pb.h>

using namespace std;

// Ros Listener Topic
string rosListenerTopic;

// RSB Publish Scope
string rsbPublishScope;

rsb::Informer<rst::generic::Value>::Ptr informer;

// program name
const string programName = "ros_int_multiarray_rst_value_array";


void process (const sai_msgs::Int32MultiArrayStamped::ConstPtr& msg){
  rsb::Informer<rst::generic::Value>::DataPtr rsbmsg(new rst::generic::Value);
  rsbmsg->set_type(rst::generic::Value::ARRAY);
  rst::generic::Value* nV;
  for (uint i=0; i<msg->data.data.size();i++){
    nV=rsbmsg->add_array();
    nV->set_type(rst::generic::Value::INT);
    nV->set_int_(msg->data.data[i]);
    //ROS_INFO("I: %i;\tValue: %i",i,msg->data.data[i]);
  }
  informer->publish(rsbmsg);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  private_nh.param<string>("rosListenerTopic", rosListenerTopic, "/motor");
  ROS_INFO("rosListenerTopic: %s", rosListenerTopic.c_str());
  private_nh.param<string>("rsbPublishScope", rsbPublishScope, "/motor/03");
  ROS_INFO("rsbPublishScope: %s", rsbPublishScope.c_str());

  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
			converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory& factory = rsb::getFactory();
  informer = factory.createInformer<rst::generic::Value>(rsbPublishScope);

  ros::Subscriber sub = node.subscribe(rosListenerTopic, 1, process);

  ros::spin();

 	return 0;
}
