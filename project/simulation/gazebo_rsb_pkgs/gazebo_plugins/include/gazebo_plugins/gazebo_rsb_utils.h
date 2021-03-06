/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef GAZEBO_RSB_UTILS_H
#define GAZEBO_RSB_UTILS_H
#include <map>
#include <boost/algorithm/string.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>

// Opencv
#include <opencv2/opencv.hpp>
// #include <cvplot/cvplot.h>

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

#include <math.h>

namespace gazebo
{

/**
* Accessing model name like suggested by nkoenig at http://answers.gazebosim.org/question/4878/multiple-robots-with-ros-plugins-sensor-plugin-vs/
* @param parent
* @return accessing model name
**/
inline std::string GetModelName ( const sensors::SensorPtr &parent )
{
    std::string modelName;
    std::vector<std::string> values;
    std::string scopedName = parent->GetScopedName();
    boost::replace_all ( scopedName, "::", "," );
    boost::split ( values, scopedName, boost::is_any_of ( "," ) );
    if ( values.size() < 2 ) {
        modelName = "";
    } else {
        modelName = values[1];
    }
    return modelName;
}

// 2D Gauss function
inline double draw2DGauss(cv::Mat x /*2x1 matrix*/, cv::Mat mu /*2x1 matrix*/, cv::Mat sigma /*2x2 matrix*/) {

  cv::Mat sigmaInv(2, 2, CV_64FC1);
  const double sigmaDet = determinant(sigma);
  sigmaInv.at<double>(0,0) = sigma.at<double>(1,1) / sigmaDet;
  sigmaInv.at<double>(1,0) = -sigma.at<double>(1,0) / sigmaDet;
  sigmaInv.at<double>(0,1) = -sigma.at<double>(0,1) / sigmaDet;
  sigmaInv.at<double>(1,1) = sigma.at<double>(0,0) / sigmaDet;

  cv::Mat tmp = x - mu;
  cv::Mat tmpT; cv::transpose(tmp, tmpT);
  cv::Mat exponent =  tmpT * sigmaInv * tmp;  // Should result in a 1x1 matrix
  return 1 / (2.0 * M_PI * sigmaDet) * exp(-0.5 * exponent.at<double>(0));
}

// 1D Gauss function
inline double draw1DGauss(double x, double mu, double sigma) {
  double tmp = pow(x - mu, 2);
  double exponent =  tmp / sigma;
  return 1 / (sqrt(2.0 * M_PI) * sigma) * exp(-0.5 * exponent);
}

inline void sendImage (const rsb::Informer<std::string>::Ptr informer, cv::Mat img)
{
  std::vector<uchar> buf;
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
  cv::imencode(".jpg", img, buf, compression_params);

  rsb::Informer<std::string>::DataPtr frameJpg(new std::string(buf.begin(), buf.end()));
  informer->publish(frameJpg);
}

// template<typename T>
// inline void sendPlot (const std::string figure_name, const rsb::Informer<std::string>::Ptr informer, const T* p, int count, int step, int R, int G, int B)
// {
//   if(count == 0 || p == NULL || informer == NULL)
//     return;
// 
//   IplImage * plot = CvPlot::plot(figure_name, p, count, step, R, G, B);
//   sendImage(informer, cv::Mat(plot));
//   CvPlot::clear(figure_name);
//   cvReleaseImage(&plot);
// }

/**
* @brief Reads the name space tag of a sensor plugin
* @param parent
* @param sdf
* @param pInfo
* @return node namespace
**/
inline std::string GetRobotNamespace ( const sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL )
{
    std::string name_space;
    std::stringstream ss;
    if ( sdf->HasElement ( "robotNamespace" ) ) {
        name_space = sdf->Get<std::string> ( "robotNamespace" );
        if ( name_space.empty() ) {
            ss << "the 'robotNamespace' param was empty";
            name_space = GetModelName ( parent );
        } else {
            ss << "Using the 'robotNamespace' param: '" <<  name_space << "'";
        }
    } else {
        ss << "the 'robotNamespace' param did not exit";
    }
    if ( pInfo != NULL ) {
        printf ( "%s Plugin (robotNamespace = %s), Info: %s\n" , pInfo, name_space.c_str(), ss.str().c_str() );
    }
    return name_space;
}

inline void PrintPluginInfoString ( std::string modelName, std::string pluginName, std::string mesg)
{
  printf ( "%s: %s Plugin %s\n" , modelName.c_str(), pluginName.c_str(), mesg.c_str() );
}


inline void replace(std::string& str, const std::string& from, const std::string& to) {
  bool doReplace = true;
  while (true) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return;
    str.replace(start_pos, from.length(), to);
  }
}

// Convert string to rsb scope:
// my::scope::subscope -> /my/scope/subscope
std::string rsbScopeFromGazeboFrame(std::string frame) {
  std::string scope("/" + frame);
  replace(scope, "::", "/");
  replace(scope, " ", "_");
  return scope;
}

// Split helper function
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

// Split function
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

// Gets an subscope from string:
// my::scope::subscope -> subscope
std::string getSubScope(std::string scope) {
  std::vector<std::string> splitString = split(scope, ':');
  return splitString[splitString.size()-1];
}

// Gets the parent from string:
// my::scope::subscope -> my
std::string getParentScope(std::string scope) {
  std::vector<std::string> splitString = split(scope, ':');
  return splitString[2];
}

inline std::string GetSdfElementValue ( const std::string modelName, const sdf::ElementPtr &sdf, std::string elementName, const char *pInfo = NULL )
{
    std::string name_space;
    std::stringstream ss;
    std::cout << std::flush;
    if ( sdf->HasElement ( elementName ) ) {
        // Get the subscope, because there might be some capsulation
        name_space = getSubScope(sdf->Get<std::string> ( elementName ));
        if ( name_space.empty() ) {
            ss << "the '"<< elementName << "' param was empty";
        } else {
            ss << "Using the '"<< elementName << "' param: '" <<  name_space << "'";
        }
    } else {
        ss << "the '"<< elementName << "' param did not exit";
    }
    if ( pInfo != NULL ) {
        PrintPluginInfoString(modelName, std::string(pInfo), std::string("( " + elementName + " = " + name_space + " ), Info: " + ss.str()));
    }
    return name_space;
}


// Gets the myScope scope from string:
// my::*myScope*::subscope -> *myScope*
std::string getFirstScopeContains(std::string scope, std::string stringContains) {
  std::vector<std::string> splitString = split(scope, ':');
  for (size_t idx = 0; idx < splitString.size(); ++idx) {
    if (splitString[idx].find_first_of(stringContains) !=  std::string::npos)
      return splitString[idx];
  }
  return std::string("NoScopeContains" + stringContains);
}

// /**
//  * Gazebo ros helper class
//  * The class simplifies the parameter and rosnode handling
//  * @author Markus Bader <markus.bader@tuwien.ac.at>
//  **/
// class GazeboRos
// {
// private:
//     sdf::ElementPtr sdf_;       /// sdf to read
//     std::string plugin_;        /// name of the plugin class
//     std::string namespace_;     /// name of the launched node
//     boost::shared_ptr<ros::NodeHandle> rosnode_; /// rosnode
//     std::string tf_prefix_;     /// prefix for the ros tf plublisher if not set it uses the namespace_
//     std::string info_text;      /// info text for log messages to identify the node
//     /**
//      * Reads the common plugin parameters used by the constructor
//      **/
//     void readCommonParameter ();
// public:
//     /**
//      * Constructor
//      * @param _parent models parent
//      * @param _sdf sdf to read
//      * @param _name of the plugin class
//      **/
//     GazeboRos ( physics::ModelPtr &_parent, sdf::ElementPtr _sdf, const std::string &_plugin )
//         : sdf_ ( _sdf ), plugin_ ( _plugin ) {
//         namespace_ = _parent->GetName ();
//         if ( !sdf_->HasElement ( "robotNamespace" ) ) {
//             ROS_INFO ( "%s missing <robotNamespace>, defaults is %s", plugin_.c_str(), namespace_.c_str() );
//         }  else {
//             namespace_ = sdf_->GetElement ( "robotNamespace" )->Get<std::string>();
//             if ( namespace_.empty() ) {
//                 namespace_ = _parent->GetName();
//             }
//         }
//         if ( !namespace_.empty() )
//             this->namespace_ += "/";
//         rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( namespace_ ) );
//         info_text = plugin_ + "(ns = " + namespace_ + ")";
//         readCommonParameter ();
//     }
//     /**
//      * Constructor
//      * @param _parent sensor parent
//      * @param _sdf sdf to read
//      * @param _name of the plugin class
//      **/
//     GazeboRos ( sensors::SensorPtr _parent, sdf::ElementPtr _sdf, const std::string &_plugin )
//         : sdf_ ( _sdf ), plugin_ ( _plugin ) {
// 
//         std::stringstream ss;
//         if ( sdf_->HasElement ( "robotNamespace" ) ) {
//             namespace_ = sdf_->Get<std::string> ( "robotNamespace" );
//             if ( namespace_.empty() ) {
//                 ss << "the 'robotNamespace' param was empty";
//                 namespace_ = GetModelName ( _parent );
//             } else {
//                 ss << "Using the 'robotNamespace' param: '" <<  namespace_ << "'";
//             }
//         } else {
//             ss << "the 'robotNamespace' param did not exit";
//         }
//         info_text = plugin_ + "(ns = " + namespace_ + ")";
//         ROS_INFO ( "%s: %s" , info_text.c_str(), ss.str().c_str() );
//         readCommonParameter ();
//     }
// 
//     /**
//      * Returns info text used for log messages
//      * @return class name and node name as string
//      **/
//     const char* info() const;
//     /**
//      * returns the initialized created within the constuctor
//      * @return rosnode
//      **/
//     boost::shared_ptr<ros::NodeHandle>& node();;
//     /**
//      * returns the initialized within the constuctor
//      * @return rosnode
//      **/
//     const boost::shared_ptr<ros::NodeHandle>& node() const;
//     /**
//      * resolves a tf frame name by adding the tf_prefix initialized within the constuctor
//      * @param _name
//      * @retun resolved tf name
//      **/
//     std::string resolveTF ( const std::string &_name );
// 
//     /**
//      * reads the follwoing _tag_name paramer or sets a _default value
//      * @param _value
//      * @param _tag_name
//      * @param _default
//      * @retun sdf tag value
//      **/
//     void getParameterBoolean ( bool &_value, const char *_tag_name, const bool &_default );
//     /**
//      * reads the follwoing _tag_name paramer
//      * @param _value
//      * @param _tag_name
//      * @retun sdf tag value
//      **/
//     void getParameterBoolean ( bool &_value, const char *_tag_name );
//     /**
//      * retuns a JointPtr based on an sdf tag_name entry
//      * @param _value
//      * @param _tag_name
//      * @param _joint_default_name
//      * @retun JointPtr
//      **/
//     physics::JointPtr getJoint ( physics::ModelPtr &_parent, const char *_tag_name, const std::string &_joint_default_name );
//     /**
//      * checks if the ros not is initialized
//      * @retun JointPtr
//      **/
//     void isInitialized();
// 
// 
//     /**
//      * reads the follwoing _tag_name paramer or sets a _default value
//      * @param _value
//      * @param _tag_name
//      * @param _default
//      * @retun sdf tag value
//      **/
//     template <class T>
//     void getParameter ( T &_value, const char *_tag_name, const T &_default ) {
//         _value = _default;
//         if ( !sdf_->HasElement ( _tag_name ) ) {
//             ROS_WARN ( "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
//         } else {
//             this->getParameter<T> ( _value, _tag_name );
//         }
//     }
//     /**
//      * reads the follwoing _tag_name paramer
//      * @param _value
//      * @param _tag_name
//      * @param _default
//      * @retun sdf tag value
//      **/
//     template <class T>
//     void getParameter ( T &_value, const char *_tag_name ) {
//         if ( sdf_->HasElement ( _tag_name ) ) {
//             _value = sdf_->GetElement ( _tag_name )->Get<T>();
//         }
//         ROS_DEBUG ( "%s: <%s> = %s", info(), _tag_name, boost::lexical_cast<std::string> ( _value ).c_str() );
// 
//     }
// 
//     /**
//      * reads the following _tag_name paramer and compares the value with an _options map keys and retuns the corresponding value.
//      * @param _value
//      * @param _tag_name
//      * @param _default
//      * @retun sdf tag value
//      **/
//     template <class T>
//     void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options, const T &_default ) {
//         _value = _default;
//         if ( !sdf_->HasElement ( _tag_name ) ) {
//             ROS_WARN ( "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
//         } else {
//             this->getParameter<T> ( _value, _tag_name, _options );
//         }
//     }
//     /**
//      * reads the following _tag_name paramer and compares the value with an _options map keys and retuns the corresponding value.
//      * @param _value
//      * @param _tag_name
//      * @param _default
//      * @retun sdf tag value
//      **/
//     template <class T>
//     void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options ) {
//         typename std::map<std::string, T >::const_iterator it;
//         if ( sdf_->HasElement ( _tag_name ) ) {
//             std::string value = sdf_->GetElement ( _tag_name )->Get<std::string>();
//             it = _options.find ( value );
//             if ( it == _options.end() ) {
//                 ROS_WARN ( "%s: <%s> no matching key to %s", info(), _tag_name, value.c_str() );
//             } else {
//                 _value = it->second;
//             }
//         }
//         ROS_DEBUG ( "%s: <%s> = %s := %s",  info(), _tag_name, ( it == _options.end() ?"default":it->first.c_str() ), boost::lexical_cast<std::string> ( _value ).c_str() );
//     }
// };
// 
// typedef boost::shared_ptr<GazeboRos> GazeboRosPtr;
}
#endif



