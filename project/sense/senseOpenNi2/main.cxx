#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <converter/iplImageConverter/IplImageConverter.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// For program options
#include <boost/program_options.hpp>

// RST
#include <rst/vision/Image.pb.h>
#ifndef RST_013_USED
# include <rst/vision/EncodedImage.pb.h>
#endif


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>

// #include <OniSampleUtilities.h>

#define SAMPLE_READ_WAIT_TIMEOUT 2000 // 2000ms

using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
// using namespace rst::converters::opencv;

static std::string g_sOutScope = "/depthImage";

int main(int argc, char ** argv) {
  namespace po = boost::program_options;

  static int imageCompression = 0;
  static int compressionValue = -1;
  static string fileExtention = ".bmp";

  po::options_description options("Allowed options");
  options.add_options() ("help,h", "Display a help message.")
    ("outscope,o", po::value<std::string>(&g_sOutScope), "Scope for sending images.")
    ("image,i", po::value<std::string>(&fileExtention), "asda.")
    ("compression,c", po::value<int>(&compressionValue)->default_value(compressionValue), "Enable image compression with value betweeen 0-100.");

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

  INFO_MSG("Scope: " << g_sOutScope)
  INFO_MSG("compression " << compressionValue)

  if (compressionValue >= 0)
    imageCompression = 1;

  // //////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  // boost::shared_ptr<IplImageConverter> converter(new IplImageConverter());
  // converterRepository<std::string>()->registerConverter(converter);
  rsb::Factory &factory = rsb::getFactory();

  // Create the informer

  Informer<rst::vision::Image>::Ptr informer;
  #ifdef RST_013_USED
  Informer<std::string>::Ptr informerCompressed;
  #else
  Informer<rst::vision::EncodedImage>::Ptr informerCompressed;
  #endif

  if (imageCompression) {
    #ifdef RST_013_USED
    // boost::shared_ptr< rsb::converter::ProtocolBufferConverter<std::string> >
    //  converter(new rsb::converter::ProtocolBufferConverter<std::string>());
    //  rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informerCompressed = factory.createInformer<std::string>(Scope(g_sOutScope));
    #else
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::vision::EncodedImage>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informerCompressed = factory.createInformer<rst::vision::EncodedImage>(Scope(g_sOutScope));
    #endif
  } else {
    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::Image> >
    converter(new rsb::converter::ProtocolBufferConverter<rst::vision::Image>());
    rsb::converter::converterRepository<std::string>()->registerConverter(converter);
    informer = factory.createInformer<rst::vision::Image>(Scope(g_sOutScope));
  }

  // stuff for compression
  std::vector<unsigned char> buff;
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = compressionValue; // default(95) 0-100

  // Informer<IplImage>::Ptr informer = getFactory().createInformer<IplImage> (Scope(g_sOutScope));
  // //////////////////////////////////////////////////////////////////////////////////////////////////

  Status rc = OpenNI::initialize();
  if (rc != STATUS_OK) {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }

  Device device;
  rc = device.open(ANY_DEVICE);
  if (rc != STATUS_OK) {
    printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
    return 2;
  }

  VideoStream depth;

  if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
    rc = depth.create(device, SENSOR_DEPTH);
    if (rc != STATUS_OK) {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
      return 3;
    }
  }

  rc = depth.start();
  if (rc != STATUS_OK) {
    printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    return 4;
  }

  VideoFrameRef frame;


  // Allocate a frame object to store the picture
  //  boost::shared_ptr<cv::Mat> frame(new cv::Mat);

  // cv::VideoCapture capture(CV_CAP_OPENNI);
  // Mat depthMap;
  // double minVal, maxVal;
  // float alpha, beta;
  while (true) {
    int changedStreamDummy;
    VideoStream * pStream = &depth;
    rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != STATUS_OK) {
      printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
      continue;
    }

    rc = depth.readFrame(&frame);
    if (rc != STATUS_OK) {
      printf("Read failed!\n%s\n", OpenNI::getExtendedError());
      continue;
    }

    if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM) {
      printf("Unexpected frame format\n");
      continue;
    }

    // DepthPixel * pDepth = (DepthPixel *) frame.getData();
    const openni::DepthPixel * pDepth = (const openni::DepthPixel *) frame.getData();
    // const uint16_t * pDepth = (const uint16_t *) frame.getData();
    // DepthPixel *pDepth = new DepthPixel[frame.getHeight()*frame.getWidth()];
    // memcpy(pDepth, frame.getData(), frame.getHeight()*frame.getWidth()*sizeof(uint16_t));

    int middleIndex = (frame.getHeight() + 1) * frame.getWidth() / 2;
    printf("[%08llu] %8d\n", (long long) frame.getTimestamp(), pDepth[middleIndex]);

    // Convert to an image which can be send via RSB
    // boost::shared_ptr<cv::Mat> imageCv(new cv::Mat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1, (void *) pDepth, cv::Mat::AUTO_STEP));
    // cv::Mat imageCv(new cv::Mat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1, (void *) pDepth, cv::Mat::AUTO_STEP));
    // cv::Mat imageCv(cv::Size(frame.getHeight(), frame.getWidth()), CV_16U, (void *) pDepth, cv::Mat::AUTO_STEP);

    // cv::Mat depthMat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1);
    // memcpy(depthMat.data, pDepth, frame.getHeight() * frame.getWidth() * sizeof(uint16_t));


    //marvin stuff
    // Mat depthDataImage1C;
    // cv::minMaxLoc(depthMat, &minVal, &maxVal);
    // alpha = 255.0 / maxVal;
    // beta  = 255.0 - maxVal * alpha;
    // depthMat.convertTo(depthDataImage1C, CV_8UC1, alpha, beta);
    // cv::cvtColor(depthDataImage1C, depthMat, CV_GRAY2RGB);
    // cv::flip(depthMat, depthMat, 1);

    // cout << depthMat << endl;

    //
    // imageCv.convertTo(imageCv, CV_8U);

    // for (int i = 0; i < imageCv.size().height * imageCv.size().width; i++) {
    //   cout << "a: " << static_cast<uchar>(pDepth[i]) << endl;
    // }
    // memmove(imageCv.data, pDepth, frame.getHeight()*frame.getWidth());

    // cout << "channels:" << depthMat.channels() << " type:" << depthMat.type() << endl;

    // cout << "data:" << imageCv << endl;
    // cout << "data: " << *imageCv.data << endl;
    // printf("data: %s \n", depthMat.data);


    // capture >> depthMap;
    // capture.grab();
    // capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );


    if (imageCompression) {
      cv::Mat depthMat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1);
      memcpy(depthMat.data, pDepth, frame.getHeight() * frame.getWidth() * sizeof(uint16_t));
      // cv::Mat depthMat8UC1;
      // cv::Mat depthMat8UC3;
      // double minVal, maxVal;
      // float alpha, beta;
      // cv::minMaxLoc(depthMat, &minVal, &maxVal);
      // alpha = 255.0 / maxVal;
      // beta  = 255.0 - maxVal * alpha;
      // depthMat.convertTo(depthMat, CV_8UC1, alpha, beta);
      // cv::cvtColor(depthMat8UC1, depthMat8UC3, CV_GRAY2RGB);
      cv::imencode(".png", depthMat, buff, param);
      #ifdef RST_013_USED
      boost::shared_ptr<std::string> framePng(new std::string(buff.begin(), buff.end()));
      informerCompressed->publish(framePng);
      INFO_MSG("Compressed Image published")
      #else
      boost::shared_ptr<rst::vision::EncodedImage> encodedImage(new rst::vision::EncodedImage());
      encodedImage->set_encoding(rst::vision::EncodedImage::JPG);
      encodedImage->set_data(std::string(reinterpret_cast<const char *>(&buff[0]), buff.size()));
      informerCompressed->publish(encodedImage);
      INFO_MSG("Encoded Image published")
      #endif
    } else {
      boost::shared_ptr<rst::vision::Image> rstVisionImage(new rst::vision::Image());
      rstVisionImage->set_width(frame.getHeight());
      rstVisionImage->set_height(frame.getHeight());
      rstVisionImage->set_data((void*) pDepth, frame.getHeight() * frame.getHeight());
      // rstVisionImage->set_data(reinterpret_cast<char *>(const_cast<uint16_t*>(pDepth)), frame.getHeight() * frame.getHeight() * sizeof(uint16_t) * 1); // conversion from uchar* tp char*
      // rstVisionImage->set_data((char *) tempImage->imageData, tempImage->imageSize)
      rstVisionImage->set_channels(1);
      rstVisionImage->set_color_mode(rst::vision::Image::COLOR_GRAYSCALE);
      rstVisionImage->set_depth(rst::vision::Image::DEPTH_16U);
      // Send the data.
      informer->publish(rstVisionImage);
      INFO_MSG("Image published")
      // cout << rstVisionImage->data() << endl;
    }

    // boost::shared_ptr<IplImage> sendIplImage(new IplImage(*image));
    // cout << reinterpret_cast<char **>(&pDepth) << endl;
    // boost::shared_ptr<rst::vision::Image> rstVisionImage(new rst::vision::Image());
    // rstVisionImage->set_width(frame.getWidth());
    // rstVisionImage->set_height(frame.getHeight());
    // rstVisionImage->set_data(reinterpret_cast<char *>((void *)pDepth));
    // cout << cout << (void *) pDepth << endl;
    // rstVisionImage->set_data(reinterpret_cast<char *>(const_cast<void *>(frame.getData())));
    // rstVisionImage->set_data(reinterpret_cast<char *>(pDepth));
    // rstVisionImage->set_data(reinterpret_cast<char *>(reinterpret_cast<void *>(pDepth)));
    // rstVisionImage->set_width(imageCv.size().width);
    // rstVisionImage->set_height(imageCv.size().height);
    // rstVisionImage->set_data(reinterpret_cast<char *>(imageCv->data)); // conversion from uchar* tp char*
    // rstVisionImage->set_data(reinterpret_cast<char *>(pDepth);
    // informer->publish(rstVisionImage);
  }

  depth.stop();
  depth.destroy();
  device.close();
  OpenNI::shutdown();

  return 0;
} // main
