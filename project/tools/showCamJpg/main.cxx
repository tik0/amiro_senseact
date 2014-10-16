
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace rsb::converter;


int main() {


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create and start the listener
  rsb::ListenerPtr listener = factory.createListener("/images");
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(imageQueue)));

  // Pop the images and show them
  while (true) {
    // Get the image as string
    std::string imageJpg = *imageQueue->pop().get();
    // Copy to a vector
    std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
    // Decode the image
    cv::Mat image = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
    // Exit the program if any key was pressed
    if ( cv::waitKey(1) >= 0 )
      break;
    cv::imshow("input", image);
  }

  return 0;

}
