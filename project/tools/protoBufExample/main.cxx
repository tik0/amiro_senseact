#include <rsb/Factory.h>
#include <rsb/Listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rsc/threading/SynchronizedQueue.h>

#include <rsb/QueuePushHandler.h>

// For program options
#include <boost/program_options.hpp>

#include <string>

#include "/usr/include/rst-converters0.11/rst/converters/opencv/IplImageConverter.h"
#include <rsb/converter/Repository.h>

namespace po = boost::program_options;
using namespace std;


string inScope = "/video";
string outScope = "/results";


int main(int argc, char *argv[]) {

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")("inscope,i",
            po::value < string > (&inScope),
            "Scope for receiving input images.")("outscope,o",
            po::value < string > (&outScope), "Scope for sending the results.");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);
    
    
    // first, process the help option
    if (vm.count("help")) {
        cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    // START RSB
    rsb::Factory& factory = rsb::getFactory();
    
    rsb::converter::Converter<string>::Ptr imageConverter(
            new rst::converters::opencv::IplImageConverter());
    rsb::converter::converterRepository<string>()->registerConverter(imageConverter);
    
    rsb::ListenerPtr imageListener = factory.createListener(
            rsb::Scope(inScope));
    
    boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> > > imageQueue(
                    new rsc::threading::SynchronizedQueue<boost::shared_ptr<IplImage> >(1));

    imageListener->addHandler(
            rsb::HandlerPtr(new rsb::QueuePushHandler<IplImage>(imageQueue)));
    
    
    while (true) {

        cv::Mat image = cv::Mat(imageQueue->pop().get(), true);

        cv::imshow("input", image);
        cv::waitKey(1);
    }
    
    return 1;

}