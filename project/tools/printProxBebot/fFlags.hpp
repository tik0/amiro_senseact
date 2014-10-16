#include <boost/program_options.hpp>

namespace fFlags
{
std::size_t uiPeriod_ms = 250;


void handleCommandline(int argc, const char *argv[]) {

   namespace po = boost::program_options;

   float fFrequency = -1;
   
    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")("frequency,f",
            po::value < float > (&fFrequency), "Frequency of reading the IR Sensor buffer.");

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

    // Calculate the period out of the frequency
    if ( fFrequency >= 0 ) {
      uiPeriod_ms = (std::size_t)(1.0 / ( fFrequency) * 1000.0);
    }
}
}