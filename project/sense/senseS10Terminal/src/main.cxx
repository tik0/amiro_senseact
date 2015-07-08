//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the distance information of DS_ODSL96B_M_TOF_S12
//               and send it via RSB
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>
#include <types/Nmea.pb.h>

// Serial interface
#include "rs232.h"
#include "nmea/nmea.h"

#define BUFSIZE 255

static std::string rsbOutScope = "/sense/Nmea";
//static uint32_t rsbPeriod = 0;
static int32_t cport_nr = 3;
static int32_t bdrate = 19200;

int main(int argc, char **argv) {

  INFO_MSG("")
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending odometry values")
    //("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)")
    ("port,p", po::value < int32_t > (&cport_nr), "Portnumber of ttyS<0-15> (Standardvalue: 3)")
    ("bdrate,b", po::value < int32_t > (&bdrate), "Baudrate (Standardvalue: 19200)");

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

//  // Register new converter for NMEA GPGGA string
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGGA> > converter_GPGGA(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGGA>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGGA);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSA> > converter_GPGSA(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSA>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGSA);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSV> > converter_GPGSV(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGSV>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGSV);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPRMC> > converter_GPRMC(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPRMC>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPRMC);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPVTG> > converter_GPVTG(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPVTG>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPVTG);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGLL> > converter_GPGLL(new rsb::converter::ProtocolBufferConverter<rst::claas::Nmea_GPGLL>);
  rsb::converter::converterRepository<std::string>()->registerConverter(converter_GPGLL);

  // Prepare RSB informer
  rsb::Factory& factory = rsb::getFactory();
  rsb::Informer<rst::claas::Nmea_GPGGA>::Ptr informer_GPGGA = factory.createInformer<rst::claas::Nmea_GPGGA> (rsbOutScope);
  rsb::Informer<rst::claas::Nmea_GPGSA>::Ptr informer_GPGSA = factory.createInformer<rst::claas::Nmea_GPGSA> (rsbOutScope);
  rsb::Informer<rst::claas::Nmea_GPGSV>::Ptr informer_GPGSV = factory.createInformer<rst::claas::Nmea_GPGSV> (rsbOutScope);
  rsb::Informer<rst::claas::Nmea_GPRMC>::Ptr informer_GPRMC = factory.createInformer<rst::claas::Nmea_GPRMC> (rsbOutScope);
  rsb::Informer<rst::claas::Nmea_GPVTG>::Ptr informer_GPVTG = factory.createInformer<rst::claas::Nmea_GPVTG> (rsbOutScope);
  rsb::Informer<rst::claas::Nmea_GPGLL>::Ptr informer_GPGLL = factory.createInformer<rst::claas::Nmea_GPGLL> (rsbOutScope);

  // Datastructure for the nmea messages
  rsb::Informer<rst::claas::Nmea_GPGGA>::DataPtr msg_GPGGA(new rst::claas::Nmea_GPGGA);
  rsb::Informer<rst::claas::Nmea_GPGSA>::DataPtr msg_GPGSA(new rst::claas::Nmea_GPGSA);
  rsb::Informer<rst::claas::Nmea_GPGSV>::DataPtr msg_GPGSV(new rst::claas::Nmea_GPGSV);
  rsb::Informer<rst::claas::Nmea_GPRMC>::DataPtr msg_GPRMC(new rst::claas::Nmea_GPRMC);
  rsb::Informer<rst::claas::Nmea_GPVTG>::DataPtr msg_GPVTG(new rst::claas::Nmea_GPVTG);
  rsb::Informer<rst::claas::Nmea_GPGLL>::DataPtr msg_GPGLL(new rst::claas::Nmea_GPGLL);
  
  // Set up the serial connection
  int n;
  unsigned char buf[BUFSIZE+1];
  char mode[]={'8','N','1',0};
  int idx = 0;

  char buffer[255] = {0};
  // Start the synchronized communication
  if(RS232_OpenComport(cport_nr, bdrate, mode, BUFSIZE))
  {
    ERROR_MSG("Can not open comport")
    return(1);
  }

  nmeaINFO info;
  nmeaPARSER parser;

  nmea_zero_INFO(&info);
  nmea_parser_init(&parser);

  while(1) {

	n = RS232_PollComport(cport_nr, buf, 1);
	INFO_MSG("Bytes: " << n)

	// Syncronization to get the start of an NMEA string
	if ((buf[0]) != '$') {
	  if(RS232_OpenComport(cport_nr, bdrate, mode, 1))
	  {
		ERROR_MSG("Can not open comport")
		return(1);
	  }
	}else{
		while(1){
			n = RS232_PollComport(cport_nr, buf, 1);
			buffer[idx] = buf[0];
			//std::cout << buffer[idx];
			idx++;
			if(buf[0] == '\n'){
				int type = nmea_parse(&parser, &buffer[0], (int)strlen(buffer), &info);
				idx = 0;
				switch (type){
					case GPGGA:
						msg_GPGGA->mutable_time()->set_year(0);
						msg_GPGGA->mutable_time()->set_mon(0);
						msg_GPGGA->mutable_time()->set_day(0);
						msg_GPGGA->mutable_time()->set_hour(info.utc.hour);
						msg_GPGGA->mutable_time()->set_min(info.utc.min);
						msg_GPGGA->mutable_time()->set_sec(info.utc.sec);
						msg_GPGGA->mutable_time()->set_hsec(info.utc.hsec);
						msg_GPGGA->mutable_pos()->set_lat(nmea_ndeg2degree(info.lat));
						msg_GPGGA->mutable_pos()->set_northsouth(info.northsouth);
						msg_GPGGA->mutable_pos()->set_lon(nmea_ndeg2degree(info.lon));
						msg_GPGGA->mutable_pos()->set_eastwest(info.eastwest);
						msg_GPGGA->set_sig(info.sig);
						msg_GPGGA->set_satinuse(info.satinfo.inuse);
						msg_GPGGA->set_hdop(info.HDOP);
						msg_GPGGA->set_elv(info.elv);
						msg_GPGGA->set_elv_units(info.elv_units);
						msg_GPGGA->set_diff(info.diff);
						msg_GPGGA->set_diff_units(info.diff_units);
						informer_GPGGA->publish(msg_GPGGA);
						break;
					case GPGSA:
						msg_GPGSA->set_fix_mode(info.fix_mode);
						msg_GPGSA->set_fix_type(info.fix);
						msg_GPGSA->set_pdop(info.PDOP);
						msg_GPGSA->set_hdop(info.HDOP);
						msg_GPGSA->set_vdop(info.VDOP);
						for(int i = 0; i < NMEA_MAXSAT; ++i){
							msg_GPGSA->add_sat_prn(info.satinfo.sat[i].id);
						}
						informer_GPGSA->publish(msg_GPGSA);
						msg_GPGSA->clear_sat_prn();
						break;
					case GPGSV:
						msg_GPGSV->set_pack_count(info.pack_count);
						msg_GPGSV->set_pack_index(info.pack_index);
						msg_GPGSV->set_sat_count(info.satinfo.inview);
						for(int i = 0; i < 4; ++i){
							msg_GPGSV->add_satellite();
							msg_GPGSV->mutable_satellite(i)->set_id(info.satinfo.sat[i].id);
							msg_GPGSV->mutable_satellite(i)->set_in_use(info.satinfo.sat[i].id);
							msg_GPGSV->mutable_satellite(i)->set_elv(info.satinfo.sat[i].id);
							msg_GPGSV->mutable_satellite(i)->set_azimuth(info.satinfo.sat[i].id);
							msg_GPGSV->mutable_satellite(i)->set_sig(info.satinfo.sat[i].id);
						}
						informer_GPGSV->publish(msg_GPGSV);
						msg_GPGSV->Clear();
						break;
					case GPRMC:
						msg_GPRMC->mutable_time()->set_year(0);
						msg_GPRMC->mutable_time()->set_mon(0);
						msg_GPRMC->mutable_time()->set_day(0);
						msg_GPRMC->mutable_time()->set_hour(info.utc.hour);
						msg_GPRMC->mutable_time()->set_min(info.utc.min);
						msg_GPRMC->mutable_time()->set_sec(info.utc.sec);
						msg_GPRMC->mutable_time()->set_hsec(info.utc.hsec);
						msg_GPRMC->set_status(info.status);
						msg_GPRMC->mutable_pos()->set_lat(nmea_ndeg2degree(info.lat));
						msg_GPRMC->mutable_pos()->set_northsouth(info.northsouth);
						msg_GPRMC->mutable_pos()->set_lon(nmea_ndeg2degree(info.lon));
						msg_GPRMC->mutable_pos()->set_eastwest(info.eastwest);
						msg_GPRMC->set_speed(info.speed);
						msg_GPRMC->set_direction(info.direction);
						msg_GPRMC->set_declination(info.declination);
						msg_GPRMC->set_declin_ew(info.declin_ew);
						msg_GPRMC->set_mode(info.mode);
						informer_GPRMC->publish(msg_GPRMC);
						break;
					case GPVTG:
						msg_GPVTG->set_dir(info.direction);
						msg_GPVTG->set_dir_t(info.dir_t);
						msg_GPVTG->set_dec(info.declination);
						msg_GPVTG->set_dec_m(info.declin_ew);
						msg_GPVTG->set_spn(info.spn);
						msg_GPVTG->set_spn_n(info.spn_n);
						msg_GPVTG->set_spk(info.speed);
						msg_GPVTG->set_spk_k(info.spk_k);
						informer_GPVTG->publish(msg_GPVTG);
						break;
					case GPGLL:
						msg_GPGLL->mutable_pos()->set_lat(nmea_ndeg2degree(info.lat));
						msg_GPGLL->mutable_pos()->set_northsouth(info.northsouth);
						msg_GPGLL->mutable_pos()->set_lon(nmea_ndeg2degree(info.lon));
						msg_GPGLL->mutable_pos()->set_eastwest(info.eastwest);
						msg_GPGLL->mutable_time()->set_year(0);
						msg_GPGLL->mutable_time()->set_mon(0);
						msg_GPGLL->mutable_time()->set_day(0);
						msg_GPGLL->mutable_time()->set_hour(info.utc.hour);
						msg_GPGLL->mutable_time()->set_min(info.utc.min);
						msg_GPGLL->mutable_time()->set_sec(info.utc.sec);
						msg_GPGLL->mutable_time()->set_hsec(info.utc.hsec);
						msg_GPGLL->set_active(info.mode);
						informer_GPGLL->publish(msg_GPGLL);
						break;
				}
			}
		}
	}
  }

  nmea_parser_destroy(&parser);
}
