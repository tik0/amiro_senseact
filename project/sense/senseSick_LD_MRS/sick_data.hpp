/*
 * Sick_Data.hpp
 *
 *  Created on: 10.12.2014
 *      Author: rene
 */

#ifndef _SICK_DATA_HPP_
#define _SICK_DATA_HPP_

// RSB
#include <rsb/Event.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

// RST"../../includes"
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// Boost
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <math.h>       /* sin */

#include <SickLdMRS4002.pb.h>

#define _USE_MATH_DEFINES /* M_Pi */

//#define TCP_SERVER_IP 	"192.168.100.150"
//#define TCP_SERVER_PORT	12002
//#define IP_ADDRESS		"192.168.100.151"
#define	ADAPTER_INDEX	2
#define MAXBUFFER 15000

using namespace rst::claas;

const uint16_t MAXMEASPOINTS = 1000;			// Maximale Anzahl Messpunkte

const uint32_t HEADERLENGTH     	= 24;      	//
const uint32_t BODYHEADERLENGTH 	= 44;      	//
const uint32_t MAXBUFSIZE       	= 262144*2;	// maximale Bufferlänge
const uint32_t SCANPOINTLENGTH  	= 10;       	// in Bytes


// Abspeichern der streaming Information aus dem Socket
char data_buffer[MAXBUFSIZE];
char tmp_buffer[MAXBUFSIZE];
char scn_buffer[MAXBUFSIZE];

int global = 0;

//onTcpReceive
uint32_t data_offset; // Speicherung der Buffergröße
uint32_t main_size;
uint32_t nr;

//Sick_LD_MRS400102 sick_LD_MRD400102;
boost::shared_ptr<SickLdMRS400102> sickLDMRS4002 = boost::make_shared<SickLdMRS400102>();

// Find the Magic Word
int indexOfMagicWord(char buffer[], uint32_t size)
{
	int idx = -1;
	for (uint32_t i = 0; i < size; i++)
	{
		if(buffer[i] == (char)0xAF && buffer[i+1] == (char)0xFE && buffer[i+2] == (char)0xC0 && buffer[i+3] == (char)0xC2)
			return i;
	}
	return idx;
}

//calculate DataSize
uint32_t getDataSize(uint32_t magic_offset, char buffer[])
{
	uint32_t uint32 = 0;

	uint32  = (uint32_t) ((uint32_t)buffer[magic_offset+ 8] & 0x000000FF) << 24;
	uint32 |= (uint32_t) ((uint32_t)buffer[magic_offset+ 9] & 0x000000FF) << 16;
	uint32 |= (uint32_t) ((uint32_t)buffer[magic_offset+10] & 0x000000FF) <<  8;
	uint32 |= (uint32_t) ((uint32_t)buffer[magic_offset+11] & 0x000000FF);
	return uint32;
}
//
uint32_t getDataType(uint32_t magic_offset, char buffer[])
{
    // big endian
    uint16_t uint16 = 0;
    uint16  = ((uint16_t) buffer[magic_offset + 14] & 0x000000FF) << 8;
    uint16 |= ((uint16_t) buffer[magic_offset + 15] & 0x000000FF);

    return uint16;
}

uint64_t getTimeStamp (char buffer[])
{
    // big endian
    uint64_t timestamp = 0;

    timestamp  = ((uint64_t) ((uint16_t)buffer[16] & 0x000000FF)) << 56;
    timestamp |= ((uint64_t) ((uint16_t)buffer[17] & 0x000000FF)) << 48;
    timestamp |= ((uint64_t) ((uint16_t)buffer[18] & 0x000000FF)) << 40;
    timestamp |= ((uint64_t) ((uint16_t)buffer[19] & 0x000000FF)) << 32;
    timestamp |= ((uint64_t) ((uint16_t)buffer[20] & 0x000000FF)) << 24;
    timestamp |= ((uint64_t) ((uint16_t)buffer[21] & 0x000000FF)) << 16;
    timestamp |= ((uint64_t) ((uint16_t)buffer[22] & 0x000000FF)) <<  8;
    timestamp |= ((uint64_t) ((uint16_t)buffer[23] & 0x000000FF));

    return timestamp;
}

uint16_t getArrayUInt16(char buffer[], uint32_t offset)
{
    // little endian
    uint16_t uint16 = 0;

    uint16  = ((uint16_t) buffer[offset  ] & 0x000000FF);
    uint16 |= ((uint16_t) buffer[offset+1] & 0x000000FF) << 8;
    return uint16;
}

int getArrayInt16(char buffer[], uint32_t offset)
{
    // little endian
	uint16_t int16 = 0;
    int16  = (int) ((uint16_t)buffer[offset  ] & 0x000000FF);
    int16 |= (int) ((uint16_t)buffer[offset+1] & 0x000000FF) << 8;
    return int16;
}

uint64_t getArrayUInt64(char buffer[], uint32_t offset)
{
    // little endian
	uint64_t uint64 = 0;
    uint64  = (uint64_t) ((uint16_t)buffer[offset  ] & 0x000000FF);
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+1] & 0x000000FF) <<  8;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+2] & 0x000000FF) << 16;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+3] & 0x000000FF) << 24;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+4] & 0x000000FF) << 32;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+5] & 0x000000FF) << 40;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+6] & 0x000000FF) << 48;
    uint64 |= (uint64_t) ((uint16_t)buffer[offset+7] & 0x000000FF) << 56;
    return uint64;
}

void setScanData (char buffer[], uint32_t size, uint32_t dataType, rsb::Informer<SickLdMRS400102>::Ptr informer)
{
    int i = 0;
    //double angle = 0;
    uint32_t offset = 0;

    uint64_t  	timestamp;							// NTP Zeit
    uint16_t   	scan_counter;						// Nummer Messung
    uint16_t   	sensor_state;						// Sensorstatus
    uint16_t   	sync_phase_offset;					// Phasenoffset in Winkelschritten zwischen dem Synchronationssignal und dem Synchronationswinkel
    uint64_t  	start_time_stamp;					// Startzeit Messung (2^(1/32) Sekunden)
    uint64_t  	stop_time_stamp;					// Endzeit Messung (2^(1/32) Sekunden)
    uint16_t   	steps_per_rotation;					// Winkelschritte pro Scannerrotation
    int    		start_angle;						// Startwinkel
    int    		end_angle;							// Endwinkel
    uint16_t   	num_scan_points;					// Anzahl Messpunkte
    //uint16_t	the_scan_time;


    /****************************** Header auslesen *******************************/

    timestamp        = getTimeStamp(buffer);                       // Sendezeit auslesen
    //the_scan_time = (uint64_t)((timestamp >> 32) % 86400);
    scan_counter      	= getArrayUInt16(buffer, HEADERLENGTH		);  // Nummer Messung
    sensor_state      	= getArrayUInt16(buffer, HEADERLENGTH +  2	);  // Sensorstatus
    sync_phase_offset  	= getArrayUInt16(buffer, HEADERLENGTH +  4	);  // Phasenoffset
    start_time_stamp   	= getArrayUInt64(buffer, HEADERLENGTH +  6	);  // Startzeit Messung
    stop_time_stamp    	= getArrayUInt64(buffer, HEADERLENGTH + 14	);  // Endzeit Messung
    steps_per_rotation 	= getArrayUInt16(buffer, HEADERLENGTH + 22	);  // Winkelschritte pro Scannerrotation
    start_angle 		= getArrayInt16(buffer,  HEADERLENGTH + 24	);  // Startwinkel
    end_angle         	= getArrayInt16(buffer,  HEADERLENGTH + 26	);  // Endwinkel
    num_scan_points    	= getArrayUInt16(buffer, HEADERLENGTH + 28	);  // Anzahl Messpunkte

    /***************************** Messpunkte auslesen *****************************/

    //Set Sick Header, dummy data just for test

    sickLDMRS4002->set_magicword(0xAFFEC0C2);
    sickLDMRS4002->set_sizeprevmsg(0);
    sickLDMRS4002->set_sizemsg(0);
	//sick_LD_MRD400102->set_has_reserved
    sickLDMRS4002->set_deviceid(42);
    sickLDMRS4002->set_datatype(0x2202);
    sickLDMRS4002->set_timemsgsent(timestamp);
	//Set Measured Data
    sickLDMRS4002->set_measurenumber(scan_counter);
    sickLDMRS4002->set_sensorstatus(sensor_state);
    sickLDMRS4002->set_synchrophase(sync_phase_offset);
    sickLDMRS4002->set_timestartmeasure(start_time_stamp);
    sickLDMRS4002->set_timeendmeasure(stop_time_stamp);
    sickLDMRS4002->set_angularstepsperrotation(steps_per_rotation);
    sickLDMRS4002->set_startangle(start_angle);
    sickLDMRS4002->set_endangle(end_angle);
    sickLDMRS4002->set_numbermeasuredpoints(num_scan_points);

    // Ist die vollständige Messung enthalten?

    if (num_scan_points * SCANPOINTLENGTH + HEADERLENGTH + BODYHEADERLENGTH == size && num_scan_points <= MAXMEASPOINTS && num_scan_points >=0)
    {
        // berechne Offset für Messpunkte
        offset = HEADERLENGTH + BODYHEADERLENGTH;
        for (i = 0; i < num_scan_points; i++)
        {
        	int32_t layer = (buffer[offset] & 0x0F);
			sickLDMRS4002->add_layer(layer);
			sickLDMRS4002->add_echo((buffer[offset] & 0x000000F0) >> 4);
			sickLDMRS4002->add_flags(buffer[offset + 1]);

			sickLDMRS4002->add_horizontalangle(getArrayInt16(buffer, offset + 2));
			uint32_t radial_distance = getArrayUInt16(buffer, offset + 4);
			sickLDMRS4002->add_radialdistance(radial_distance);
			sickLDMRS4002->add_echopulsewidth(getArrayUInt16(buffer, offset + 6));

            //Add scanpoints to the sick_LD_MRS400102 struct, so they can be published
            /*uint64_t va = 0;
			memcpy(&va, &buffer[offset],8);
             */
            offset += SCANPOINTLENGTH;
        }

        informer->publish(sickLDMRS4002);
        //Clear the scanned points after a whole data set was transmitted
        sickLDMRS4002->clear_layer();
		sickLDMRS4002->clear_echo();
		sickLDMRS4002->clear_flags();
		sickLDMRS4002->clear_horizontalangle();
		sickLDMRS4002->clear_radialdistance();
		sickLDMRS4002->clear_echopulsewidth();
    }
}

void onTcpReceive(char buffer[], int size, rsb::Informer<SickLdMRS400102>::Ptr informer)
{
	// Konstanten für die Datenverarbeitung in onTcpReceive
	const uint32_t SICKTYPE          = 0x2202;  	// Sick Datenformat
	const uint32_t ERRORTYPE         = 0x2030;  	// Fehler
	const uint32_t CMDTYPE           = 0x2020;  	//
	//const uint32_t SENDCMDTYPE       = 0x2010;  	//

	int   		magic_offset       	= -1;
	uint32_t 	data_size          	= 0;
	uint32_t  	data_type          	= 0;
	if (size > 0)
	{

	  // kopiere in den großen Buffer
	  memcpy(&data_buffer[data_offset], &buffer[0], size);
	  data_offset += size;

	  do {
		  magic_offset = indexOfMagicWord(data_buffer, data_offset);

		  if (magic_offset >= 0)
		  {
			  // Ist der gesamte Header in den Daten vorhanden?
			  if(data_offset >= magic_offset + HEADERLENGTH)
			  {
				  // Header auswerten
				  data_size = getDataSize(magic_offset, data_buffer);

					  data_type = getDataType(magic_offset, data_buffer);

				  // berechne gesamt Länge des Datensatzes inkl. Header
				  main_size = HEADERLENGTH + data_size;

				  // beinhaltet der data_buffer eine gesamte Botschaft?
				  if (data_offset >= magic_offset + main_size)
				  {
					  // kopiere kompletten data_buffer in einen tmpBuffer
					  memcpy(&tmp_buffer, &data_buffer, data_offset);
					  // kopiere eine Messung vom data_buffer in einen buffer
					  memcpy(&scn_buffer, &data_buffer[magic_offset], main_size);
					  // kopiere den Rest aus dem tmpBuffer wieder in den data_buffer
					  memcpy(&data_buffer,&tmp_buffer[magic_offset + main_size], data_offset - magic_offset - main_size);

					  // Offset berechnen
					  data_offset -= (magic_offset + main_size);

					  switch(data_type)
					  {
						  case SICKTYPE:
							  // werte die Sensordaten aus
							  setScanData(scn_buffer, main_size, data_type, informer);
							  break;
						  case CMDTYPE:
						  case ERRORTYPE:
						  default:
							  break;
					  }
				  }
				  else
				  {
					magic_offset = -1;
				  }
			  }
			  else
			  {
				magic_offset = -1;
			  }
		  }

	  } while(magic_offset >= 0);
	}
	nr++;
}
#endif /* SICK_DATA_HPP_ */
