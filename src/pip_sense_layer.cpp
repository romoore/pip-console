/*
 * Copyright (c) 2013 Bernhard Firner and Rutgers University
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 * or visit http://www.gnu.org/licenses/gpl-2.0.html
 */

/*******************************************************************************
 * @file pip_sense_layer.cpp
 * Collect data from a PIP receiver connected via SPI to a Raspberry Pi. Forward
 * that data to an aggregator.
 *
 * @author Bernhard Firner
 ******************************************************************************/

//These includes need to come first because of the macro defining INT64_MAX
//TODO FIXME Are some of the old C-style includes breaking this macro?
#include <owl/sensor_connection.hpp>
#include <owl/world_model_protocol.hpp>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

//#include <fcntl.h>
//#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <time.h>

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <stdexcept>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

#include "spi.h"

using std::string;
using std::list;
using std::map;
using std::pair;

#define MAX_PACKET_SIZE_READ		100

/* various debug levels */ 
#define DEBUG_BAD 1 
#define DEBUG_GOOD 5 
#define DEBUG_ALL  10

//Global variable for the signal handler.
bool killed = false;
//Signal handler.
void handler(int signal) {
  psignal( signal, "Received signal ");
  if (killed) {
    std::cerr<<"Aborting.\n";
    // This is the second time we've received the interrupt, so just exit.
    exit(-1);
  }
  std::cerr<<"Shutting down...\n";
  killed = true;
}


float toFloat(unsigned char* pipFloat) {
    return ((float)pipFloat[0] * 0x100 + (float)pipFloat[1] + (float)pipFloat[2] / (float)0x100);
}

//Constants for the CRC/LQI field
#define RSSI_OFFSET 78
#define CRC_OK 0x80



//PIP 3 Byte ID packet structure with variable data segment.
//3 Byte receiver ID, 21 bit transmitter id, 3 bits of parity plus up to 20 bytes of extra data.
typedef struct {
	unsigned char ex_length : 8; //Length of data in the optional data portion
	unsigned int boardID    : 32;//Basestation ID
	unsigned int time       : 32;//Timestamp in quarter microseconds.
	unsigned int tagID      : 24;//Transmitter ID
	unsigned char rssi      : 8; //Received signal strength indicator
	unsigned char status    : 8; //The lower 7 bits contain the link quality indicator
	unsigned char data[20];      //The optional variable length data segment
} __attribute__((packed)) pip_packet_t;

int main(int ac, char** arg_vector) {
  bool offline = false;
  unsigned int pip_debug = 0;

  if ("offline" == std::string(arg_vector[ac-1])) {
    //Don't look at this last argument
    ac -= 1;
    offline = true;
  }

  if (ac != 3 and ac != 4 and ac != 5 ) {
    std::cerr<<"This program requires 2 arguments,"<<
      " the ip address and the port number of the aggregation server to send data to.\n";
    std::cerr<<"An optional third argument specifies the minimum RSS for a packet to be reported.\n";
    std::cerr<<"An optional fourth argument specifies the debug level (1-10) \n";
    std::cerr<<"If 'offline' is given as the last argument then this program will not connect to the aggregator and will instead print packets to the screen.\n";
    return 0;
  }

  //Get the ip address and ports of the aggregation server
  std::string server_ip(arg_vector[1]);
  int server_port = atoi(arg_vector[2]);

  float min_rss = -600.0;
  if (ac > 3) {
    min_rss = atof(arg_vector[3]);
    std::cout<<"Using min RSS "<<min_rss<<'\n';
  }

  if (ac > 4) {
    pip_debug = atoi(arg_vector[4]);
    if ( (pip_debug >= 1) && (pip_debug <= 10)) { 
      std::cout<<"Using debug level "<<pip_debug<<'\n';
    } else {
      std::cout<<"bad debug level "<<pip_debug<<'\n';   
      pip_debug = 0;      
    }
  }

  //TODO Add a flag to pring things out in hex
  bool use_hex = false;

  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  
  signal(SIGKILL, handler);  

  //Initialize SPI connection
  if (!bcm2835_init()) {
      std::cerr<<"Error initializing PI bus! Aborting.\n";
	  return 1;
  }

  //Set up the SPI pins
  setupSPI();

  while (not killed) {
    SensorConnection agg(server_ip, server_port);

    //A try/catch block is set up to handle exceptions during data transfer
    try {
      while ((offline or agg) and not killed) {
		//See if we dropped any packets
		exchangeByte(REQ_DROPPED, SPI_RATE);
		uint8_t dropped = exchangeByte(REQ_NULL, SPI_RATE);
		if (0 < dropped) {
			if (pip_debug > DEBUG_BAD) {
				std::cout<<"SPI under-read, "<<(unsigned int)dropped<<" packets dropped.\n";
			}
		}

		//Try to read some data. First see how long the next packet is
		exchangeByte(REQ_PACKET, SPI_RATE);
		uint8_t transferred = exchangeByte(REQ_NULL, SPI_RATE);

		//Verify that there is a packet to read
		if (0 < transferred and transferred <= MAX_PACKET_SIZE_READ) {
			uint8_t buf[MAX_PACKET_SIZE_READ];
			memset(buf, 0, MAX_PACKET_SIZE_READ);	  
			//Get the packet
			//bcm2835_spi_transfernb((char*)buf, (char*)buf, (unsigned int)transferred);
			for (int i = 0; i < transferred; ++i) {
				buf[i] = exchangeByte(REQ_NULL, SPI_RATE);
			}

			//TODO FIXME Debugging received packet
			/*
			std::cout<<"Packet is: ";
			for (int i = 0; i < transferred; ++i) {
				std::cout<<'\t'<<std::hex<<(uint32_t)buf[i];
			}
			std::cout<<'\n';
			*/
			
			//Overlay the packet struct on top of the pointer to the rpip's message.
			pip_packet_t *pkt = (pip_packet_t *)buf;
			//Check to make sure this was a good packet.
			if (((pkt->rssi != (int) 0) and (pkt->status != 0))) {
				//Process packet if its CRC is OK
				if (pkt->status & CRC_OK) {
					//Now assemble a sample data variable and send it to the aggregation server.
					SampleData sd;
					//Get the transmitter and receiver IDs, converting endianness
					//unsigned int netID = ntohl(pkt->tagID << 8);
					unsigned int netID = (buf[9]<<16) + (buf[10]<<8) + buf[11];
					unsigned long baseID = ntohl(pkt->boardID);
					//We do not currently use the pip's local timestamp
					//unsigned long time = ntohl(pkt->time);

					//The physical layer of a pipsqueak device is 1
					sd.physical_layer = 1;
					sd.tx_id = netID;
					sd.rx_id = baseID;
					//Set this to the real timestamp, milliseconds since 1970
					timeval tval;
					gettimeofday(&tval, NULL);
					sd.rx_timestamp = tval.tv_sec*1000 + tval.tv_usec/1000;
					//Convert from one byte value to a float for receive signal
					//strength as described in the TI/chipcon Design Note DN505 on cc1100
					sd.rss = ( (pkt->rssi) >= 128 ? (signed int)(pkt->rssi-256)/2.0 : (pkt->rssi)/2.0) - RSSI_OFFSET;
					sd.sense_data = std::vector<unsigned char>(pkt->data, pkt->data+pkt->ex_length);
					sd.valid = true;

					if (pip_debug > DEBUG_GOOD) { 
						printf("pkt tx: %0x rx: %0lx rss: %0.2f data length %0x \n", 
								netID, baseID, sd.rss, pkt->ex_length);
					}

					//Send the sample data as long as it meets the min RSS constraint
					if (sd.rss > min_rss) {
						//Send data to the aggregator if we are not in offline mode
						//Otherwise print out the packet
						if (not offline) {
							agg.send(sd);
						}
						else {

							//Print out the packet (in hex or decimal)
							if (use_hex) {
								std::cout<<std::hex<<sd.rx_id<<"\t"<<std::dec<<sd.rx_timestamp<<'\t'<<std::hex<<sd.tx_id<<std::dec;
							}
							else {
								std::cout<<std::dec<<baseID<<"\t"<<sd.rx_timestamp<<'\t'<<std::dec<<netID;
							}
							std::cout<<"\t0\t"<<sd.rss<<"\t0x00\tExtra:"<<sd.sense_data.size();
							for (auto I = sd.sense_data.begin(); I != sd.sense_data.end(); ++I) {
								std::cout<<'\t'<<std::hex<<(uint32_t)(*I);
							}
							std::cout<<std::endl;
						}
					}
				}
			}
		}
		//Size seems too large, read from SPI to empty the buffer
		else if (transferred >= MAX_PACKET_SIZE_READ) {
			std::cerr<<"SPI seems mis-aligned (got packet of length "<<(unsigned int)transferred<<"), trying to realign\n";
			while (0 != exchangeByte(REQ_REALIGN, SPI_RATE) and not killed);
			std::cerr<<"SPI re-aligned\n";
		}
		//No data, so try to sleep to reduce CPU consumption
		else {
			usleep(100);
		}
      }
    }
    catch (std::runtime_error& re) {
      std::cerr<<"SPI error: "<<re.what()<<'\n';
    }
    catch (std::exception& e) {
      std::cerr<<"SPI error: "<<e.what()<<'\n';
    }
    //Try to reconnect to the server after losing the connection.
    //Sleep a little bit, then try connecting to the server again.
    usleep(1000000);
  }
  std::cerr<<"Exiting\n";

  //Turn off SPI pins
  tearDownSPI();

  //Normal program halt
  return 0;
}


