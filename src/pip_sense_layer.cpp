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

//Include GPIO and SPI library for the Pi
#include <bcm2835.h>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

using std::string;
using std::list;
using std::map;
using std::pair;

#define MAX_PACKET_SIZE_READ		100

/* various debug levels */ 
#define DEBUG_BAD 1 
#define DEBUG_GOOD 5 
#define DEBUG_ALL  10

#define REQ_DROPPED 0xFC
#define REQ_PACKET 0xFF
#define REQ_REALIGN 0xFD
#define REQ_NULL 0x1

//The SPI rate in microseconds
#define SPI_RATE 60

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
	unsigned char dropped   : 8; //The number of packet that were dropped if the queue overflowed.
	unsigned int boardID    : 24;//Basestation ID
	unsigned int time       : 32;//Timestamp in quarter microseconds.
	unsigned int tagID      : 21;//Transmitter ID
	unsigned int parity     : 3; //Even parity check on the transmitter ID
	unsigned char rssi      : 8; //Received signal strength indicator
	unsigned char status    : 8; //The lower 7 bits contain the link quality indicator
	unsigned char data[20];      //The optional variable length data segment
} __attribute__((packed)) pip_packet_t;

//Bit banging SPI code
//TODO Migrate into own file

//GPIO 10 is MOSI, pin is 19
//GPIO  9 is MISO, pin is 21
//GPIO 11 is SCLK, pin is 23
#define MOSI RPI_GPIO_P1_19
#define MISO RPI_GPIO_P1_21
#define SCLK RPI_GPIO_P1_23

void setupSPI() {
	//Use bcm2835_gpio_fsel(uint8_t pin, uint8_t mode) to set pin modes
	//Modes are BCM2835_GPIO_FSEL_INPT for input
	//Modes are BCM2835_GPIO_FSEL_OUTP for output
	bcm2835_gpio_fsel(SCLK, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(MOSI, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(MISO, BCM2835_GPIO_FSEL_INPT);

	//Clear the two output pins
	bcm2835_gpio_clr(SCLK);
	bcm2835_gpio_clr(MOSI);
}

//Sends and reads a bit, delaying clock_period/2 milliseconds after writing and again after reading
//Going to change data on falling edges, so read on rising edges
uint8_t exchangeBit(uint8_t value, unsigned int clock_period) {
	//Go low and set value, then wait a period and go high
	//Delay half a clock cycle since we may have just gone high from a previous call
	bcm2835_delayMicroseconds(clock_period / 2);
	bcm2835_gpio_write(SCLK, LOW);
	//Change output on falling edge
	if (0 == value) {
		bcm2835_gpio_write(MOSI, LOW);
	}
	else {
		bcm2835_gpio_write(MOSI, HIGH);
	}
	bcm2835_delayMicroseconds(clock_period / 2);
	bcm2835_gpio_write(SCLK, HIGH);
	//Read data from the slave
	uint8_t bit = bcm2835_gpio_lev(MISO);
	return bit;
}

uint8_t exchangeByte(uint8_t byte, unsigned int clock_period) {
	uint8_t value = 0;
	for (int i = 7; i >= 0; --i) {
		if (HIGH == exchangeBit(0x1 & (byte >> i), clock_period)) {
			value |= 1 << i;
		}
	}
	return value;
}

int main(int ac, char** arg_vector) {
  bool offline = false;
  unsigned int pip_debug = 0;

  if ("offline" == std::string(arg_vector[ac-1])) {
    //Don't look at this last argument
    ac -= 1;
    offline = true;
	pip_debug = DEBUG_ALL;
  }

  if (ac != 3 and ac != 4 and ac != 5 ) {
    std::cerr<<"This program requires 2 arguments,"<<
      " the ip address and the port number of the aggregation server to send data to.\n";
    std::cerr<<"An optional third argument specifies the minimum RSS for a packet to be reported.\n";
    std::cerr<<"An optional forth argument specifies the debug level (1-10) \n";
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

  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  

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
			std::cout<<"Dropped "<<(unsigned int)dropped<<" packets over SPI interface.\n";
		}

		//Try to read some data. First see how long the next packet is
		exchangeByte(REQ_PACKET, SPI_RATE);
		uint8_t transferred = exchangeByte(REQ_NULL, SPI_RATE);

		//Verify that there is a packet to read
		if (0 < transferred and transferred <= MAX_PACKET_SIZE_READ) {
			std::cout<<"Reading packet of length "<<(unsigned int)transferred<<'\n';
			uint8_t buf[MAX_PACKET_SIZE_READ];
			memset(buf, 0, MAX_PACKET_SIZE_READ);	  
			//Get the packet
			//bcm2835_spi_transfernb((char*)buf, (char*)buf, (unsigned int)transferred);
			for (int i = 0; i < transferred; ++i) {
				buf[i] = exchangeByte(REQ_NULL, SPI_RATE);
			}

			//TODO FIXME Debugging received packet
			std::cout<<"Packet is: ";
			for (int i = 0; i < transferred; ++i) {
				std::cout<<'\t'<<std::hex<<(uint32_t)buf[i];
			}
			std::cout<<'\n';
			
			//Overlay the packet struct on top of the pointer to the rpip's message.
			pip_packet_t *pkt = (pip_packet_t *)buf;
			//Check to make sure this was a good packet.
			if (((pkt->rssi != (int) 0) and (pkt->status != 0))) {
				unsigned char* data = (unsigned char*)pkt;

				//Even parity check
				bool parity_failed = false;
				{
					unsigned char p1 = 0;
					unsigned char p2 = 0;
					unsigned char p3 = 0;
					unsigned long packet = ((unsigned int)data[9]  << 16) |
						((unsigned int)data[10] <<  8) |
						((unsigned int)data[11]);

					int i;
					/* XOR each group of 3 bytes until all of the 24 bits have been XORed. */
					for (i = 7; i >= 0; --i) {
						unsigned char triple = (packet >> (3 * i)) & 0x7;
						p1 ^= triple >> 2;
						p2 ^= (triple >> 1) & 0x1;
						p3 ^= triple & 0x1;
					}
					/* If the end result of the XORs is three 0 bits then even parity held,
					 * which suggests that the packet data is good. Otherwise there was a bit error. */
					if (p1 ==  0 && p2 == 0 && p3 == 0) {
						parity_failed = false;
					}
					else {
						parity_failed = true;
					}
				}
				if (true or not parity_failed) {
					//Now assemble a sample data variable and send it to the aggregation server.
					SampleData sd;
					//Calculate the tagID here instead of using be32toh since it is awkward to convert a
					//21 bit integer to 32 bits. Multiply by 8192 and 32 instead of shifting by 13 and 5
					//bits respectively to avoid endian issues with bit shifting.
					unsigned int netID = ((unsigned int)data[9] * 8192)  + ((unsigned int)data[10] * 32) +
						((unsigned int)data[11] >> 3);
					//We do not currently use the pip's local timestamp
					//unsigned long time = ntohl(pkt->time);
					unsigned long baseID = ntohl(pkt->boardID << 8);

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
					sd.sense_data = std::vector<unsigned char>(pkt->data, pkt->data+((unsigned char*)buf)[0]);
					sd.valid = true;

					if (pip_debug > DEBUG_GOOD) { 
						printf("pkt tx: %0x rx: %0lx rss: %0.2f data length %0x \n", 
								netID, baseID, sd.rss, pkt->ex_length);
					}

					if (pip_debug > DEBUG_BAD
							and 0 < pkt->dropped) { 
						std::cout<<"USB under-read, "<<(unsigned int)pkt->dropped<<" packets dropped.\n";
					}


					//Send the sample data as long as it meets the min RSS constraint
					if (true or sd.rss > min_rss) {
						//Send data to the aggregator if we are not in offline mode
						//Otherwise print out the packet
						if (not offline) {
							agg.send(sd);
						}
						else {

							if (0 < pkt->dropped) {
								std::cout<<"Dropped: "<<(unsigned int)(pkt->dropped)<<" packets."<<std::endl;
							}

							//TODO Add a flag to pring things out in hex
							bool use_hex = false;
							if (use_hex) {
								std::cout<<std::hex<<sd.rx_id<<"\t"<<std::dec<<world_model::getGRAILTime()<<'\t'<<std::hex<<sd.tx_id<<std::dec;
								//cout<<std::hex<<sd.rx_id<<"\t"<<std::dec<<sd.rx_timestamp<<'\t'<<std::hex<<sd.tx_id<<std::dec;
							}
							else {
								std::cout<<std::dec<<sd.rx_id<<"\t"<<sd.rx_timestamp<<'\t'<<sd.tx_id;
							}
							std::cout<<"\t0\t"<<sd.rss<<"\t0x00\tExtra:"<<sd.sense_data.size();
							for (auto I = sd.sense_data.begin(); I != sd.sense_data.end(); ++I) {
								std::cout<<'\t'<<(uint32_t)(*I);
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
			while (0 != exchangeByte(REQ_REALIGN, SPI_RATE));
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
  //bcm2835_spi_end();

  //Normal program halt
  return 0;
}


