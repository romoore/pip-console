/*
 * Copyright (c) 2012 Bernhard Firner and Rutgers University
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
 * Collect data from a PIP receiver connected via USB to this device. Forward
 * that data to an aggregator.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <usb.h>
#include <errno.h>
#include <sys/time.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <time.h>

//TODO Create lib-cppsensor so that sockets don't need to be handled here.
#include <owl/sensor_connection.hpp>

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <stdexcept>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

using std::string;
using std::list;
using std::map;
using std::pair;

#define MAX_WRITE_PKTS		0x01

#define FT_READ_MSG			0x00
#define FT_WRITE_MSG		0x01
#define FT_READ_ACK			0x02

#define FT_MSG_SIZE			0x03

#define MAX_PACKET_SIZE_READ		(64 *1024 )
#define MAX_PACKET_SIZE_WRITE		512

typedef unsigned int frequency;
typedef unsigned char bsid;
typedef unsigned char rating;

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

/* #defines of the commands to the pipsqueak tag */
#define LM_PING (0x11)
#define LM_PONG (0x12)
#define LM_GET_NEXT_PACKET (0x13)
#define LM_RETURNED_PACKET (0x14)
#define LM_NULL_PACKET (0x15)

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

//USB PIPs' vendor ID and strings
const char *silicon_labs_s = "Silicon Labs\0";
const char *serial_num_s = "1234\0";
const int PACKET_LEN = 13;

//Map of usb devices in use, accessed by the USB device number
map<u_int8_t, bool> in_use;
//0 for 2.X tags, 1 for GPIP
map<usb_dev_handle*, uint8_t> versions;

#define SILICON_LABS_VENDOR  ((unsigned short) (0x10C4))
#define SILICON_LABS_PIPPROD ((unsigned char) (0x03))

#define TI_LABS_VENDOR  ((unsigned short) (0x2047))
#define TI_LABS_PIPPROD ((unsigned short) (0x0300))

void attachPIPs(list<usb_dev_handle*> &pip_devs) {
  struct usb_bus *bus = NULL; 
  struct usb_device *dev = NULL;

  /* Slot numbers used to differentiate multiple PIP USB connections. */
  int slot = 0;

  /* these loops crawl the whole USB tree */
  for (bus = usb_busses; bus != NULL; bus = bus->next) {
    for (dev = bus->devices; dev != NULL; dev = dev->next) {
      uint8_t version = 0;

      int found_manu, found_serial, found_prod;
      found_manu = found_prod = found_serial = 0;

      if ((unsigned short) dev->descriptor.idVendor ==  (unsigned short) TI_LABS_VENDOR)
      {
	  found_manu = 1;
      	if ((unsigned short) dev->descriptor.idProduct == (unsigned short) TI_LABS_PIPPROD)
        {	
	   found_prod = 1; 
	   version = 1;
	}else{/* ignore */}
      }
      else if ((unsigned short) dev->descriptor.idVendor ==  (unsigned short) SILICON_LABS_VENDOR)
      {
	found_manu = 1;      	
	if ((unsigned short) dev->descriptor.idProduct == (unsigned short) SILICON_LABS_PIPPROD)
        {
	    found_prod = 1; 
	    version = 0;
	}

      }else{/* Ignore */}

      //If this is a pipsqueak device that is not already opened try opening it.
      if ( (found_manu == 1) && (found_prod == 1) && not in_use[dev->devnum] ) {
        ++slot;
        std::cerr<<"Connected to USB Tag Reader.\n";
        usb_dev_handle* new_handle = usb_open(dev);

        if (!new_handle) {
          std::cout<<"Failed to open pipsqueak.\n";
        }
        else {
          //Add the new device to the pip device list.
          pip_devs.push_back(new_handle);
          std::cout<<"New pipsqueak opened.\n";
          in_use[dev->devnum] = true;
          versions[new_handle] = version;

          int retval = usb_set_configuration(pip_devs.back(), 1);
          if (retval < 0 ) { 
            printf("Setting configuration to 1 failed %d \n",retval);
          }

          //Retry claiming the device up to two times.
          int retries = 2;

          int interface_num = 0;
          while ((retval = usb_claim_interface(pip_devs.back(), interface_num)) && retries-- > 0) {
            if (retval == -ENOMEM) {
              std::cerr<<"usb_claim_interface failed try "<<retries<<": -ENOMEM\n";
              std::cerr<<"This program is being run without permission to open usb devices - aborting.\n";
              retries = 0;
              //This failure indicates that we do not have permission to open the usb device.
              return;
            } else if (retval == -EBUSY) {
#if LIBUSB_HAS_GET_DRIVER_NP
              char drivername[256];
              if (usb_get_driver_np(new_handle, 0, drivername, sizeof(drivername))) {
                std::cerr<<"usb_get_driver_np failed\n";
                retries = 0;
              }
              else {
                std::cerr<<"kernel driver '"<<drivername<<"' is bound to interface 0\n";
#else
                std::cerr<<"kernel driver is bound to interface 0\n";
#endif /* LIBUSB_HAS_GET_DRIVER_NP */

#if LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP
                if (usb_detach_kernel_driver_np(new_handle, 0)) {
                  std::cerr<<"usb_detach_kernel_driver_np failed\n";
                  retries = 0;
                }
                std::cerr<<"kernel driver successfully detached\n";
#else
                retries = 0;
#endif /* LIBUSB_HAS_DETACH_KERNEL_DRIVER_NP */
#if LIBUSB_HAS_GET_DRIVER_NP
              }
#endif /* LIBUSB_HAS_GET_DRIVER_NP */
            } else {
              std::cerr<<"usb_claim_interface failed: "<<retval<<" tries "<<retries<<"\n";
            }
          }
        }
      }
      if (!dev->config) { 
        std::cout<<"Couldn't retrieve descriptors\n"; 
      }
    }
  }
}

int main(int ac, char** arg_vector) {
  if (ac != 3 and ac != 4) {
    std::cerr<<"This program requires 2 arguments,"<<
      " the ip address and the port number of the aggregation server to send data to.\n";
    std::cerr<<"An optional third argument specifies the minimum RSS for a packet to be reported.\n";
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

  //Now connect to pip devices and send their packet data to the aggregation server.
  unsigned char msg[128];
  char* signed_msg = (char*)msg;
  unsigned char buf[MAX_PACKET_SIZE_READ];
  char* signed_buf = (char*)buf;
  list<usb_dev_handle*> pip_devs;
  //Set up the USB
  usb_init(); 
  usb_find_busses(); 
  usb_find_devices(); 

  //Attach new pip devices.
  attachPIPs(pip_devs);

  while (not killed) {
    bool connected = false;

    SensorConnection agg(server_ip, server_port);

    //A try/catch block is set up to handle exception during quitting.
    try {
      while (agg and not killed) {
        //Check for new USB devices by checking if new devices were added or force a check every 3 seconds.
        if (0 < usb_find_busses() + usb_find_devices()) {
          attachPIPs(pip_devs);
          //Remove any duplicate devices
          pip_devs.sort();
          pip_devs.unique();
        }
        if (pip_devs.size() > 0) {
          for (list<usb_dev_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
            //A pip can fail up to two times in a row if this is the first time querying it.
            //If the pip fails after three retries then this pip usb_dev_handle is no longer
            //valid, probably because the pip was removed from the USB.
            int retries_left = 3;
            int retval = -1;
            while (retval < 0 and retries_left > 0) {
              // Request the next packet from the pip
              msg[0] = LM_GET_NEXT_PACKET;
              retval = usb_bulk_write(*I, 2, signed_msg, 1, 100); 
              memset(buf, 0, MAX_PACKET_SIZE_READ);	  

              //Allow up to 20 extra bytes of sensor data beyond the normal packet length.
              if(0 == versions[*I])              
                retval = usb_bulk_read(*I, 0x81, signed_buf+1, PACKET_LEN+20, 100);
              else
                retval = usb_bulk_read(*I, 0x82, signed_buf+1, PACKET_LEN+20, 100);
              //Fill in the length of the extra portion of the packet
              signed_buf[0] = retval - PACKET_LEN;
              --retries_left;
            }
            //If the pip fails 3 times in a row then it was probably disconnected.
            if (retval < 0) {
              usb_reset(*I);
              usb_close(*I); 
              *I = NULL;
            }
            //If the length of the message is equal to or greater than PACKET_LEN then this is a data packet.
            else if (PACKET_LEN <= retval) {
              //Overlay the packet struct on top of the pointer to the pip's message.
              pip_packet_t *pkt = (pip_packet_t *)buf;

              //Check to make sure this was a good packet.
              if ((pkt->rssi != (int) 0) and (pkt->status != 0)) {
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
                if (not parity_failed) {
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
                  sd.sense_data = std::vector<unsigned char>(pkt->data, pkt->data+signed_buf[0]);
                  sd.valid = true;
                  //Send the sample data as long as it meets the min RSS constraint
                  if (sd.rss > min_rss) {
                    agg.send(sd);
                  }
                }
              }
            }
          }
        }
        //Clear dead connections
        pip_devs.remove(NULL);
      }
    }
    catch (std::runtime_error& re) {
      std::cerr<<"USB sensor layer error: "<<re.what()<<'\n';
    }
    catch (std::exception& e) {
      std::cerr<<"USB sensor layer error: "<<e.what()<<'\n';
    }
    //Try to reconnect to the server after losing the connection.
    //Sleep a little bit, then try connecting to the server again.
    usleep(1000000);
  }
  std::cerr<<"Exiting\n";
  //Clean up the pip connections before exiting.
  for (list<usb_dev_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
    usb_reset(*I);
    usb_close (*I); 
  }
}


