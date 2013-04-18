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
#include <libusb-1.0/libusb.h>
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

/* various debug levels */ 
#define DEBUG_GOOD 5 
#define DEBUG_ALL  10
unsigned int pip_debug ; 

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
map<int, bool> in_use;
//0 for 2.X tags, 1 for GPIP
#define OLD_PIP 0
#define GPIP 1
#define NOT_PIP -1
map<struct libusb_device_handle*, int8_t> versions;

//The 8051 PIP
#define SILICON_LABS_VENDOR  ((unsigned short) (0x10C4))
#define SILICON_LABS_PIPPROD ((unsigned char) (0x03))

//The MSP430 PIP
#define TI_LABS_VENDOR  ((unsigned short) (0x2047))
#define TI_LABS_PIPPROD ((unsigned short) (0x0300))

void attachPIPs(list<libusb_device_handle*> &pip_devs) {
  //Keep track of the count of USB devices. Don't check if this doesn't change.
  //static int last_usb_count = 0;
  //TODO FIXME Try out that optimization (skipping the check) if this is slow
  //An array of pointers to usb devices.
  libusb_device **devices = NULL;

  /* Slot numbers used to differentiate multiple PIP USB connections. */
  int slot = 0;

  //Get the device list
  ssize_t count = libusb_get_device_list(NULL, &devices);

  //Scan for new pips
  for (int dev_idx = 0; dev_idx < count; ++dev_idx) {
    int8_t version = NOT_PIP;
    libusb_device* dev = devices[dev_idx];
    libusb_device_descriptor desc;
    if (0 >= libusb_get_device_descriptor(dev, &desc)) {

      if (((unsigned short) desc.idVendor ==  (unsigned short) TI_LABS_VENDOR) and
          ((unsigned short) desc.idProduct == (unsigned short) TI_LABS_PIPPROD)) {
        version = GPIP;
      }
      else if (((unsigned short) desc.idVendor ==  (unsigned short) SILICON_LABS_VENDOR) and
          ((unsigned short) desc.idProduct == (unsigned short) SILICON_LABS_PIPPROD)) {
        version = OLD_PIP;
      }
      //Make the device number a combination of bus number and the address on the bus
      int device_num = 0x100 * libusb_get_bus_number(dev) + libusb_get_device_address(dev);

      //See if we found a pip that is not already open
      if (NOT_PIP != version && not in_use[device_num]) {
        ++slot;
        std::cerr<<"Connected to USB Tag Reader.\n";
        libusb_device_handle* new_handle;
        int err = libusb_open(dev, &new_handle);

        if (0 != err) {
          if (LIBUSB_ERROR_ACCESS == err) {
            std::cout<<"Insufficient permission to open reader (try sudo).\n";
          }
        }
        //Otherwise getting a handle was successful
        else {
          //Reset the device before trying to use it
          if (0 == libusb_reset_device(new_handle)) {
            //Add the new device to the pip device list.
            pip_devs.push_back(new_handle);
            std::cout<<"New pipsqueak opened.\n";
            in_use[device_num] = true;
            versions[new_handle] = version;

            int retval = libusb_set_configuration(pip_devs.back(), 1);
            if (0 != retval ) { 
              printf("Setting configuration to 1 failed with error number %d \n",retval);
            }
            else {
              int interface_num = 0;

              //Detach the kernel driver on linux
              if (libusb_kernel_driver_active(pip_devs.back(), interface_num)) {
                libusb_detach_kernel_driver(pip_devs.back(), interface_num);
              }
              //Retry claiming the device up to two times.
              int retries = 2;

              while ((retval = libusb_claim_interface(pip_devs.back(), interface_num)) && retries-- > 0) {
                ;
              }
              //int alt_setting = 0;
              //libusb_set_interface_alt_setting(pip_devs.back(), interface_num, alt_setting);
              if (0 == retries) {
                std::cerr<<"usb_claim_interface failed\n";
              }
            }
          }
        }
      }
    }
  }

  //Free the device list
  libusb_free_device_list(devices, true);
}

int main(int ac, char** arg_vector) {
  if (ac != 3 and ac != 4 and ac != 5 ) {
    std::cerr<<"This program requires 2 arguments,"<<
      " the ip address and the port number of the aggregation server to send data to.\n";
    std::cerr<<"An optional third argument specifies the minimum RSS for a packet to be reported.\n";
    std::cerr<<"An optional forth argument specifies the debug level (1-10) \n";
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
      std::cout<<"Using debug level RSS "<<pip_debug<<'\n';
    } else {
      std::cout<<"bad debug level "<<pip_debug<<'\n';   
      pip_debug = 0;      
    }
  }

  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  

  //Now connect to pip devices and send their packet data to the aggregation server.
  unsigned char msg[128];
  unsigned char buf[MAX_PACKET_SIZE_READ];
  list<libusb_device_handle*> pip_devs;
  //Set up the USB for a single context (pass NULL as the context)
  libusb_init(NULL);
  libusb_set_debug(NULL, 3);

  //Attach new pip devices.
  attachPIPs(pip_devs);
  //Remember when the USB tree was last checked and check it occasionally
  float last_usb_check;
  {
    timeval tval;
    gettimeofday(&tval, NULL);
    last_usb_check = tval.tv_sec*1000.0 + tval.tv_usec/1000.0;
  }

  while (not killed) {
    SensorConnection agg(server_ip, server_port);

    //A try/catch block is set up to handle exception during quitting.
    try {
      while (agg and not killed) {
        //Check for new USB devices every second
        float cur_time;
        {
          timeval tval;
          gettimeofday(&tval, NULL);
          cur_time = tval.tv_sec*1000.0 + tval.tv_usec/1000.0;
        }
        if (cur_time - last_usb_check > 1.0) {
          last_usb_check = cur_time;
          attachPIPs(pip_devs);
          //Remove any duplicate devices
          pip_devs.sort();
          pip_devs.unique();
        }

        if (pip_devs.size() > 0) {
          //If there isn't any data on USB then sleep for a bit to reduce CPU load.
          bool got_packet = false;
          for (list<libusb_device_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
            //A pip can fail up to two times in a row if this is the first time querying it.
            //If the pip fails after three retries then this pip libusb_device_handle is no longer
            //valid, probably because the pip was removed from the USB.
            int retries_left = 3;
            int transferred = -1;
            int retval = -1;
            while (retval != LIBUSB_ERROR_NO_DEVICE and transferred < 0 and retries_left > 0) {
              // Request the next packet from the pip
              unsigned int timeout = 100;
              msg[0] = LM_GET_NEXT_PACKET;
              retval = libusb_bulk_transfer(*I, 2 | LIBUSB_ENDPOINT_OUT, msg, 1, &transferred, timeout);
              if (0 > retval) {
                std::cout<<"Error requesting data: "<<strerror(retval)<<'\n';
              }
              memset(buf, 0, MAX_PACKET_SIZE_READ);	  

              //Allow up to 20 extra bytes of sensor data beyond the normal packet length.
              if(0 == versions[*I]) {
                retval = libusb_bulk_transfer(*I, 1 | LIBUSB_ENDPOINT_IN, buf+1, PACKET_LEN+20, &transferred, timeout);
                if (0 > retval) {
                  std::cout<<"Error transferring data (old pip): "<<strerror(retval)<<'\n';
                }
              }
              else {
                retval = libusb_bulk_transfer(*I, 2 | LIBUSB_ENDPOINT_IN, buf+1, PACKET_LEN+20, &transferred, timeout);
                if (0 > retval) {
                  std::cout<<"Error transferring data (gpip): "<<strerror(retval)<<'\n';
                }
              }
              //Fill in the length of the extra portion of the packet
              buf[0] = transferred - PACKET_LEN;
              --retries_left;
            }
            //TODO FIXME Check for partial transfers
            //If the pip fails 3 times in a row then it was probably disconnected.
            if (retval < 0) {
              libusb_release_interface(*I, 0);
              libusb_close(*I);
              *I = NULL;
            }
            //If the length of the message is equal to or greater than PACKET_LEN then this is a data packet.
            else if (PACKET_LEN <= transferred) {
              //Data is flowing over USB, continue polling
              got_packet = true;
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
                  sd.sense_data = std::vector<unsigned char>(pkt->data, pkt->data+buf[0]);
                  sd.valid = true;

                  if (pip_debug > DEBUG_GOOD) { 
                    printf("pkt tx: %0x rx: %0lx rss: %0.2f data length %0x \n", 
                        netID, baseID, sd.rss, pkt->ex_length);
                  }
		  

                  //Send the sample data as long as it meets the min RSS constraint
                  if (sd.rss > min_rss) {
                    agg.send(sd);
                  }
                }
              }
            }
          }
          //If there isn't any current data on USB then sleep to
          //reduce CPU consumption
          if (not got_packet) {
            usleep(100);
          }
        }
        else {
          //Sleep for a second if there aren't even any pip devices
          usleep(1000000);
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
  for (list<libusb_device_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
    libusb_release_interface(*I, 0);
    libusb_close(*I);
  }
  libusb_exit(NULL);
}


