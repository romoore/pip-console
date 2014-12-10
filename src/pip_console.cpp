/*
 * Copyright (c) 2012 Bernhard Firner and Rutgers University
 * Copyright (C) 2014 Robert S. Moore II and Rutgers University
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
 * @author Robert S. Moore II
 ******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#include <arpa/inet.h>

#include <iostream>
#include <list>
#include <map>
#include <algorithm>


// Ncurses library for fancy printing
#include <cons_ncurses.hpp>

//Handle interrupt signals to exit cleanly.
#include <signal.h>

using std::string;
using std::list;
using std::map;
using std::pair;

#define MAX_PACKET_SIZE_READ		(64 *1024 )

//Global variable for the signal handler.
bool killed = false;


//Signal handler.
void handler(int signal) {
  if(signal == SIGINT){
    setStatus("Shutting down. Use CTRL+C to force exit.");
    if (killed) {
      exit(-1);
    }
    killed = true;
  }
}


float toFloat(unsigned char* pipFloat) {
    return ((float)pipFloat[0] * 0x100 + (float)pipFloat[1] + (float)pipFloat[2] / (float)0x100);
}

void cleanShutdown(){
  libusb_exit(NULL);
  stopNCurses();
}

/* #defines of the commands to the pipsqueak tag */
#define LM_GET_NEXT_PACKET (0x13)

/* Defined in CC1100 data sheet/errata. */
#define RSSI_OFFSET 78



//PIP 3 Byte ID packet structure with variable data segment.
//3 Byte receiver ID, 21 bit transmitter id, 3 bits of parity plus up to 20 bytes of extra data.
typedef struct {
	unsigned char ex_length : 8; //Length of data in the optional data portion
	unsigned char dropped   : 8; //The number of packet that were dropped if the queue overflowed.
	unsigned int boardID    : 24;//Basestation ID
	unsigned int time       : 32;//Timestamp in quarter microseconds.
	unsigned int tagID      : 24;//Transmitter ID
//	unsigned int parity     : 3; //Even parity check on the transmitter ID
	unsigned char rssi      : 8; //Received signal strength indicator
  unsigned char lqi       : 7; //The lower 7 bits contain the link quality indicator
	unsigned char crcok     : 1; 
	unsigned char data[20];      //The optional variable length data segment
  float rss;
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


/*
 * Parses the "extra data" portion of the Pip packet into 
 * the actual recorded data.  Values are written into 
 * s if present.
 */
void parseData(std::vector<unsigned char>& data,pip_sample_t& s){
  if(data.size() == 0){
    return;
  }
  
  unsigned char hdr = data[0];
  int i = 1;
  // Binary sensing (doors, water, etc.) in bit 0x01
  // Temperature in bits 0xFE, Celsius, offset 40 degrees
  if(hdr & 0x01){
    s.tempC = (data[i]>>1) - 40;
    ++i;
  }
  // Temperature in Celsius, not offset, in 16ths of a degree.
  if(hdr & 0x02){
    s.tempC = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
  }

  // Ambient light based on "dark" (0x00) to bright office (0xFF)
  if(hdr & 0x04){
    s.light = data[i];
    ++i;
  }

  /*
   * Off-chip temperature and relative humidity sensing.
   * Temperature is 16ths of a degree C.
   * Relative humidity is in 16ths of a percent.
   */
  if(hdr & 0x08){
    s.tempC = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
    s.rh = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
  }

  // Skipping 2-byte moisture values for now
  if(hdr & 0x10){
    s.moisture = ((data[i]<<8) + (data[i+1]));
    i += 2;
  }

  // Skipping 6-byte history values for now
  if(hdr & 0x20){
    i += 6;
  }

  /*
   * Battery status.  First is the battery voltage measured in millivolts.
   * Second is the estimated number of Joules consumed since start-up.
   */
  if(hdr & 0x40){
    s.batteryMv = ((data[i]<<8) + (data[i+1]))/1000.0;
    i += 2;
    s.batteryJ = ((data[i]<<8) + (data[i+1]));
    i += 2;
  }

}


/*
 * Legacy code (from GRAIL?) that needs to be replaced.
 * 
 * Idea: Scan the USB tree, extract PIPs, grab a packet (to read ID), and 
 *       present them to the user to pick one.
 */
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
//        std::cerr<<"Connected to USB Tag Reader.\n";
        libusb_device_handle* new_handle;
        int err = libusb_open(dev, &new_handle);

        if (0 != err) {
          if (LIBUSB_ERROR_ACCESS == err) {
//            std::cout<<"Insufficient permission to open reader (try sudo).\n";
          }
        }
        //Otherwise getting a handle was successful
        else {
          //Reset the device before trying to use it
          if (0 == libusb_reset_device(new_handle)) {
            //Add the new device to the pip device list.
            pip_devs.push_back(new_handle);
//            std::cout<<"New pipsqueak opened.\n";
            in_use[device_num] = true;
            versions[new_handle] = version;

            int retval = libusb_set_configuration(pip_devs.back(), 1);
            if (0 != retval ) { 
              switch(retval){
                case LIBUSB_ERROR_NOT_FOUND:
                  //printf("Device not found.\n");
                  break;
                  case LIBUSB_ERROR_BUSY: 
                  //printf("Device is busy.\n");
                  retval = libusb_detach_kernel_driver(pip_devs.back(),0);
                  if(0 != retval){
                    //printf("Unable to detach kernel driver with error number %d.\n",retval);
                  }
                  retval = libusb_set_configuration(pip_devs.back(),1);
                  break;
                  case LIBUSB_ERROR_NO_DEVICE:
                  //printf("No device present.\n");
                  break;
                  default:
                  //printf("Unknown error.\n");
                  break;
              }
              //printf("Setting configuration to 1 failed with error number %d \n",retval);
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
//                std::cerr<<"usb_claim_interface failed\n";
//                std::cerr<<"If the interface cannot be claimed try running with root privileges.\n";
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


/*
 * Main method, scans for USB devices, reads Pip packets (if Pipsqueak device
 * found), and checks for user input on keyboard.  When Ctrl+C (SIGINT) is
 * detected, "killed" will become false, and main will exit.
 */
int main(void) {

  // Prepare ncurses
  initNCurses();
  
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
  double last_usb_check;
  double last_ch_check;
  {
    timeval tval;
    gettimeofday(&tval, NULL);
    last_usb_check = tval.tv_sec*1000.0;
    last_ch_check = tval.tv_sec*1000.0 + tval.tv_usec/1000.0;
  }

  while (not killed) {

    //A try/catch block is set up to handle exception during quitting.
    try {
      while (not killed) {
        //Check for new USB devices every second
        double cur_time;
        {
          timeval tval;
          gettimeofday(&tval, NULL);
          cur_time = tval.tv_sec * 1000.0 + tval.tv_usec/1000.0;
        }
        if(cur_time - last_ch_check > 50){
          last_ch_check = cur_time;
          ncursesUserInput();
        }
        if (cur_time - last_usb_check > 30000) {
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
            //A return value of -99 means unknown error. In many cases we should detach and reconnect.
            while (-99 != retval and retval != LIBUSB_ERROR_NO_DEVICE and transferred < 0 and retries_left > 0) {
              // Request the next packet from the pip
              unsigned int timeout = 100;
              msg[0] = LM_GET_NEXT_PACKET;
              retval = libusb_bulk_transfer(*I, 2 | LIBUSB_ENDPOINT_OUT, msg, 1, &transferred, timeout);
              if (0 > retval) {
//                std::cout<<"Error requesting data: "<<strerror(retval)<<'\n';
              }
              else {
                memset(buf, 0, MAX_PACKET_SIZE_READ);	  

                //Allow up to 20 extra bytes of sensor data beyond the normal packet length.
                if(0 == versions[*I]) {
                  retval = libusb_bulk_transfer(*I, 1 | LIBUSB_ENDPOINT_IN, buf+1, PACKET_LEN+20, &transferred, timeout);
                  if (0 > retval) {
//                    std::cout<<"Error transferring data (old pip): "<<strerror(retval)<<'\n';
                  }
                }
                else {
                  retval = libusb_bulk_transfer(*I, 2 | LIBUSB_ENDPOINT_IN, buf+1, PACKET_LEN+20, &transferred, timeout);
                  if (0 > retval) {
//                    std::cout<<"Error transferring data (gpip): "<<strerror(retval)<<'\n';
                  }
                }
                //Fill in the length of the extra portion of the packet
                buf[0] = transferred - PACKET_LEN;
              }
              --retries_left;
            }
            //TODO FIXME Check for partial transfers
            //If the pip fails 3 times in a row then it was probably disconnected.
            //If it is still attached to the interface it will be detected again.
            if (retval < 0) {
              //In older versions of libusb1.0 this is an unrecoverable error that destroys the library.
              //Close everything and try again
              if (-99 == retval) {
                for (list<libusb_device_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
                  libusb_release_interface(*I, 0);
                  libusb_close(*I);
                  *I = NULL;
                }
                cleanShutdown();
                return 0;
              }
              else if (LIBUSB_ERROR_NO_DEVICE == retval) {
//                std::cerr<<"Device disconnected\n";
                libusb_release_interface(*I, 0);
                libusb_close(*I);
                *I = NULL;
              }
              else {
                std::cerr<<"Trying to detach\n";
                //libusb_reset_device (*I);
                //libusb_clear_halt(*I, 0);
                libusb_release_interface(*I, 0);
                libusb_close(*I);
                *I = NULL;
                std::cerr<<"Detached\n";
                std::cerr<<"At this point in time the flawed libusb probably cannot attach new devices.\n";
              }
            }
            //If the length of the message is equal to or greater than PACKET_LEN then this is a data packet.
            else if (PACKET_LEN <= transferred) {
              //Data is flowing over USB, continue polling
              got_packet = true;
              //Overlay the packet struct on top of the pointer to the pip's message.
              pip_packet_t *pkt = (pip_packet_t *)buf;

              //Check to make sure this was a good packet.
              if ((pkt->rssi != (int) 0) and (pkt->lqi != 0) and (pkt->crcok)) {
                unsigned char* data = (unsigned char*)pkt;

                  unsigned int netID = ((unsigned int)data[9] * 65536)  + ((unsigned int)data[10] * 256) +
                    ((unsigned int)data[11] );
                  //We do not currently use the pip's local timestamp
                  //unsigned long time = ntohl(pkt->time);
//                  unsigned long baseID = ntohl(pkt->boardID << 8);
                  pip_sample_t s;
                  s.tagID = netID;

                  //Set this to the real timestamp, milliseconds since 1970
                  timeval tval;
                  gettimeofday(&tval,NULL);
                  s.time = tval;
                  s.rcvTime = ntohl(pkt->time);
                  s.dropped = pkt->dropped;
                  //Convert from one byte value to a float for receive signal
                  //strength as described in the TI/chipcon Design Note DN505 on cc1100
                  s.rssi = ( (pkt->rssi) >= 128 ? (signed int)(pkt->rssi-256)/2.0 : (pkt->rssi)/2.0) - RSSI_OFFSET;
                  initPipData(s);
                  if(pkt->ex_length){
                    std::vector<unsigned char> dat(pkt->data, pkt->data+buf[0]);
                    parseData(dat,s);
                  }
                  
                  updateState(s);

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
//      std::cerr<<"USB sensor layer error: "<<re.what()<<'\n';
    }
    catch (std::exception& e) {
//      std::cerr<<"USB sensor layer error: "<<e.what()<<'\n';
    }
  }
//  std::cerr<<"Exiting\n";
  //Clean up the pip connections before exiting.
  for (list<libusb_device_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
    libusb_release_interface(*I, 0);
    libusb_close(*I);
  }
  cleanShutdown();
  return 0;
}


