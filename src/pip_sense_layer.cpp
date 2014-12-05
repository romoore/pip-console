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
//TODO Create lib-cppsensor so that sockets don't need to be handled here.

//These includes need to come first because of the macro defining INT64_MAX
//TODO FIXME Are some of the old C-style includes breaking this macro?
#include <owl/sensor_connection.hpp>
#include <owl/world_model_protocol.hpp>

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

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <set>
#include <algorithm>
#include <stdexcept>
#include <ctime>
#include <cmath>


// Ncurses library for fancy printing
#include <ncurses.h>

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

#define COLOR_RSSI_LOW 1
#define COLOR_RSSI_MED 2
#define COLOR_RSSI_HIGH 3

#define COLOR_LIGHT_LOW 4
#define COLOR_LIGHT_MED 5
#define COLOR_LIGHT_HIGH 6

#define COLOR_SCROLL_ARROW 7

#define COLOR_CONFIDENCE_LOW 8
#define COLOR_CONFIDENCE_MED 9
#define COLOR_CONFIDENCE_HIGH 10

#define DATE_TIME_FORMAT "%m/%d/%Y %H:%M:%S"

typedef unsigned int frequency;
typedef unsigned char bsid;
typedef unsigned char rating;
typedef struct {
  timeval time;
  int tagID;
  float rssi;
  float tempC;
  float rh;
  int light;
  float batteryMv;
  int batteryJ;
  int dropped;
  unsigned long int rcvTime;
  long int interval;
  float intervalConfidence;
} pip_sample_t;
void setStatus(char*);

void printStatusLine(pip_sample_t , bool);
void updateStatusLine(int);
void updateStatusList();
bool updateWindowBounds();
int getHighlightIndex();
void initNCurses();
void drawFraming();

std::set<int> recordedIds;

//Global variable for the signal handler.
bool killed = false;
//Signal handler.
void handler(int signal) {
  if(signal == SIGWINCH){
    endwin();
    initNCurses();
    return;
  }

  if(signal == SIGINT){
    setStatus((char*)"Shutting down. Use CTRL+C to force exit.");
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
  endwin(); // Stop ncurses
}

/* #defines of the commands to the pipsqueak tag */
#define LM_PING (0x11)
#define LM_PONG (0x12)
#define LM_GET_NEXT_PACKET (0x13)
#define LM_RETURNED_PACKET (0x14)
#define LM_NULL_PACKET (0x15)

#define STATUS_INFO_KEYS "Use arrow keys to scroll. Toggle recording with R."

#define RSSI_OFFSET 78
#define CRC_OK 0x80



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

map<int,pip_sample_t> latestSample;

int highlightId = -1;


#define RECORD_FILE_FORMAT "%Y%m%d_%H%M%S.csv"
std::ofstream recordFile;


pair<int,int> displayBounds(0,0);

void setStatus(char* message){
  int r,c;
  getmaxyx(stdscr,r,c);
  move(r-1,2);
  clrtoeol();
  printw(message);
  refresh();
}

void parseData(std::vector<unsigned char>& data,pip_sample_t& s){
  if(data.size() == 0){
    return;
  }
  
  unsigned char hdr = data[0];
  int i = 1;
  s.tempC = -300;
  s.rh = -300;
  s.light = -1;
  s.batteryMv = -1;
  s.batteryJ = -1;
  if(hdr & 0x01){
    s.tempC = data[i]>>1;
    ++i;
  }
  if(hdr & 0x02){
    s.tempC = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
  }

  if(hdr & 0x04){
    s.light = data[i];
    ++i;
  }

  if(hdr & 0x08){
    s.tempC = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
    s.rh = ((data[i]<<4) + (data[i+1]/16.0));
    i += 2;
  }

  if(hdr & 0x10){
    i += 2;
  }

  if(hdr & 0x20){
    i += 6;
  }
  if(hdr & 0x40){
    s.batteryMv = ((data[i]<<8) + (data[i+1]))/1000.0;
    i += 2;
    s.batteryJ = ((data[i]<<8) + (data[i+1]));
    i += 2;
  }

}

void toggleRecording(int tagId){
  if(tagId < 0){
    return;
  }
  
  std::set<int>::iterator it = recordedIds.find(tagId);
  char buffer[80];
  if(it == recordedIds.end()){
    if(recordedIds.empty()){
      if(recordFile){
        recordFile.close();
      }

      char filename[22];
      time_t tval;
      std::time(&tval);
      strftime(filename,22,RECORD_FILE_FORMAT,std::localtime(&tval));
      recordFile.open(filename);
      if(!recordFile){
        setStatus((char*)"Unable to open record file!");
      }else {
        recordFile << "Timestamp,Date,Tag ID, RSSI, Temp (C), Relative Humidity (%), Light (%), Battery (mV), Battery (J)" << std::endl;
        char msg[40];
        snprintf(msg,39,"Recording to \"%s\".",filename);
        setStatus(msg);
      }


    }
    recordedIds.insert(tagId);
    sprintf(buffer,"Started recording %d",tagId);
  }else {
    recordedIds.erase(it);
    sprintf(buffer,"Stopped recording %d",tagId);
    if(recordFile and recordedIds.empty()){
      recordFile.close();
      setStatus((char*)"Stopped recording.");
    }
  }




  updateStatusLine(tagId);
}

void updateHighlight(int userKey){
  switch(userKey){
    case KEY_UP:
      {
        map<int,pip_sample_t>::iterator currIt = latestSample.find(highlightId);
        if(currIt != latestSample.begin()){
          int oldId = currIt->first;
          currIt--;
          highlightId = currIt->first;
          if(updateWindowBounds()){
            updateStatusList();
          }else {
            updateStatusLine(oldId);
            updateStatusLine(currIt->first);
          }
        }
      }
      break;
    case KEY_DOWN:
      {
        if(highlightId == -1 && latestSample.size() > 0){
          highlightId = latestSample.begin()->first;
          updateStatusLine(highlightId);
        }else {
          map<int,pip_sample_t>::iterator currIt = latestSample.find(highlightId);
          if(currIt != latestSample.end()){
            int oldId = currIt->first;
            currIt++;
            if(currIt != latestSample.end()){
              highlightId = currIt->first;
              if(updateWindowBounds()){
                updateStatusList();
              }else {
                updateStatusLine(oldId);
                updateStatusLine(currIt->first);
              }
            } 
          } 
        }
        refresh();
      }
      break;
    case 'R':
    case 'r':
      {
        toggleRecording(highlightId);
      }
      break;
    default:
      break;
  }
}

void printStatusLine(pip_sample_t pkt, bool highlight){
    clrtoeol();

      if(recordedIds.count(pkt.tagID)){
        printw("R ");
      }else {
        printw("  ");
      }
      if(highlight){
        attron(A_REVERSE);
        attron(A_BOLD);
      }


      printw("%04d  ",pkt.tagID);
      int color = COLOR_RSSI_MED;
      if(pkt.rssi < -90.0){
        color = COLOR_RSSI_LOW;
      }else if(pkt.rssi > -60.0){
        color = COLOR_RSSI_HIGH;
      }
      attron(COLOR_PAIR(color));
      printw("%4.1f",pkt.rssi);
      attroff(COLOR_PAIR(color));
      if(pkt.tempC > -300){
        printw("  %6.2f C",pkt.tempC);
      }else{
        printw("  ------  ");
      }
      if(pkt.rh > -300){
        printw("  %6.2f %%  ",pkt.rh);
      }else {
        printw("  ------    ");
      }
      if(pkt.light >= 0){
        color = COLOR_LIGHT_MED;
        if(pkt.light < 0x40){
          color = COLOR_LIGHT_LOW;
        }else if(pkt.light > 0xB0){
          color = COLOR_LIGHT_HIGH;
        }
        attron(COLOR_PAIR(color));
        printw("%02x",pkt.light);
        attroff(COLOR_PAIR(color));
      }else {
        printw("--");
      }

      printw("  %4.3f  %4d",pkt.batteryMv,pkt.batteryJ);

      //2014-12-02 13:34:04
      char buffer[20];
      strftime(buffer,20,DATE_TIME_FORMAT,std::localtime(&pkt.time.tv_sec));
      printw("  %s",buffer);

      // Interval
      color = pkt.intervalConfidence > 0.5 ? (pkt.intervalConfidence > 0.95 ? COLOR_CONFIDENCE_HIGH : COLOR_CONFIDENCE_MED) : COLOR_CONFIDENCE_LOW;
      attron(COLOR_PAIR(color));
      printw("  %6d",pkt.interval);
      attroff(COLOR_PAIR(color));
      attroff(A_BOLD);
      attroff(A_REVERSE);
}

int getMaxRow(){

  int maxx, maxy;
  getmaxyx(stdscr,maxy,maxx);
  return maxy-2;
}

int getMinRow(){
  return 1;
}

/*
 * Returns the index (into latestSample) of the highlighted tag, or -1 if none is highlighted.
 */
int getHighlightIndex(){
  if(highlightId < 0){
    return -1;
  }
  return std::distance(latestSample.begin(),latestSample.find(highlightId));
}

/*
 * Returns true if window display bounds have changed since last call.
 */
bool updateWindowBounds(){
  pair<int,int> oldBounds = displayBounds;

  int hiIndex = getHighlightIndex();

  int maxRows = getMaxRow() - getMinRow();
  int currWindowSize = oldBounds.second - oldBounds.first;
  bool boundsChanged = false;
  // Need to shrink the window size 
  if(currWindowSize > maxRows){
    displayBounds.second = latestSample.size();
    displayBounds.first = displayBounds.second - maxRows;
    if(displayBounds.first < 0){
      displayBounds.first = 0;
    }
    boundsChanged = true;
  }else if(currWindowSize < maxRows){
    displayBounds.first = 0;
    displayBounds.second = maxRows;
    boundsChanged = true;
  }

  // Check to see if bounds need to "move"
  if(hiIndex >= 0){
    // Highlighted row is no longer within bounds, move the bounds
    if(hiIndex < displayBounds.first){
      displayBounds.first = hiIndex;
      displayBounds.second = displayBounds.first + maxRows;
      boundsChanged = true;
    }else if(hiIndex > displayBounds.second){
      displayBounds.second = hiIndex;
      displayBounds.first = displayBounds.second - maxRows;
      boundsChanged = true;
    }
  }

  return boundsChanged;
}

void updateStatusLine(int tagId){
  drawFraming();
  map<int,pip_sample_t>::iterator it = latestSample.find(tagId);
  int row = std::distance(latestSample.begin(),it);
  if(row >= displayBounds.first and row <= displayBounds.second){
    move(getMinRow()+row-displayBounds.first,0);
    pip_sample_t pkt = latestSample.find(tagId)->second;
    printStatusLine(pkt,pkt.tagID == highlightId);
  }


}


/*
 * timestamp, date/time, tagId, rssi
 */
#define RECORD_FILE_LINE_FORMAT "%ld%03ld,%s,%d,%.1f,"
#define RECORD_FILE_LINE_FORMAT_F4 "%.4f"
#define RECORD_FILE_LINE_FORMAT_F3 "%.3f"
/*
 * (timestamp,date/time/tagId,rssi), temp, rh, light, battery, joules
 */ 
#define RECORD_FILE_LINE_FORMAT_ALL "%s,%s,%s,%s,%s"
#define RECORD_FILE_TIME_FORMAT "%m/%d/%Y %H:%M:%S"

void recordSample(pip_sample_t& sd){
  if(recordFile){
    char buff[255];
    char tbuff[24]; // Date + time
    strftime(tbuff,23,RECORD_FILE_TIME_FORMAT,std::localtime(&sd.time.tv_sec));
    
    int length = snprintf(buff,254,RECORD_FILE_LINE_FORMAT,sd.time.tv_sec,sd.time.tv_usec/1000,tbuff,sd.tagID,sd.rssi);
    if(sd.tempC > -299){
      length += snprintf(buff+length,254-length,RECORD_FILE_LINE_FORMAT_F4,sd.tempC);
    }
    length += snprintf(buff+length,254-length,",");

    if(sd.rh > -299){
      length += snprintf(buff+length,254-length,RECORD_FILE_LINE_FORMAT_F4,sd.rh);
    }
    length += snprintf(buff+length,254-length,",");

    if(sd.light >= 0){
      length += snprintf(buff+length,254-length,RECORD_FILE_LINE_FORMAT_F3,sd.light/255.0);
    }
    length += snprintf(buff+length,254-length,",");

    if(sd.batteryMv >=0){
      length += snprintf(buff+length,254-length,RECORD_FILE_LINE_FORMAT_F3,sd.batteryMv);
    }
    length += snprintf(buff+length,254-length,",");

    if(sd.batteryJ >= 0){
      length += snprintf(buff+length,254-length,"%d",sd.batteryJ);
    }
  
    if(!(recordFile << std::string(buff,length) << std::endl)){
      setStatus((char*)"Error writing to record file!");
    }
  }

}

void updateState(pip_sample_t& sd){
  int prevLength = latestSample.size();
  pip_sample_t& storedData = latestSample[sd.tagID];
  unsigned long int oldTime = (storedData.time.tv_sec*1000 + storedData.time.tv_usec/1000);
  storedData.time = sd.time;
  storedData.tagID = sd.tagID;
  storedData.time = sd.time;
  storedData.rssi = sd.rssi;
  storedData.tempC = sd.tempC;
  storedData.rh = sd.rh;
  storedData.light = sd.light;
  storedData.rcvTime = sd.rcvTime;
  if(sd.batteryMv > 0){
    storedData.batteryMv = sd.batteryMv;
    storedData.batteryJ = sd.batteryJ;
  }

  // Update interval and confidence metric
  if(storedData.interval == 0){
    storedData.interval = 15000;
    storedData.intervalConfidence = 0.0;
  }else {
    long int newTime = (storedData.time.tv_sec*1000 + storedData.time.tv_usec/1000);
    long int newInterval = newTime - oldTime;
    float ratio = ((storedData.interval-newInterval)/(float)storedData.interval);

    float intAdj = (newInterval-storedData.interval);
//    float intAdj = (diff - storedData.interval)/1.0;
//    long int compDiff = std::round(diff / 100.0);
//    long int compInterval = std::round(storedData.interval / 100.0);
//    long int decide = compDiff - compInterval;

    if(ratio > .05){
      storedData.intervalConfidence *= 0.95;
    }else if(ratio < -.05){
      storedData.intervalConfidence *= 0.95;
    }else {
      storedData.intervalConfidence = storedData.intervalConfidence*0.65 + .35;
      if(storedData.intervalConfidence > 0.99){
        storedData.intervalConfidence = 1.0;
      }
    }
    storedData.interval += (intAdj*(1-(storedData.intervalConfidence*.9)));
  }
  if(prevLength != latestSample.size()){
    updateWindowBounds();
    updateStatusList();
  }else {
    updateStatusLine(sd.tagID);
  }

  std::set<int>::iterator it = recordedIds.find(sd.tagID);
  if(it != recordedIds.end()){
    recordSample(sd);
  }
  if(sd.dropped > 0){
    char buff[20];
    snprintf(buff,19,"Dropped: %3d",sd.dropped);
    setStatus(buff);
  }
}

void drawFraming(){
  // Draw "scroll" indicator arrows
  move(0,0);
  addch(displayBounds.first > 0 ? ('^'|A_BOLD|COLOR_PAIR(COLOR_SCROLL_ARROW)) : ' ');

  int maxx, maxy;
  getmaxyx(stdscr,maxy,maxx);
  move(maxy-1,0);
  int numIds = latestSample.size();
  addch((numIds > 0) and (displayBounds.second < (numIds-1)) ? ('v'|A_BOLD|COLOR_PAIR(COLOR_SCROLL_ARROW)) : ' ');
  move(0,1);
  clrtoeol();
  attron(A_BOLD);
  printw("  Tag    RSSI  Temp (C) Rel. Hum. Lt  Batt   Joul  Date                 Period");
  attroff(A_BOLD);
}

void updateStatusList(){
  updateWindowBounds();
  drawFraming();

  int row = 0;
  map<int,pip_sample_t>::iterator pIter = latestSample.begin();
  for(; pIter != latestSample.end(); ++pIter,++row){
    if(row < displayBounds.first){
      continue;
    }else if(row > displayBounds.second){
      break;
    }
    
    pip_sample_t pkt = pIter->second;
    move(row-displayBounds.first+getMinRow(),0);
    printStatusLine(pkt,pkt.tagID == highlightId);
  }
  for(;row <= displayBounds.second; ++row){
    move(row-displayBounds.first+getMinRow(),0);
    clrtoeol();
  }
  refresh();
}

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


void initNCurses(){
  initscr();  // Start ncurses mode
  //halfdelay(1); // Allow character reads to end after 100ms
  cbreak();
  timeout(0);
  keypad(stdscr,TRUE); // Support F1, F2, arrow keys
  noecho();   // Don't show user input
  start_color();// Use color!
  curs_set(0);
  updateStatusList();
  setStatus((char*)STATUS_INFO_KEYS);
}

int main(int ac, char** arg_vector) {

  // Prepare ncurses
  initNCurses();

  init_pair(COLOR_RSSI_LOW,COLOR_RED, COLOR_BLACK);
  init_pair(COLOR_RSSI_MED, COLOR_YELLOW, COLOR_BLACK);
  init_pair(COLOR_RSSI_HIGH, COLOR_GREEN, COLOR_BLACK);
  init_pair(COLOR_LIGHT_LOW, COLOR_WHITE, COLOR_BLACK);
  init_pair(COLOR_LIGHT_MED, COLOR_WHITE, COLOR_BLACK);
  init_pair(COLOR_LIGHT_HIGH, COLOR_YELLOW, COLOR_BLACK);
  init_pair(COLOR_SCROLL_ARROW, COLOR_WHITE, COLOR_BLUE);
  init_pair(COLOR_CONFIDENCE_LOW, COLOR_RED, COLOR_BLACK);
  init_pair(COLOR_CONFIDENCE_MED, COLOR_YELLOW, COLOR_BLACK);
  init_pair(COLOR_CONFIDENCE_HIGH, COLOR_GREEN, COLOR_BLACK);
  
  
  //Set up a signal handler to catch interrupt signals so we can close gracefully
  signal(SIGINT, handler);  
  signal(SIGWINCH, handler);

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
          int userCh = getch();
          if(userCh != ERR){
            updateHighlight(userCh);
          }
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

/*
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
*/
                  /* XOR each group of 3 bytes until all of the 24 bits have been XORed. */
/*                  for (i = 7; i >= 0; --i) {
                    unsigned char triple = (packet >> (3 * i)) & 0x7;
                    p1 ^= triple >> 2;
                    p2 ^= (triple >> 1) & 0x1;
                    p3 ^= triple & 0x1;
                  }
*/
                  /* If the end result of the XORs is three 0 bits then even parity held,
                   * which suggests that the packet data is good. Otherwise there was a bit error. */
/*                  if (p1 ==  0 && p2 == 0 && p3 == 0) {
                    parity_failed = false;
                  }
                  else {
                    parity_failed = true;
                  }
                }
*/
//                if (not parity_failed) {
                  //Now assemble a sample data variable and send it to the aggregation server.
                  //Calculate the tagID here instead of using be32toh since it is awkward to convert a
                  //21 bit integer to 32 bits. Multiply by 8192 and 32 instead of shifting by 13 and 5
                  //bits respectively to avoid endian issues with bit shifting.
                  unsigned int netID = ((unsigned int)data[9] * 65536)  + ((unsigned int)data[10] * 256) +
                    ((unsigned int)data[11] );
                  //We do not currently use the pip's local timestamp
                  //unsigned long time = ntohl(pkt->time);
//                  unsigned long baseID = ntohl(pkt->boardID << 8);
                  pip_sample_t s;
                  s.tagID = netID;

                  //Set this to the real timestamp, milliseconds since 1970
                  timeval tval;
                  //time_t tval;
                  //std::time(&tval);
                  gettimeofday(&tval,NULL);
                  s.time = tval;
                  s.rcvTime = ntohl(pkt->time);
                  s.dropped = pkt->dropped;
                  //Convert from one byte value to a float for receive signal
                  //strength as described in the TI/chipcon Design Note DN505 on cc1100
                  s.rssi = ( (pkt->rssi) >= 128 ? (signed int)(pkt->rssi-256)/2.0 : (pkt->rssi)/2.0) - RSSI_OFFSET;
                  std::vector<unsigned char> dat(pkt->data, pkt->data+buf[0]);
                  parseData(dat,s);
                  
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
    //Try to reconnect to the server after losing the connection.
    //Sleep a little bit, then try connecting to the server again.
  }
//  std::cerr<<"Exiting\n";
  //Clean up the pip connections before exiting.
  for (list<libusb_device_handle*>::iterator I = pip_devs.begin(); I != pip_devs.end(); ++I) {
    libusb_release_interface(*I, 0);
    libusb_close(*I);
  }
  if(recordFile){
    recordFile.close();
  }
  cleanShutdown();
  return 0;
}


