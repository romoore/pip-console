#ifndef PIP_CONS_NCURSES_H_
#define PIP_CONS_NCURSES_H_
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


#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
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

#include <cons_ncurses.hpp>

using std::string;
using std::list;
using std::map;
using std::pair;

std::set<int> recordedIds;
map<int,pip_sample_t> latestSample;
int highlightId = -1;
std::ofstream recordFile;
pair<int,int> displayBounds(0,0);

//Signal handler.
void whandler(int signal) {
  if(signal == SIGWINCH){
    endwin();
    initNCurses();
    return;
  }

}

void stopNCurses(){
  if(recordFile){
    recordFile.close();
  }
  endwin(); // Stop ncurses
}


void setStatus(char* message){
  int r,c;
  getmaxyx(stdscr,r,c);
  move(r-1,2);
  clrtoeol();
  printw(message);
  refresh();
}

void initPipData(pip_sample_t& s){
  s.tempC = -300;
  s.rh = -300;
  s.light = -1;
  s.batteryMv = -1;
  s.batteryJ = -1;
  s.interval = 0;
}

void parseData(std::vector<unsigned char>& data,pip_sample_t& s){
  if(data.size() == 0){
    return;
  }
  
  unsigned char hdr = data[0];
  int i = 1;
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
  int step = 0;
  switch(userKey){
    case KEY_HOME:
      if(not latestSample.empty()){
        highlightId = latestSample.begin()->first;
        updateWindowBounds();
        updateStatusList();
      }
      break;
    case KEY_END:
      if(not latestSample.empty()){
        map<int,pip_sample_t>::iterator it = latestSample.end();
        it--;
        highlightId = it->first;
        updateWindowBounds();
        updateStatusList();
      }
      break;
    case KEY_UP:
      step = -1;
      break;
    case KEY_PPAGE:
      step = (displayBounds.first - displayBounds.second);
      break;
    case KEY_DOWN:
      step = 1;
      break;
    case KEY_NPAGE:
      step = (displayBounds.second - displayBounds.first);
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
  if(step){
    if(highlightId == -1 && latestSample.size() > 0){
      highlightId = latestSample.begin()->first;
      updateStatusLine(highlightId);
    }else {
      map<int,pip_sample_t>::iterator currIt = latestSample.find(highlightId);
      int oldId = currIt->first;
      // Move up the list
      if(step < 0 and currIt != latestSample.begin()){
        while(step < 0 and currIt != latestSample.begin()){
          currIt--;
          ++step;
        }
        highlightId = currIt->first;
      }
      // Move down the list
      else {
        map<int,pip_sample_t>::iterator stopIt = latestSample.end();
        stopIt--;
        while(step > 0 and currIt != stopIt){
          currIt++;
          --step;
        }
        highlightId = currIt->first;
      }
      // Update display
      if(updateWindowBounds()){
        updateStatusList();
      }else {
        updateStatusLine(oldId);
        updateStatusLine(currIt->first);
      }
    }
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

      // Battery
      printw("  ");
      if(pkt.batteryMv > 0){
        color = 0; // default color
        if(pkt.batteryMv >2.9){
          color = COLOR_BATTERY_NORMAL;
        }else if(pkt.batteryMv > 0){
          color = COLOR_BATTERY_LOW;
        }
        attron(COLOR_PAIR(color));
        printw("%4.3f",pkt.batteryMv);
        attroff(COLOR_PAIR(color));
        printw("  ");
        attron(COLOR_PAIR(color));
        printw("%4d",pkt.batteryJ);
        attroff(COLOR_PAIR(color));
      }else {
        printw("-----  ----");
      }

      //2014-12-02 13:34:04
      char buffer[20];
      strftime(buffer,20,DATE_TIME_FORMAT,std::localtime(&pkt.time.tv_sec));
      printw("  %s  ",buffer);

      // Interval
      color = pkt.intervalConfidence > 0.5 ? (pkt.intervalConfidence > 0.95 ? COLOR_CONFIDENCE_HIGH : COLOR_CONFIDENCE_MED) : COLOR_CONFIDENCE_LOW;
      attron(COLOR_PAIR(color));
      printw("%6d",pkt.interval);
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
  // If initialized to 0, then make invalid
  else if(storedData.batteryMv < 0.0001){
    storedData.batteryMv = -1;
    storedData.batteryJ = -1;
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

void initNCurses(){
  initscr();  // Start ncurses mode
  //halfdelay(1); // Allow character reads to end after 100ms
  cbreak();
  timeout(0);
  keypad(stdscr,TRUE); // Support F1, F2, arrow keys
  noecho();   // Don't show user input
  start_color();// Use color!
  curs_set(0);
  
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
  init_pair(COLOR_BATTERY_LOW, COLOR_RED, COLOR_BLACK);
  init_pair(COLOR_BATTERY_NORMAL, COLOR_GREEN, COLOR_BLACK);
  
  signal(SIGWINCH, whandler);
  updateStatusList();
  setStatus((char*)STATUS_INFO_KEYS);
  
}
void ncursesUserInput(){
  int userCh = getch();
  if(userCh != ERR){
    updateHighlight(userCh);
  }
}

#endif
