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
#include <sys/time.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <time.h>
// Ncurses library for fancy printing
#include <ncurses.h>
#include <panel.h>
#include <cons_ncurses.hpp>

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



//Handle interrupt signals to exit cleanly.
#include <signal.h>


using std::string;
using std::list;
using std::map;
using std::pair;
using std::vector;

std::set<int> recordedIds;
map<int,pip_sample_t> latestSample;
map<int,vector<pip_sample_t>> history;
int highlightId = -1;
std::ofstream recordFile;
pair<int,int> displayBounds(0,0);

WINDOW* mainWindow;
PANEL* mainPanel;

WINDOW* historyWindow;
PANEL* historyPanel;

bool isShowHistory = false;

//Signal handler.
void whandler(int signal) {
  if(signal == SIGWINCH){
//    endwin();
  //  initNCurses();
    setStatus("Console resized. Need to handle this condition since it is not handled yet, and this message is long.");
    return;
  }

}

void stopNCurses(){
  if(recordFile){
    recordFile.close();
  }
  endwin(); // Stop ncurses
}


void setStatus(std::string message){
  int lines,cols;
  getmaxyx(mainWindow,lines,cols);
  wmove(mainWindow,lines-1,2);
  wclrtoeol(mainWindow);
  int trim = message.length() - cols-2;
  if(trim > 0){
    std::string::iterator it = message.end();
    for(; trim > 0 && it != message.begin(); --trim, it--){
    }
    message = std::string(it,message.end());
  }
  wprintw(mainWindow,message.c_str());

  wnoutrefresh(mainWindow);
  doupdate();
}

void initPipData(pip_sample_t& s){
  s.tempC = -300;
  s.rh = -300;
  s.light = -1;
  s.batteryMv = -1;
  s.batteryJ = -1;
  s.interval = 0;
}

void toggleRecording(int tagId){
  if(tagId < 0){
    return;
  }

  if(!panel_hidden(mainPanel)){
    
    std::set<int>::iterator it = recordedIds.find(tagId);
    char buffer[80];
    int bufferOffset = 0;
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
          bufferOffset += snprintf(&buffer[bufferOffset],79,"Unable to open record file!");
        }else {
          recordFile << "Timestamp,Date,Tag ID, RSSI, Temp (C), Relative Humidity (%), Light (%), Battery (mV), Battery (J)" << std::endl;
          bufferOffset += snprintf(&buffer[bufferOffset],79,"Recording to \"%s\". ",filename);
        }


      }
      recordedIds.insert(tagId);
      bufferOffset += snprintf(&buffer[bufferOffset],79,"Started recording %d.",tagId);
    }else {
      recordedIds.erase(it);
      bufferOffset += snprintf(&buffer[bufferOffset],79,"Stopped recording %d. ",tagId);
      if(recordFile and recordedIds.empty()){
        recordFile.close();
        bufferOffset += snprintf(&buffer[bufferOffset],79,"Stopped recording.");
      }
    }
    setStatus(std::string(buffer));

    updateStatusLine(mainWindow,tagId);
  }
}

// Hides the history panel
void hideHistory(){
  show_panel(mainPanel);
  hide_panel(historyPanel);
  isShowHistory = false;
  update_panels();
  repaint();
}

void showHistory(int historyId){
  if (historyId < 0){
    return; 
  }
  //populate the history panel


  setStatus("Showing history");
  show_panel(historyPanel);
  hide_panel(mainPanel);
  update_panels();
  repaint();
}

void updateHighlight(int userKey){
  int step = 0;
  switch(userKey){
    case 27:  // ESC or ALT key
      timeval start,end;
      gettimeofday(&start, NULL);
      userKey = getch();
      if(userKey == ERR){ // ESC key
        hideHistory();
      gettimeofday(&end,NULL);
      char buff[80];
      snprintf(buff,79,"Diff: %ld",(end.tv_usec-start.tv_usec));
      setStatus(std::string(buff));
      }
      break;
    case KEY_HOME:
      if(not latestSample.empty()){
        highlightId = latestSample.begin()->first;
        updateWindowBounds();
        updateStatusList(mainWindow);
      }
      break;
    case KEY_END:
      if(not latestSample.empty()){
        map<int,pip_sample_t>::iterator it = latestSample.end();
        it--;
        highlightId = it->first;
        updateWindowBounds();
        updateStatusList(mainWindow);
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
    case '\n':
    case '\r':
      showHistory(highlightId);
      break;
    default:
      break;
  }
  if(step){
    if(highlightId == -1 && latestSample.size() > 0){
      highlightId = latestSample.begin()->first;
      updateStatusLine(mainWindow,highlightId);
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
        updateStatusList(mainWindow);
      }else {
        updateStatusLine(mainWindow,oldId);
        updateStatusLine(mainWindow,currIt->first);
      }
    }
  }
}

void printStatusLine(WINDOW* win,pip_sample_t pkt, bool highlight){
    wclrtoeol(win);

      if(recordedIds.count(pkt.tagID)){
        wprintw(win,"R ");
      }else {
        wprintw(win,"  ");
      }
      if(highlight){
        wattron(win,A_REVERSE);
        wattron(win,A_BOLD);
      }


      wprintw(win,"%04d  ",pkt.tagID);
      int color = COLOR_RSSI_MED;
      if(pkt.rssi < -90.0){
        color = COLOR_RSSI_LOW;
      }else if(pkt.rssi > -60.0){
        color = COLOR_RSSI_HIGH;
      }
      wattron(win,COLOR_PAIR(color));
      wprintw(win,"%4.1f",pkt.rssi);
      wattroff(win,COLOR_PAIR(color));
      if(pkt.tempC > -300){
        wprintw(win,"  %6.2f C",pkt.tempC);
      }else{
        wprintw(win,"  ------  ");
      }
      if(pkt.rh > -300){
        wprintw(win,"  %6.2f %%  ",pkt.rh);
      }else {
        wprintw(win,"  ------    ");
      }
      if(pkt.light >= 0){
        color = COLOR_LIGHT_MED;
        if(pkt.light < 0x40){
          color = COLOR_LIGHT_LOW;
        }else if(pkt.light > 0xB0){
          color = COLOR_LIGHT_HIGH;
        }
        wattron(win,COLOR_PAIR(color));
        wprintw(win,"%02x",pkt.light);
        wattroff(win,COLOR_PAIR(color));
      }else {
        wprintw(win,"--");
      }

      // Battery
      wprintw(win,"  ");
      if(pkt.batteryMv > 0){
        color = 0; // default color
        if(pkt.batteryMv >2.9){
          color = COLOR_BATTERY_NORMAL;
        }else if(pkt.batteryMv > 0){
          color = COLOR_BATTERY_LOW;
        }
        wattron(win,COLOR_PAIR(color));
        wprintw(win,"%4.3f",pkt.batteryMv);
        wattroff(win,COLOR_PAIR(color));
        wprintw(win,"  ");
        wattron(win,COLOR_PAIR(color));
        wprintw(win,"%4d",pkt.batteryJ);
        wattroff(win,COLOR_PAIR(color));
      }else {
        wprintw(win,"-----  ----");
      }

      //2014-12-02 13:34:04
      char buffer[20];
      strftime(buffer,20,DATE_TIME_FORMAT,std::localtime(&pkt.time.tv_sec));
      wprintw(win,"  %s  ",buffer);

      // Interval
      color = pkt.intervalConfidence > 0.5 ? (pkt.intervalConfidence > 0.95 ? COLOR_CONFIDENCE_HIGH : COLOR_CONFIDENCE_MED) : COLOR_CONFIDENCE_LOW;
      wattron(win,COLOR_PAIR(color));
      wprintw(win,"%6d",pkt.interval);
      wattroff(win,COLOR_PAIR(color));
      wattroff(win,A_BOLD);
      wattroff(win,A_REVERSE);

      wnoutrefresh(win);
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

void updateStatusLine(WINDOW* win,int tagId){
  if(!panel_hidden(mainPanel)){
    drawFraming(win);
    map<int,pip_sample_t>::iterator it = latestSample.find(tagId);
    int row = std::distance(latestSample.begin(),it);
    if(row >= displayBounds.first and row <= displayBounds.second){
      wmove(win,getMinRow()+row-displayBounds.first,0);
      pip_sample_t pkt = latestSample.find(tagId)->second;
      printStatusLine(win,pkt,pkt.tagID == highlightId);
      repaint();
    }
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

  vector<pip_sample_t>& tagHistory = history[sd.tagID];
  tagHistory.insert(tagHistory.begin(),sd);
  if(tagHistory.size() > 100){
    tagHistory.pop_back();
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

  std::set<int>::iterator it = recordedIds.find(sd.tagID);
  if(it != recordedIds.end()){
    recordSample(sd);
  }
  if(sd.dropped > 0){
    char buff[20];
    snprintf(buff,19,"Dropped: %3d",sd.dropped);
//    setStatus(buff);
  }

  renderUpdate(sd.tagID,prevLength != latestSample.size());
}

/**
 * Handles screen updates after a data update
 */
void renderUpdate(int updatedId,bool newEntry){
  updateWindowBounds();
  if(!panel_hidden(mainPanel)){
    if(newEntry){
      updateStatusList(mainWindow);
    }else {
      updateStatusLine(mainWindow,updatedId);
    }
  }

  if(!panel_hidden(historyPanel) and (updatedId == highlightId)){
    if(newEntry){
      updateHistoryList(historyWindow);
    }
  }
}

void updateHistoryList(WINDOW* win){
  
}

void drawFraming(WINDOW* win){
  // Draw "scroll" indicator arrows
  wmove(win,0,0);
  waddch(win,displayBounds.first > 0 ? ('^'|A_BOLD|COLOR_PAIR(COLOR_SCROLL_ARROW)) : ' ');

  int maxx, maxy;
  getmaxyx(win,maxy,maxx);
  wmove(win,maxy-1,0);
  int numIds = latestSample.size();
  waddch(win,(numIds > 0) and (displayBounds.second < (numIds-1)) ? ('v'|A_BOLD|COLOR_PAIR(COLOR_SCROLL_ARROW)) : ' ');
  wmove(win,0,1);
  wclrtoeol(win);
  wattron(win,A_BOLD);
  wprintw(win,"  Tag    RSSI  Temp (C) Rel. Hum. Lt  Batt   Joul  Date                 Period");
  wattroff(win,A_BOLD);
}

void updateStatusList(WINDOW* win){
  if(!panel_hidden(mainPanel)){
    updateWindowBounds();
    drawFraming(win);

    int row = 0;
    map<int,pip_sample_t>::iterator pIter = latestSample.begin();
    for(; pIter != latestSample.end(); ++pIter,++row){
      if(row < displayBounds.first){
        continue;
      }else if(row > displayBounds.second){
        break;
      }
      
      pip_sample_t pkt = pIter->second;
      wmove(win,row-displayBounds.first+getMinRow(),0);
      printStatusLine(win,pkt,pkt.tagID == highlightId);
    }
    for(;row <= displayBounds.second; ++row){
      wmove(win,row-displayBounds.first+getMinRow(),0);
      wclrtoeol(win);
    }

    repaint();
  }
}

void repaint(){
  doupdate();
}

void initNCurses(){
  set_escdelay(25);
  initscr();  // Start ncurses mode
  //halfdelay(1); // Allow character reads to end after 100ms
  cbreak();   // Don't wait for new lines
  nonl();
  timeout(0);   // Non-blocking input from getch()
  keypad(stdscr,TRUE); // Support F1, F2, arrow keys
  noecho();   // Don't show user input
  start_color();// Use color!
  curs_set(0);  // Disable showing cursor on-screen
  
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

  int maxX, maxY;
  getmaxyx(stdscr,maxY,maxX);

  mainWindow = newwin(maxY,maxX, 0, 0);// main window covers entire screen
  historyWindow = newwin(maxY, maxX, 0, 0); // history window covers entire screen

  mainPanel = new_panel(mainWindow);
  historyPanel = new_panel(historyWindow);
  box(historyWindow,0,0);
  hide_panel(historyPanel);
  // Update the stacking order of panels, history on top
  
  signal(SIGWINCH, whandler);
  updateStatusList(mainWindow);
  setStatus(STATUS_INFO_KEYS);
  resizePanels();
  repaint();
}

void resizePanels(){
  int maxX, maxY;
  getmaxyx(stdscr,maxY,maxX);
  WINDOW* oldWin = mainWindow;
  mainWindow = newwin(maxY,maxX,0,0);
  replace_panel(mainPanel,mainWindow);
  delwin(oldWin);
  update_panels();
}


void ncursesUserInput(){
  int userCh = getch();
  if(userCh != ERR){
    updateHighlight(userCh);
  }
}

