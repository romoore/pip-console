#ifndef PIP_CONS_NCURSES_H_
#define PIP_CONS_NCURSES_H_
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
 * @file cons_ncurses.hpp
 * Header definition for ncurses console for Pipsqueak tags.
 *
 * @author Robert S. Moore II
 ******************************************************************************/

#include <ncurses.h>
#include <string>

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

#define COLOR_BATTERY_LOW 11
#define COLOR_BATTERY_NORMAL 12

#define DATE_TIME_FORMAT "%m/%d/%Y %H:%M:%S"

#define STATUS_INFO_KEYS "Use arrow keys to scroll. Toggle recording with R. Esc to quit."
#define STATUS_INFO_HISTORY "Use arrow keys to scroll. Esc to exit."

#define RECORD_FILE_FORMAT "%Y%m%d_%H%M%S.csv"

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

void repaint();
void resizePanels();
void setStatus(std::string);
void printStatusLine(WINDOW*,pip_sample_t , bool);
void updateState(pip_sample_t&);
void updateStatusLine(WINDOW*, int);
void updateStatusList(WINDOW*);
bool updateWindowBounds();
int getMainHighlightIndex();
void initNCurses();
void drawFraming(WINDOW*);
void initNCurses();
void stopNCurses();
void ncursesUserInput();
void initPipData(pip_sample_t&);
void toggleRecording(int);
void renderUpdate(int,bool);
int getMinRow(WINDOW* win);
int getMaxRow(WINDOW* win);


#endif
