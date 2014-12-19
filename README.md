pip-console
===========

About
-----
  This program reads data from PIP transmitters over USB and displays it to
  the console using the new curses (ncurses) library.  Requires both ncurses
  and the panel library.

  Owl Platform: <https://github.com/OwlPlatform>

Usage
-----
  Once running, the program will display a list of Pipsqueak transmitters based
  on the packets it receives.  Each row is a separate Pipsqueak receiver and
  includes RSSI, temperature and relative humidity values, ambient light
  values, battery voltage (measured) and energy consumption (estimated), and
  the timestamp of the most recent packet received.  In addition, it will
  attempt to calculate the broadcast period of each transmitter, displaying
  this value in milliseconds.  The estimation confidence of the broadcast
  period alters the color of the value where red is very low confidence, yellow
  is moderate confidence, and green is high confidence.

  Pressing the 'X' key during operation will translate the Pipsqueak tag IDs
  from decimal to hexadecimal representation.

  Pressing either the 'Delete' or 'Backspace' key will remove that sensor from
  the main listing.

  You can highlight the different Pipsqueak transmitter rows by using the Up
  and Down arrow keys, the Page Up and Page Down keys, or the Home and End
  keys. Pressing Enter or Return on a row will display the packet history of
  the transmitter, up to the last 100 packets.  Scrolling the history is the
  same as the main screen.  Press the Esc key to return to the main screen.
  Exiting the program is accomplished by sending a SIGQUIT, typically with
  Ctrl+C.

  You may record the decoded packet data to a file at any time while running
  the program.  To start recording, highlight a Pipsqueak using the arrow keys,
  and then press the 'R' key.  A new file will be created in whichever
  directory you started the program.  You may record additional Pipsqueak tags
  by highlighting their rows and pressing the 'R' key. Tags which are being
  recorded are marked with an 'R' in the far left margin of their row.  To stop
  recording a tag, highlight its row and press the 'R' key again.  When no tags
  are being recorded, the file will be closed.  A new file will be created the
  next time recording is started.

  The optional flag "--fun" will reduce the delay for the "screen saver"
  feature.

Dependencies
------------
  This program depends upon the libusb1.0 library and ncurses library. On
  Debian-derived systems you can fetch this with the command

  sudo apt-get install libusb-1.0-0-dev libncurses-dev

Building
--------
  The build process is as follows:

  cmake . && make && sudo make install

License
-------
 Copyright (C) 2012 Bernhard Firner and Rutgers University  
 Copyright (C) 2014 Robert S. Moore II and Rutgers University  
 All rights reserved.
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 or visit http://www.gnu.org/licenses/gpl-2.0.html
