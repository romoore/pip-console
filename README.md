pip-sense-layer
===============

About
-----
  This program reads data from PIP transmitters over USB and transmit the data
  to an Owl Platform aggregator. See the Owl Platform website for more
  information on using the data or running an aggregator:

  Owl Platform: <https://github.com/OwlPlatform>

Dependencies
------------
  This program depends upon the libusb1.0 library. On Debian-derived systems you
  can fetch this with the command

  sudo apt-get install libusb-1.0-0-dev 

  This program also depends upon the cpp-owl-common and cpp-owl-sensor packages
  that can be found in the OwlPlatform github repository and downloaded through
  git with the following commands:

  git clone git://github.com/OwlPlatform/cpp-owl-common.git

  git clone git://github.com/OwlPlatform/cpp-owl-sensor.git

Building
--------
  The build process is as follows:

  cmake . && make && sudo make install

License
-------
 Copyright (c) 2012 Bernhard Firner and Rutgers University
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
