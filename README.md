pip-sense-layer
===============

About
-----
  This program reads data from PIP transmitters over USB and transmit the data
  to an Owl Platform aggregator. See the Owl Platform website for more
  information on using the data or running an aggregator:

  Owl Platform: <https://github.com/OwlPlatform>

Building
--------
  Building this requires the cpp-owl-common and cpp-owl-sensor libraries from
  the owl platform github page: https://github.com/owlplatform

  The build process is as follow:
  cmake .
  make
  sudo make install

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
