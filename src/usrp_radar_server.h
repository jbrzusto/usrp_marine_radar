/* -*- c++ -*- */
/*
 * Copyright 2011 John Brzustowski
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_USRP_RADAR_SERVER_H
#define INCLUDED_USRP_RADAR_SERVER_H

class usrp_radR_server {

 public:

  std::string get_device_name(); /**< return a string representing the USRP this server operates, or an empty string for none */
  
  bool start_server (); /**< begin accepting client connections */

  bool stop_server (); /**< stop accepting client connections */

  bool start_device(); /**< begin USRP device digitizing */

  bool stop_device (); /**< stop USRP device digitizing */

  

  


#endif // INCLUDED_USRP_RADAR_SERVER_H 
