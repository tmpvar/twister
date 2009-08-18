/*
  serial_protocol.h - the serial protocol master control unit
  Part of Twister

  Copyright (c) 2009 Simen Svale Skogsrud

  Twister is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Twister is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Twister.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef serial_h
#define serial_h

// Initialize the serial protocol
void sp_init();

// Read command lines from the serial port and execute them as they
// come in. Blocks until the serial buffer is emptied. 
void sp_process();

#endif
