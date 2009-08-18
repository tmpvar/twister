/*
  gcode.h - rs274/ngc parser.
  Part of Turntable

  Copyright (c) 2009 Simen Svale Skogsrud

  Turntable is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Turntable is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Turntable.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef gcode_h
#define gcode_h
#include <avr/io.h>

#define GCSTATUS_OK 0
#define GCSTATUS_BAD_NUMBER_FORMAT 1
#define GCSTATUS_EXPECTED_COMMAND_LETTER 2
#define GCSTATUS_UNSUPPORTED_STATEMENT 3
#define GCSTATUS_MOTION_CONTROL_ERROR 4
#define GCSTATUS_FLOATING_POINT_ERROR 5

// Initialize the parser
void gc_init();

// Execute one block of rs275/ngc/g-code
uint8_t gc_execute_line(char *line);

// get the current logical position (in current units), the current status code and the unit mode
void gc_get_status(double *position_, uint8_t *status_code_, int *inches_mode_, uint32_t *line_number_);

#endif
