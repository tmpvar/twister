/*
  GCodeParser.h - rs274/ngc parser.
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


#ifndef gcode_h
#define gcode_h
#include <avr/io.h>

#define GCSTATUS_OK 0
#define GCSTATUS_BAD_NUMBER_FORMAT 1
#define GCSTATUS_EXPECTED_COMMAND_LETTER 2
#define GCSTATUS_UNSUPPORTED_STATEMENT 3
#define GCSTATUS_MOTION_CONTROL_ERROR 4
#define GCSTATUS_FLOATING_POINT_ERROR 5

class GCodeParser {
private:
  GCodeParser();
  static uint32_t line_number;
  static uint8_t status_code;
  static uint8_t motion_mode;         /* {G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89} */
  static uint8_t inverse_feed_rate_mode; /* G93, G94 */
  static uint8_t inches_mode;         /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
  static uint8_t absolute_mode;       /* 0 = relative motion, 1 = absolute motion {G90, G91} */
  static uint8_t program_flow;
  static double feed_rate;              /* Millimeters/second */
  static double position[3];    /* Where the interpreter considers the tool to be at this point in the code */
  static uint8_t tool;
  static uint8_t plane_axis_0, plane_axis_1, plane_axis_2; // The axes of the selected plane
  static int read_double(char *line, //!< string: line of RS274/NGC code being processed
                       int *counter,       //!< pointer to a counter for position on the line 
                       double *double_ptr); //!< pointer to double to be read                  

  static int next_statement(char *letter, double *double_ptr, char *line, int *counter);
  static void select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2);
  static float to_millimeters(double value);

public:
  // Initialize the parser
  static void init();

  // Execute one block of rs275/ngc/g-code
  static uint8_t execute_line(char *line);

  // get the current logical position (in current units), the current status code and the unit mode
  static void get_status(double *_position, uint8_t *_status_code, int *_inches_mode, uint32_t *_line_number);

};

#endif
