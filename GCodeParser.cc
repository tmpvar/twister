/*
  gcode.c - rs274/ngc parser.
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

/* This code is inspired by the Arduino GCode Interpreter by Mike Ellery and the NIST RS274/NGC Interpreter
   by Kramer, Proctor and Messina. */

/* Intentionally not supported:
  - Canned cycles
  - Tool radius compensation
  - A,B,C-axes
  - Multiple coordinate systems
  - Evaluation of expressions
  - Variables
  - Multiple home locations
  - Probing
  - Override control
*/

/* 
   Omitted for the time being:
   group 0 = {G10, G28, G30, G92, G92.1, G92.2, G92.3} (Non modal G-codes)
   group 8 = {M7, M8, M9} coolant (special case: M7 and M8 may be active at the same time)
   group 9 = {M48, M49} enable/disable feed and speed override switches
   group 12 = {G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3} coordinate system selection
   group 13 = {G61, G61.1, G64} path control mode
*/

#include "GCodeParser.h"

extern "C" {
#include <stdlib.h>
#include <string.h>
#include "nuts_bolts.h"
#include <math.h>
#include "config.h"
#include "errno.h"
}

#include "MotionControl.h"
#include "serial_protocol.h"
#include "spindle_control.h"

#define NEXT_ACTION_DEFAULT 0
#define NEXT_ACTION_DWELL 1
#define NEXT_ACTION_GO_HOME 2

#define MOTION_MODE_RAPID_LINEAR 0 // G0 
#define MOTION_MODE_LINEAR 1 // G1
#define MOTION_MODE_CW_ARC 2  // G2
#define MOTION_MODE_CCW_ARC 3  // G3
#define MOTION_MODE_CANCEL 4 // G80

#define PATH_CONTROL_MODE_EXACT_PATH 0
#define PATH_CONTROL_MODE_EXACT_STOP 1
#define PATH_CONTROL_MODE_CONTINOUS  2

#define PROGRAM_FLOW_RUNNING 0
#define PROGRAM_FLOW_PAUSED 1
#define PROGRAM_FLOW_COMPLETED 2

#define SPINDLE_DIRECTION_CW 0
#define SPINDLE_DIRECTION_CCW 1

uint32_t GCodeParser::line_number;
uint8_t GCodeParser::status_code;
uint8_t GCodeParser::motion_mode;         /* {G0, G1, G2, G3, G38.2, G80, G81, G82, G83, G84, G85, G86, G87, G88, G89} */
uint8_t GCodeParser::inverse_feed_rate_mode; /* G93, G94 */
uint8_t GCodeParser::inches_mode;         /* 0 = millimeter mode, 1 = inches mode {G20, G21} */
uint8_t GCodeParser::absolute_mode;       /* 0 = relative motion, 1 = absolute motion {G90, G91} */
uint8_t GCodeParser::program_flow;
int GCodeParser::spindle_direction;
double GCodeParser::feed_rate;              /* Millimeters/second */
double GCodeParser::position[3];    /* Where the interpreter considers the tool to be at this point in the code */
uint8_t GCodeParser::tool;
int16_t GCodeParser::spindle_speed;         /* RPM/100 */
uint8_t GCodeParser::plane_axis_0, GCodeParser::plane_axis_1, GCodeParser::plane_axis_2; // The axes of the selected plane
  
#define FAIL(status) status_code = status;

void GCodeParser::select_plane(uint8_t axis_0, uint8_t axis_1, uint8_t axis_2) 
{
  plane_axis_0 = axis_0;
  plane_axis_1 = axis_1;
  plane_axis_2 = axis_2;
}

void GCodeParser::init() {
  feed_rate = DEFAULT_FEEDRATE;
  select_plane(X_AXIS, Y_AXIS, Z_AXIS);
  absolute_mode = TRUE;
}

float GCodeParser::to_millimeters(double value) {
  return(inches_mode ? value * INCHES_PER_MM : value);
}


// Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
// characters and signed floats (no whitespace).
uint8_t GCodeParser::execute_line(char *line) {
  int counter = 0;  
  char letter;
  double value;
  double unit_converted_value;
  double inverse_feed_rate = -1; // negative inverse_feed_rate means no inverse_feed_rate specified
  int radius_mode = FALSE;
  
  uint8_t absolute_override = FALSE;       /* 1 = absolute motion for this block only {G53} */
  uint8_t next_action = NEXT_ACTION_DEFAULT;         /* One of the NEXT_ACTION_-constants */
  
  double target[3], offset[3];  
  
  double p = 0, r = 0;
  int int_value;

  clear_vector(target);
  clear_vector(offset);

  line_number++;
  status_code = GCSTATUS_OK;
  
  /* First: parse all statements */
  
  if (line[0] == '(') { return(status_code); }
  if (line[0] == '/') { counter++; } // ignore block delete
  
  // Pass 1: Commands
  while(next_statement(&letter, &value, line, &counter)) {
    int_value = (int) trunc(value);
    switch(letter) {
      case 'G':
      switch(int_value) {
        case 0: motion_mode = MOTION_MODE_RAPID_LINEAR; break;
        case 1: motion_mode = MOTION_MODE_LINEAR; break;
        case 2: motion_mode = MOTION_MODE_CW_ARC; break;
        case 3: motion_mode = MOTION_MODE_CCW_ARC; break;
        case 4: next_action = NEXT_ACTION_DWELL; break;
        case 17: select_plane(X_AXIS, Y_AXIS, Z_AXIS); break;
        case 18: select_plane(X_AXIS, Z_AXIS, Y_AXIS); break;
        case 19: select_plane(Y_AXIS, Z_AXIS, X_AXIS); break;
        case 20: inches_mode = TRUE; break;
        case 21: inches_mode = FALSE; break;
        case 28: case 30: next_action = NEXT_ACTION_GO_HOME; break;
        case 53: absolute_override = TRUE; break;
        case 80: motion_mode = MOTION_MODE_CANCEL; break;
        case 90: absolute_mode = TRUE; break;
        case 91: absolute_mode = FALSE; break;
        case 93: inverse_feed_rate_mode = TRUE; break;
        case 94: inverse_feed_rate_mode = FALSE; break;
        default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
      }
      break;
      
      case 'M':
      switch(int_value) {
        case 0: case 1: program_flow = PROGRAM_FLOW_PAUSED; break;
        case 2: case 30: case 60: program_flow = PROGRAM_FLOW_COMPLETED; break;
        case 3: spindle_direction = 1; break;
        case 4: spindle_direction = -1; break;
        case 5: spindle_direction = 0; break;
        default: FAIL(GCSTATUS_UNSUPPORTED_STATEMENT);
      }            
      break;
      case 'T': tool = (uint8_t) trunc(value); break;
    }
    if(status_code) { break; }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (status_code) { return(status_code); }

  counter = 0;
  clear_vector(offset);
  memcpy(target, position, sizeof(target)); // target = position

  // Pass 2: Parameters
  while(next_statement(&letter, &value, line, &counter)) {
    int_value = (int) trunc(value);
    unit_converted_value = to_millimeters(value);
    switch(letter) {
      case 'F': 
      if (inverse_feed_rate_mode) {
        inverse_feed_rate = unit_converted_value; // seconds per motion for this motion only
      } else {
        feed_rate = unit_converted_value; // millimeters pr second
      }
      break;
      case 'I': case 'J': case 'K': offset[letter-'I'] = unit_converted_value; break;
      case 'P': p = value; break;
      case 'R': r = unit_converted_value; radius_mode = TRUE; break;
      case 'S': spindle_speed = (int16_t) value; break;
      case 'X': case 'Y': case 'Z':
      if (absolute_mode || absolute_override) {
        target[letter - 'X'] = unit_converted_value;
      } else {
        target[letter - 'X'] += unit_converted_value;
      }
      break;
    }
  }
  
  // If there were any errors parsing this line, we will return right away with the bad news
  if (status_code) { return(status_code); }
    
  // Update spindle state
  if (spindle_direction) {
    spindle_run(spindle_direction, spindle_speed);
  } else {
    spindle_stop();
  }
  
  // Perform any physical actions
  switch (next_action) {
    case NEXT_ACTION_GO_HOME: MotionControl::go_home(); break;
    case NEXT_ACTION_DWELL: MotionControl::dwell((uint32_t) trunc(p*1000)); break;
    case NEXT_ACTION_DEFAULT: 
    switch (motion_mode) {
      case MOTION_MODE_CANCEL: break;
      case MOTION_MODE_RAPID_LINEAR: case MOTION_MODE_LINEAR:
      MotionControl::line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], 
        (inverse_feed_rate_mode) ? inverse_feed_rate : feed_rate, inverse_feed_rate_mode);
      break;
      case MOTION_MODE_CW_ARC: case MOTION_MODE_CCW_ARC:
      // not supported yet
      break;
    }    
  }
  
  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  memcpy(position, target, sizeof(double)*3);
  return(status_code);
}

void GCodeParser::get_status(double *_position, uint8_t *_status_code, int *_inches_mode, uint32_t *_line_number) 
{
  int axis;
  if (inches_mode) {
    for(axis = X_AXIS; axis <= Z_AXIS; axis++) {
      _position[axis] = position[axis]*INCHES_PER_MM;
    }
  } else {
    memcpy(_position, position, sizeof(position));    
  }
  *_status_code = status_code;
  *_inches_mode = inches_mode;
  *_line_number = line_number;
}

// Parses the next statement and leaves the counter on the first character following
// the statement. Returns 1 if there was a statements, 0 if end of string was reached
// or there was an error (check state.status_code).
int GCodeParser::next_statement(char *letter, double *double_ptr, char *line, int *counter) {
  if (line[*counter] == 0) {
    return(0); // No more statements
  }
  
  *letter = line[*counter];
  if((*letter < 'A') || (*letter > 'Z')) {
    FAIL(GCSTATUS_EXPECTED_COMMAND_LETTER);
    return(0);
  }
  (*counter)++;
  if (!read_double(line, counter, double_ptr)) {
    return(0);
  };
  return(1);
}

int GCodeParser::read_double(char *line, //!< string: line of RS274/NGC code being processed
                     int *counter,       //!< pointer to a counter for position on the line 
                     double *double_ptr) //!< pointer to double to be read                  
{
  char *start = line + *counter;
  char *end;
  
  *double_ptr = strtod(start, &end);
  if(end == start) { 
    FAIL(GCSTATUS_BAD_NUMBER_FORMAT); 
    return(0); 
  };

  *counter = end - line;
  return(1);
}
