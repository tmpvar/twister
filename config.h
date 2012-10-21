/*
  config.h - configuration data for Twister
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

#ifndef config_h
#define config_h

#define VERSION "0.1b"

#define MM_PER_INCH (1.0/25.4)

#define RAPID_FEEDRATE 600.0 // in millimeters per minute
#define DEFAULT_FEEDRATE 600.0

#define MM_PER_Z_STEP (1.5/200)

#define BAUD_RATE 9600

// length of the theta-arm in mm
#define ARM_LENGTH (42.0)

// Set the update frequency of the stepper updater. Must be > F_CPU/65535 (== 244 @ 16Mhz)
#define UPDATE_FREQUENCY 300

/* The dynamic speed control always uses this accelleration when changing speeds. The accelleration
   applied is always the same - only the time taken to reach a new speed is adjusted. For max accuracy this 
   should be as high as possible without creating jerky movement.
   The unit is mm/s^2 (millimeters per second per second) */
#define CONTROL_ACCELLERATION 0.2  

/* 0 == no jerk reduction, 3 == Very soft corners */
#define JERK_REDUCTION 0.5

/* The minimum speed: to make sure we always reach the end of motions even if we normally would have come to a 
   full stop a micrometer or so before the actual target point. */
#define MIN_SPEED 0.005

/* The maximum number of motions that can be buffered. Must be <= 127 
   (each entry in the buffer consumes 28 bytes of RAM) */
#define MOTION_BUFFER_SIZE 50
                  
/* Define this to compile Twister with support for driving the z-axis without a stepper controller. An H-bridge
must be connected to port A pins 3-6 so that each pair of adjacent pins control one coil of the stepper. By 
commenting out this line Twister is compiled with support for a stepper controller on the z-axis using port a pin 1 
for direction and pin 0 for step pulses. See Twister::move_z for more information. */
#define USE_RAW_Z_AXIS_H_BRIDGE

#endif
