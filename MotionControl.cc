/*
  motion_control.c - cartesian robot controller.
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

/* The structure of this module was inspired by the Arduino GCode_Interpreter by Mike Ellery. The arc
   interpolator written from the information provided in the Wikipedia article 'Midpoint circle algorithm'
   and the lecture 'Circle Drawing Algorithms' by Leonard McMillan. 
   
   http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
   http://www.cs.unc.edu/~mcmillan/comp136/Lecture7/circle.html
*/

#include <avr/io.h>
#include "config.h"
#include "motion_control.h"
#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include "nuts_bolts.h"
#include "twister.h"
#include "wiring_serial.h"

#define ONE_MINUTE_OF_MICROSECONDS 60000000.0      
#define STEPS_PER_SECOND 500
#define STEPS_PER_MINUTE (STEPS_PER_SECOND*60)
#define STEPS_PER_REVOLUTION 1600

double mc_position[3]; // The current mc_position of the tool in cartesian space

void mc_init()
{
  // no initialization needed
}

void mc_dwell(uint32_t milliseconds) 
{
  // not supported
}

// Buffer linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// 1/feed_rate minutes.
void mc_line(double x, double y, double z, float feed_rate, int invert_feed_rate)
{                   
  double delta[3]; 

  delta[0] = x-mc_position[0];
  delta[1] = y-mc_position[1];
  delta[2] = z-mc_position[2];

  tt_push_motion(delta[0],delta[1],delta[2],feed_rate);
  mc_position[0] = x; mc_position[1] = y; mc_position[2] = z;  
}


// Execute an arc. theta == start angle, angular_travel == number of radians to go along the arc,
// positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
// circle in millimeters. axis_1 and axis_2 selects the circle plane in tool space. Stick the remaining
// axis in axis_l which will be the axis for linear travel if you are tracing a helical motion.
// ISSUE: The arc interpolator assumes all axes have the same steps/mm as the X axis.
void mc_arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
  int axis_linear, double feed_rate, int invert_feed_rate)
{  
  // not implemented
}

void mc_go_home()
{
  // not implemented
}

int mc_status() 
{
  // not actually implemented
  return(MC_MODE_AT_REST);
}

// bock execution until all buffered motion is completed
void mc_sync() {
  tt_sync();
}
