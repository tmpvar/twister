/*
  MotionControl.h - cartesian robot controller.
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

#ifndef MotionControl_h
#define MotionControl_h

#include <avr/io.h>
#include "nuts_bolts.h"

#define MC_MODE_AT_REST 0
#define MC_MODE_LINEAR 1
#define MC_MODE_ARC 2
#define MC_MODE_DWELL 3
#define MC_MODE_HOME 4

class MotionControl {
private: 
  MotionControl();  
  static double position[3]; // The current position of the tool in cartesian space
public:
  static void init();
  // Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
  // unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
  // 1/feed_rate minutes.
  static void line(double x, double y, double z, float feed_rate, int invert_feed_rate);
  // Prepare an arc. theta == start angle, angular_travel == number of radians to go along the arc,
  // positive angular_travel means clockwise, negative means counterclockwise. Radius == the radius of the
  // circle in millimeters. axis_1 and axis_2 selects the plane in tool space. 
  static void arc(double theta, double angular_travel, double radius, double linear_travel, int axis_1, int axis_2, 
    int axis_linear, double feed_rate, int invert_feed_rate);
  // Dwell for a couple of time units
  static void dwell(uint32_t milliseconds);
  static void go_home();
  // Block until all buffered motions have been executed
  static void synchronize();
  // Return current status 
  static int status();  
};

#endif
