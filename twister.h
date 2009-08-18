/*
  twister.h - module for buffering and executing smooth motion
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

/* Initialize the theta-lambda motion coordinator subsystem */
void tt_init();

/* Buffer a new motion. dx, dy and dz is the relative change in the position during this motion (mm). Feed rate 
   is the speed of the motion in mm/minute. This method also computes lookahead-data for the dynamic speed control. */
void tt_push_motion(double dx, double dy, double dz, double feed_rate);

// Block execution until the motion buffer is empty
void tt_sync();
