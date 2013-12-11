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

#include "config.h"

class Twister {
private:
  Twister();
  static double current_travel; // millimeters of travel so far in the current motion
  static double current_rate; // the current speed
  static double target_rate;
  static double tool_position[3]; // the current position of the tool in xyz-space
  static int exit_rate_mode; // True if the dynamic speed control is no longer maintaining base_speed, and have moved on to exit_rate
  static double exit_rate_checkpoint; /* The point (in mm of the current motion) where acceleration for exit_rate must start, 
    given that the current feed rate is maintained. Automatically updated as the speed changes. */
  static struct motion_command motion_buffer[MOTION_BUFFER_SIZE];
  static uint8_t z_stepper_state;
  volatile static int motion_buffer_head;
  volatile static int motion_buffer_tail;  

  //static void set_nanostep_lambda(uint16_t position);
  static void set_nanostep_lambda(float angle);
  static void set_nanostep_theta(uint16_t position);
  static void maintain_speed(double target_speed);
  static double estimate_jerk_magnitude(struct motion_command *command1, struct motion_command *command2);
  static double estimate_acceleration_distance(double speed, double target_speed);
  
public:
  /* Initialize the theta-lambda motion coordinator subsystem */
  static void init();

  /* Buffer a new motion. dx, dy and dz is the relative change in the position during this motion (mm). Feed rate 
     is the speed of the motion in mm/minute. This method also computes lookahead-data for the dynamic speed control. */
  static void push_motion(double dx, double dy, double dz, double feed_rate);

  // Block execution until the motion buffer is empty
  static void synchronize();

  // Handles execution of motions. Called by a timer UPDATE_RATE times per second.
  static void update();
  static void move_z(double travel);
};

