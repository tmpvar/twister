/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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

extern "C" {
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "config.h"
}

#include "libraries/HardwareSerial.h"
#include "GCodeParser.h"
#include "serial_protocol.h"
#include "MotionControl.h"
#include "spindle_control.h"
#include "Twister.h"
#include "extruder.h"

int main(void)
{
  Serial.begin(BAUD_RATE);
  Serial2.begin(38400);

  MotionControl::init(); // initialize motion control interface
  Twister::init(); // initialize theta-lambda stepper driver subsystem
  
  spindle_init(); // initialize spindle controller
  GCodeParser::init(); // initialize gcode-parser
  sp_init(); // initialize the serial protocol
  init_tool(0); //  initialize extruder
  
  sei();

  // set_tool_temp(0, 250);
  // set_motor1_pwm(0, 240);
  // toggle_motor1(0, true, 1);
  
  //exercise_motors();
  print_stats();
  
  for(;;){
    sleep_mode();
    sp_process(); // process the serial protocol
  }
  return 0;   /* never reached */
}
