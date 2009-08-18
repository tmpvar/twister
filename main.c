/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
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

#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "turntable.h"
#include "spindle_control.h"
#include "motion_control.h"
#include "gcode.h"
#include "serial_protocol.h"

#include "config.h"
#include "wiring_serial.h"

int main(void)
{
  beginSerial(BAUD_RATE);

  tt_init(); // initialize theta-lambda stepper driver subsystem
  mc_init(); // initialize motion control interface
  spindle_init(); // initialize spindle controller
  gc_init(); // initialize gcode-parser
  sp_init(); // initialize the serial protocol
  
  sei();
  
  for(;;){
    sleep_mode();
    sp_process(); // process the serial protocol
  }
  return 0;   /* never reached */
}
