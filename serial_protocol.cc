/*
  serial_protocol.c - the serial protocol master control unit
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
#include "config.h"
#include <math.h>
#include "nuts_bolts.h"
}

#include "libraries/HardwareSerial.h"
#include "serial_protocol.h"
#include "GCodeParser.h"

#define LINE_BUFFER_SIZE 128

char line[LINE_BUFFER_SIZE];
uint8_t char_counter;

void prompt() {
  Serial.print("ok\r\n");
  char_counter = 0;
}

void print_result() {
  double position[3];
  int inches_mode;
  uint8_t status_code;
  uint32_t line_number;
  int i; // loop variable
  GCodeParser::get_status(position, &status_code, &inches_mode, &line_number);
  Serial.print("\r\n[ ");  
  for(i=X_AXIS; i<=Z_AXIS; i++) {
    Serial.print((long int) trunc(position[i]*100));
    Serial.print(' ');
  }
  Serial.print(']');
  Serial.print('@');
  Serial.print(line_number);
  Serial.print(':');
  switch(status_code) {
    case GCSTATUS_OK: Serial.print("0 OK\r\n"); break;
    case GCSTATUS_BAD_NUMBER_FORMAT: Serial.print("1 Bad number format\r\n"); break;
    case GCSTATUS_EXPECTED_COMMAND_LETTER: Serial.print("2 Expected command letter\r\n"); break;
    case GCSTATUS_UNSUPPORTED_STATEMENT: Serial.print("3 Unsupported statement\r\n"); break;
    case GCSTATUS_MOTION_CONTROL_ERROR: Serial.print("4 Motion control error\r\n"); break;
    case GCSTATUS_FLOATING_POINT_ERROR: Serial.print("5 Floating point error\r\n"); break;
  }
}

void sp_init() 
{
  Serial.begin(BAUD_RATE);
  
  Serial.print("\r\nTwister ");
  Serial.print(VERSION);
  Serial.print("\r\n");  
  prompt();
}

void sp_process()
{
  char c;
  while(Serial.available()) 
  {
    c = Serial.read();
    if((char_counter > 0) && ((c == '\n') || (c == '\r'))) {  // Line is complete. Then execute!
      line[char_counter] = 0;
      Serial.print('"');
      Serial.print(line);
      Serial.print('"');              
      Serial.print("\r\n");
      GCodeParser::execute_line(line);
      char_counter = 0;
      prompt();
    } else if (c <= ' ') { // Throw away whitepace and control characters
    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
      line[char_counter++] = c-'a'+'A';
    } else {
      line[char_counter++] = c;
    }
  }
}

