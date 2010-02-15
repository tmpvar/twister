/*
  twister.c - module for buffering and executing smooth motion
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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/sleep.h>
#include "config.h"
#include "nuts_bolts.h"
#include "Twister.h"

/* A adjusted tangential table for a quadratic step sequence maximizing torque but retaining the correct
   angle of the magnetic field for each position
   Mathematica notation: Table[tt = t; 1023 - Floor[1023*(Abs[N[Sin[tt]/Cos[tt]]])^(1/1.8)], {t, -Pi/4, Pi/4, Pi/4/128}] */
const uint16_t __ATTR_PROGMEM__ abstan[] = 
{0, 7, 14, 21, 28, 35, 42, 48, 55, 62, 68, 75, 81, 88, 94, 100, 107, \
113, 120, 126, 132, 138, 145, 151, 157, 163, 169, 175, 182, 188, 194, \
200, 206, 212, 218, 224, 230, 236, 242, 248, 254, 260, 266, 272, 277, \
283, 289, 295, 301, 307, 313, 319, 325, 331, 337, 343, 349, 355, 361, \
367, 373, 378, 385, 391, 397, 403, 409, 415, 421, 427, 433, 439, 446, \
452, 458, 464, 471, 477, 483, 490, 496, 503, 509, 516, 522, 529, 536, \
543, 549, 556, 563, 570, 577, 584, 592, 599, 606, 614, 621, 629, 637, \
645, 653, 661, 669, 678, 686, 695, 704, 713, 722, 732, 741, 751, 762, \
772, 783, 794, 806, 819, 832, 845, 860, 876, 893, 912, 935, 963, \
1023, 963, 935, 912, 893, 876, 860, 845, 832, 819, 806, 794, 783, \
772, 762, 751, 741, 732, 722, 713, 704, 695, 686, 678, 669, 661, 653, \
645, 637, 629, 621, 614, 606, 599, 592, 584, 577, 570, 563, 556, 549, \
543, 536, 529, 522, 516, 509, 503, 496, 490, 483, 477, 471, 464, 458, \
452, 446, 439, 433, 427, 421, 415, 409, 403, 397, 391, 385, 378, 373, \
367, 361, 355, 349, 343, 337, 331, 325, 319, 313, 307, 301, 295, 289, \
283, 277, 272, 266, 260, 254, 248, 242, 236, 230, 224, 218, 212, 206, \
200, 194, 188, 182, 175, 169, 163, 157, 151, 145, 138, 132, 126, 120, \
113, 107, 100, 94, 88, 81, 75, 68, 62, 55, 48, 42, 35, 28, 21, 14, 7,
  0};

                                   
/* Arduino mega pwm pins:

  2 PE4 oc3b \
  3 PE5 oc3c  | Theta Stepper
  6 PH3 oc4a  |
  7 PH4 oc4b /

  8 PH5 oc4c  \
  11 PB5 oc1a  | Lambda Stepper
  12 PB6 oc1b  |
  13 PB7 oc1c /
*/

#define THETA_1A OCR3B
#define THETA_1B OCR3C
#define THETA_2A OCR4A
#define THETA_2B OCR4B

#define LAMBDA_1A OCR4C
#define LAMBDA_1B OCR1A
#define LAMBDA_2A OCR1B
#define LAMBDA_2B OCR1C

#define CYCLES_PER_UPDATE (F_CPU/UPDATE_FREQUENCY)
                                  
#define CONTROL_ACCELLERATION_PER_UPDATE (CONTROL_ACCELLERATION/UPDATE_FREQUENCY)
                      
struct motion_command {
  double delta[3];  // (mm along each axis) The displacement of the tool in xyz-space during this motion
  double magnitude; // (mm) The total length of this motion 
  double base_rate; // (mm/s) The typical feed rate during as much as possible of this motion
  double exit_rate; // (mm/s) The desired feed rate at the end of this motion
};

// motion control state variables
struct motion_command *current_command; // The motion currently in progress

double Twister::current_travel; // millimeters of travel so far in the current motion
double Twister::current_rate = 0.0; // the current speed
double Twister::target_rate = 0.0;
double Twister::tool_position[3]; // the current position of the tool in xyz-space
int Twister::exit_rate_mode; // True if the dynamic speed control is no longer maintaining base_speed, and have moved on to exit_rate
double Twister::exit_rate_checkpoint = 0.0; /* The point (in mm of the current motion) where acceleration for exit_rate must start, 
                                      given that the current feed rate is maintained. Automatically updated as the speed change. */

struct motion_command Twister::motion_buffer[MOTION_BUFFER_SIZE];

volatile int Twister::motion_buffer_head = 0;
volatile int Twister::motion_buffer_tail = 0;

/* Initialize the theta-lambda motion coordinator subsystem */
void Twister::init() { 
  // Set stepper pwm pins as outputs
  DDRE |= (1<<4)|(1<<5);
  DDRH |= (1<<3)|(1<<4)|(1<<5);
  DDRB |= (1<<5)|(1<<6)|(1<<7);
  
  // Set up 16 bit pwm timers
  
  // Settings for each timer used for stepper pwm:
  // WGMn3:0 == 0111 (10-bit fast PWM)
  // CNn2:0 == 001 (no prescaler)
  
  // Settings for each used pwm-channel: 
  // COMnc1:0 == 10

  TCCR1A = (1<<WGM10)|(1<<WGM11) | (1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1);
  TCCR1B = (1<<WGM12)|(1<<CS10);

  TCCR3A = (1<<WGM30)|(1<<WGM31) | (1<<COM3B1)|(1<<COM3C1);
  TCCR3B = (1<<WGM32)|(1<<CS30);

  TCCR4A = (1<<WGM40)|(1<<WGM41) | (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM42)|(1<<CS40);           
  
  // Timer 5 is set to CTC with OCR5A as MAX and OCIE5A interrupt enabled
  TCCR5A = 0;
  TCCR5B = (1<<WGM52)|(1<<CS50);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = CYCLES_PER_UPDATE; // Set the frequency of the stepper updater interrupt
}       

// Estimate the power of the jerk at the intersection of two motions
double Twister::estimate_jerk_magnitude(struct motion_command *command1, struct motion_command *command2) {
  // For our application jerk is half the phytagorean magnitude of the difference between the unit vector of the two motions
  // which gives us a value between 0 and 1.0 where 0 represents no change of direction and 1.0 is a full U-turn
  return(sqrt(
    square(command1->delta[0]/command1->magnitude - command2->delta[0]/command2->magnitude) +
    square(command1->delta[1]/command1->magnitude - command2->delta[1]/command2->magnitude))/2.0);
}
                                                             
/* Estimates the travel needed to accellerate to the target speed at a given speed using
   the configured constant accelleration CONTROL_ACCELLERATION 
   (units: speed in mm/s, result in mm) */
double Twister::estimate_acceleration_distance(double speed, double target_speed) {
  double min_speed, max_speed;
  if (speed < target_speed) {
    min_speed = speed;
    max_speed = target_speed;
  } else {
    min_speed = target_speed;
    max_speed = speed;
  }
  double time = (max_speed-min_speed)/CONTROL_ACCELLERATION_PER_UPDATE;   
  return((min_speed*time)+(CONTROL_ACCELLERATION_PER_UPDATE*square(time)/2));
}

/* Buffer a new motion. dx, dy and dz is the relative change in the position during this motion (mm). Feed rate 
   is the speed of the motion in mm/minute. This method also computes lookahead-data for the dynamic speed control. */
void Twister::push_motion(double dx, double dy, double dz, double feed_rate) {
  double magnitude = 
    sqrt(square(dx) + 
        square(dy) + 
        square(dz));
  // If this is a zero-length motion, just skip it
  if (magnitude == 0.0) { return; }

  // Calculate the next buffer head as it will stand after this 
  int next_buffer_head = (motion_buffer_head + 1) % MOTION_BUFFER_SIZE;
  // Block until there is room on the motion buffer
  while (next_buffer_head == motion_buffer_tail) { sleep_mode(); }

  // Configure the new motion
  struct motion_command *next_command;
  next_command = &motion_buffer[motion_buffer_head];
  next_command->delta[0]=dx;
  next_command->delta[1]=dy;
  next_command->delta[2]=dz;
  next_command->base_rate = (feed_rate/(60.0*UPDATE_FREQUENCY));
  next_command->magnitude = magnitude;
  next_command->exit_rate = 0;

  // Post lookahead-data for the intersection between the previous motion and this motion
  // Fetch the current last buffered command in order to post lookahead data for the dynamic speed control
  int last_command_position = motion_buffer_head-1;
  if (last_command_position < 0) { last_command_position = MOTION_BUFFER_SIZE; }
  struct motion_command *last_command = &motion_buffer[last_command_position];
  double intersection_jerk = estimate_jerk_magnitude(last_command, next_command);
  last_command->exit_rate =  ((next_command->base_rate+last_command->base_rate)/2)*(pow(1.0-intersection_jerk,3));

  // Advance the buffer-head
  motion_buffer_head = next_buffer_head;      
}         
   
// Called regularly to maintain a certain speed. If the speed is off, CONTROL_ACCELLERATION is used to 
// increase or decrease the current speed. The speed will never be lowered beneath MIN_SPEED.
// This method will also update the exit_rate_checkpoint when the speed changes.
void Twister::maintain_speed(double target_speed) {  
  if (fabs(target_speed-current_rate) <= CONTROL_ACCELLERATION_PER_UPDATE) {
    current_rate = target_speed;
  } else {
  
    if (target_speed < current_rate) {
      current_rate -= CONTROL_ACCELLERATION_PER_UPDATE;
    } else {
      current_rate += CONTROL_ACCELLERATION_PER_UPDATE;
    }

    // Estimate the new exit_rate_checkpoint unless we are allready in exit_rate_mode
    if (!exit_rate_mode) {
      exit_rate_checkpoint = current_command->magnitude -
        estimate_acceleration_distance(current_rate, current_command->exit_rate);    
    }
  }
  if (current_rate < MIN_SPEED) {
    current_rate = MIN_SPEED;
  }
}

/* A timer interrupt called UPDATE_FREQUENCY times per second. Calls Twister::update() */ 
SIGNAL(SIG_OUTPUT_COMPARE5A) {   
  Twister::update();
}

/* The workhorse of the module. It pops motion-commands from the buffer and executes them. */
void Twister::update() {
  if (current_command) {      
    double t = (1.0*current_travel/current_command->magnitude);
    double x = tool_position[0]+current_command->delta[0]*t;
    double y = tool_position[1]+current_command->delta[1]*t;
    double distance = sqrt(x*x+y*y);
    double theta = 2*asin(distance/(2*ARM_LENGTH));
    double lambda = theta/2-acos(y/distance)*signof(x);    
    set_nanostep_theta((uint16_t) round(theta*((100.0/M_PI)*256)));
    set_nanostep_lambda((uint16_t) round(lambda*((100.0/M_PI)*256)));
    
    if (current_travel >= current_command->magnitude) {
      tool_position[0] = tool_position[0] + current_command->delta[0];
      tool_position[1] = tool_position[1] + current_command->delta[1];
      tool_position[2] = tool_position[2] + current_command->delta[2];
      current_command = 0; 
      current_travel = 0;   
      exit_rate_mode = FALSE;
      exit_rate_checkpoint = 0;
    } else {                    
      
      // Is it time to forget about base_rate and move on to the exit-rate?
      if (!exit_rate_mode && (current_travel > exit_rate_checkpoint)) {
        exit_rate_mode = TRUE; // setting this lets the speed control forget about updating the exit_rate_checkpoint
        target_rate = current_command->exit_rate;
      }

      maintain_speed(target_rate);
      current_travel += current_rate; 
      // Are we done with this motion? Clamp travel to end-point of current motion
      if (current_travel > current_command->magnitude) { current_travel = current_command->magnitude; }
    }
  }
  
  // Pop a new motion command if there is no current command, but there are more in the buffer
  if (!current_command && (motion_buffer_head != motion_buffer_tail)) {
    // Pop and advance the buffer tail
    current_command = &motion_buffer[motion_buffer_tail];
    motion_buffer_tail = (motion_buffer_tail + 1) % MOTION_BUFFER_SIZE;		
    // Set new target rate and recompute exit_rate_checkpoint
    target_rate = current_command->base_rate;
    exit_rate_checkpoint = current_command->magnitude-estimate_acceleration_distance(current_rate, current_command->exit_rate);    
  }  
}

// Update PWM for the theta-stepper. A full cycle of microsteps runs from position 0 to 1024
void Twister::set_nanostep_theta(uint16_t position) {
  uint8_t sector = (position>>7 )%8;
  uint16_t tangent = pgm_read_word_near(&abstan[position&0xff]);
  // OCR0A = xcoil+, OCR0B = xcoil-, OCR2A = ycoil+, OCR2B = ycoil-
  switch (sector) {
    case 0: 
    THETA_1A = tangent; THETA_1B = 1023;
    THETA_2A = 0; THETA_2B = 1023;
    break;
    case 1:
    THETA_1A = 1023; THETA_1B = tangent;
    THETA_2A = 0; THETA_2B = 1023;
    break;
    case 2:
    THETA_1A = 1023; THETA_1B = 0;
    THETA_2A = tangent; THETA_2B = 1023;
    break;
    case 3:
    THETA_1A = 1023; THETA_1B = 0;
    THETA_2A = 1023; THETA_2B = tangent;
    break;
    case 4:
    THETA_1A = 1023; THETA_1B = tangent;
    THETA_2A = 1023; THETA_2B = 0;
    break;
    case 5: 
    THETA_1A = tangent; THETA_1B = 1023;
    THETA_2A = 1023; THETA_2B = 0;      
    break;
    case 6:
    THETA_1A = 0; THETA_1B = 1023;
    THETA_2A = 1023; THETA_2B = tangent;
    break;
    case 7: 
    THETA_1A = 0; THETA_1B = 1023;
    THETA_2A = tangent; THETA_2B = 1023;
    break;
  }
}

// Update PWM for the lambda-stepper. A full cycle of microsteps runs from position 0 to 1024
void Twister::set_nanostep_lambda(uint16_t position) {
  uint8_t sector = (position>>7 )%8;
  uint16_t tangent = pgm_read_word_near(&abstan[position&0xff]);
  // OCR0A = xcoil+, OCR0B = xcoil-, OCR2A = ycoil+, OCR2B = ycoil-
  switch (sector) {
    case 0: 
    LAMBDA_1A = tangent; LAMBDA_1B = 1023;
    LAMBDA_2A = 0; LAMBDA_2B = 1023;
    break;
    case 1:
    LAMBDA_1A = 1023; LAMBDA_1B = tangent;
    LAMBDA_2A = 0; LAMBDA_2B = 1023;
    break;
    case 2:
    LAMBDA_1A = 1023; LAMBDA_1B = 0;
    LAMBDA_2A = tangent; LAMBDA_2B = 1023;
    break;
    case 3:
    LAMBDA_1A = 1023; LAMBDA_1B = 0;
    LAMBDA_2A = 1023; LAMBDA_2B = tangent;
    break;
    case 4:
    LAMBDA_1A = 1023; LAMBDA_1B = tangent;
    LAMBDA_2A = 1023; LAMBDA_2B = 0;
    break;
    case 5: 
    LAMBDA_1A = tangent; LAMBDA_1B = 1023;
    LAMBDA_2A = 1023; LAMBDA_2B = 0;      
    break;
    case 6:
    LAMBDA_1A = 0; LAMBDA_1B = 1023;
    LAMBDA_2A = 1023; LAMBDA_2B = tangent;
    break;
    case 7: 
    LAMBDA_1A = 0; LAMBDA_1B = 1023;
    LAMBDA_2A = tangent; LAMBDA_2B = 1023;
    break;
  }
}

// Block execution until the motion buffer is empty
void Twister::synchronize() {
  while (motion_buffer_head != motion_buffer_tail) { sleep_mode(); }
}
