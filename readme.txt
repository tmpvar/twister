Twister - An embedded rs274/ngc (g-code) interpreter and RepRap-controller for the AVR series of microcontrollers.

Goals: 
* Support GCode from common free Skeinforge right out of the box
* Use novel ultra-ultra-simple mechanical scheme using only hinge joints for horizontal positioning
* Smooth, silent and accurate motion using 1/256 detent-force-corrected microsteps and 10 bit pwm

Status:
* Runs on Arduino Mega / atmega1280
* GCode interpreter complete
* Linear interpolation machine control complete
* Buffered, non blocking, asynchronous stepping so the rest of the system is free to generate new steps and parse 
  g-code while the steppers are still steppin' 
* Dynamic speed control with lookahead to avoid jerky cornering
* Basic serial protocol complete

