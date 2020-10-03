/*
   OpTh.h 
   Copyright 2009 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   
   This file is part of the OpTh library for reading OpenTherm (TM)
   communications with Arduino.

   OpTh is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   OpTh is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with OpTh.  If not, see <http://www.gnu.org/licenses/>.

*/


/* This library is written for the Arduino Duemilanova and Arduino software version 0017.
 *
 * It's purpose is to make data transmitted with the OpenTherm (TM) protocol available
 * for other applications.
 *
 * This library may work with other hardware and/or software. YMMV.
 */

 

#ifndef OpTh_h
#define OpTh_h

//#include "WProgram.h"

#include "stdint.h"
#define VERSION 0.2               // Version of this library
//#define DATA_PIN 0                // Arduino pin where we receive the OT communication. Tied to INT0.

class OpTh {
   public:
      OpTh();                           // constructor
      void init(uint8_t extdp);         // initialise timer and input pin
	  void start();                     // enable interrupt and start listening
	  void stop();                      // disable interrupt and stop listening
      void setFrame(uint32_t frame);    // Set the frame value manually
	  void setFrameCnt(uint32_t cnt);	// Set the frameCnt manually
	  uint32_t getFrameCnt();		    // frameCnt (32 bit = 68 years i.c.o. 2 frames / sec ;)
	  uint32_t getUsSinceLast();		// returns the usSinceLast (1.19 hour for turn-over)
	  uint32_t getFrame();              // Get the frame value: all 32 bits
 	  uint8_t newFrame();               // Flag for new frame
	  uint8_t error();                  // error code. 0=no error, 1=interfal too short, 2=interfal too long, 3=invalid data, 4=parity error
	  uint8_t getMsgType();             // Return the type of message
      uint8_t getDataId();              // Return the data ID
	  uint16_t getDataValue();          // Return the data value
	  uint8_t getDataValueHB();         // Return the data value HB only
	  uint8_t getDataValueLB();         // Return the data value LB only
	  uint8_t getParity();              // Return the frame parity
      uint8_t isMaster();               // Returns 1 for master frame, 0 for slave frame
	  	  
   private:
      //char *_errorMsg;                  // can contain an error message (text)
      //uint8_t _dataPin;                 // Arduino pin where we read the signal
	  //uint8_t _newFrame;                // new Frame Flag
	  //uint8_t _error;                   // new error Flag
      //uint32_t _frame;                  // a frame contains FRAME_LENGTH - 2 bits (start/stop are discarded)
	  //uint32_t _frameCnt;               // frameCnt private
	  
};

#endif