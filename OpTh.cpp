/*
   OpTh.cpp 
   Copyright 2009 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl
   Update by Sybren de Jong in 2019

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


#include "OpTh.h"
#include "stdint.h"
#include <elapsedMillis.h>
#define VERSION 0.4




/* Variables used in interrupts */ 
elapsedMicros usCnt;    		// used by elapsedmilis example
volatile int8_t prefBit = 1;   	// used to determine new bit value
volatile int8_t dataReady = 0;  // indicates tha data is frame data
volatile uint32_t usSinceLast;  // interval since last falling edge
volatile int8_t bitCnt = 0;     // bit counter in the frame construction
volatile int8_t parityOdd = 0;  // parity check
volatile uint32_t tempFrame = 0;// incomplete frame while reading bits
volatile uint8_t DATA_PIN=0;    // Arduino pin where we read the signal
volatile uint8_t _newFrame=0;   // new Frame Flag
volatile uint8_t _error=0;      // error
volatile uint32_t _frame=0;     // a frame contains FRAME_LENGTH - 2 bits (start/stop are discarded)
volatile uint32_t _frameCnt=0;  // frameCnt private

#define T0 300   //spurious pulse
#define T1 850   //850 = 1 period boundary
#define T2 1250  //1-1.5 period boundary
#define T3 1760  //1.5-2 period boundary
#define T4 2350  //2     period boundary

/* * * * Static functions * * * */

/* Interrupt service routine falling edge */
static void ICACHE_RAM_ATTR _ISRfe() {
   
   if(usCnt > T0){            //no spurious pulse
		usSinceLast = usCnt;  //capture the interval in us
		usCnt = 0;            //reset ustimer
	}
				
   
  /*  Fuction table based on interval period and previous bit:
   
   Period:		PrefBit:		NewBit(s):
   1			0				0
   1			1				1
   1.5			0				1
   1.5			1				10
   2			0				10
   2			1				x (cannot occur)
   
   one OT period may be 900 - 1150 us
   1.5 =			   1350 - 1725 us
   2   =			   1800 - 2300 us
   
   timing with some marging:
   1   =  850 - 1250 us 
   1.5 = 1250 - 1760 us 
   2   = 1760 - 2350 us
   
   OT_errors:
   1 = period shorter than 1 -> invalid
   2 = period longer than 2 -> invalid
   3 = wrong bit in a period 2 -> invalid
   4 = parity error
   
   shorter or longer -> error */
   
    //Serial.println(dataReady);
	//Serial.println(usSinceLast);
	//Serial.println(_error);
	//Serial.println(_errorMsg);
	
	
	
   if (dataReady == 1) {
	   
	   if(usSinceLast >=T0 && usSinceLast < T1){ //period < 1
		   _error = 1;
		   dataReady = 0;  //ignore follow on edges until new frame
		} //period < 1
	   
	   if(usSinceLast >= T1 && usSinceLast < T2){ //1 period
		   tempFrame <<= 1;
		   tempFrame |= prefBit;
		   bitCnt++;
		   if(prefBit) parityOdd = !parityOdd;
	   } //1 period
	   
	   if(usSinceLast >=T2 && usSinceLast < T3){	  //1.5 period
			if(prefBit){
				prefBit 	= 0; // 0 is last bit of 10
				tempFrame <<= 2; // shift 2 bit
				tempFrame  |= 2; // add 10
				bitCnt     += 2; // bit counter +2
				parityOdd = !parityOdd;
			}  //1.5 period & 1
			else{
				prefBit     = 1;
				tempFrame <<= 1;
				tempFrame  |= 1;
				bitCnt++;
				parityOdd = !parityOdd;
			}  //1.5 period & 0
	   } //1.5 period
	   
	   if(usSinceLast >=T3 && usSinceLast < T4){ // 2 period
			if(prefBit){
				_error = 3;
				dataReady = 0;  //ignore follow on edges until new frame
			} //2 period & 1
			else{	//2 period icw prefBit = 0 cannot occur -> error!
				prefBit 	= 0;
				tempFrame <<= 2;
				tempFrame  |= 2;
				bitCnt 	   += 2;
				parityOdd = !parityOdd;
			} //2 period & 0
	   }// 2 period
			
		if(usSinceLast >=T4){ // > 2 period
			_error = 2;
			dataReady = 0;  //ignore follow on edges until new frame
		}// > 2 period						 	   
	  	   
	   //check frame is complete..., note that the start bit was shifted out....
	   if (bitCnt > 33){
		   if (parityOdd == 1) {
			   _frame = tempFrame;  // store measured frame in class private variable.
			   _frameCnt++;
			   _newFrame=1;
			}
			else{
			   _error = 4;	// 4 = parity error
			}
			dataReady = 0;  // stop fetching bits...
	   } // frame complete			
	   
   } //dataReady == 1
   
   else {	//data ready ==0    //edge was begin of the startbit, reset all data capture variables
	   if(usSinceLast > 20000){	//only when > 20 ms interval between last edge...
		   dataReady = 1;		//set dataReady to indicate edges are data	
		   tempFrame = 0;		//reset tempFrame
	       parityOdd = 0;		//reset parity
	       bitCnt 	 = 1;		//reset bitCnt
	       prefBit 	 = 1;	   	//startbit will be a 1
	   }  // if usSinceLast > 20000
	   
   } //dataReady == 0
   
}  // ISR falling edge



/* * * * End static functions * * * */


/* Class constructor.
 * 
 */
OpTh::OpTh() {
   //_dataPin = DATA_PIN;
}

/* Initialise Timer1 and set the pinmode for the pin that we
 * use to listen to the communication.
 */
void OpTh::init(uint8_t extdp) {
   //TCCR1A = 0 << COM1A1 | 0 << COM1A0;
   //TCCR1B = 0 << CS12 | 1 << CS11 | 1 << CS10;    // 16MHz clock with prescaler means TCNT1 increments every 4 us.
   //TIMSK1 = 0x00;
   DATA_PIN = extdp;								// Update data pin with external provided data pin
   pinMode(DATA_PIN, INPUT);						// ToDo: check ESP input pin levels
   digitalWrite(DATA_PIN, HIGH);                    // enable internal pull-up resistor
  
   _frame = 0;
   dataReady = 0;  // reset 
   usCnt = 0;      // reset usCnt
}

void OpTh::start() {
   attachInterrupt(digitalPinToInterrupt(DATA_PIN),_ISRfe,FALLING);
   dataReady = 0;  // reset 
   usCnt = 0;      // to avoid start capturing bits in the middle of a frame
}

void OpTh::stop() {
   detachInterrupt(digitalPinToInterrupt(DATA_PIN));
   //dataReady = 0;
   //usCnt = 0;      // to avoid start capturing bits in the middle of a frame
}


/* Accessors for internal data structures. */


/* Set the frame value manually. Use this for example when receiving the frame from another
 * source (e.g. radio transmitter) to be able to access its content. */
void OpTh::setFrame(uint32_t frame) {
   _frame = frame;
}

void OpTh::setFrameCnt(uint32_t cnt) {
   _frameCnt = cnt;
}

uint32_t OpTh::getFrameCnt() {
	return _frameCnt;
}

uint32_t OpTh::getUsSinceLast() {
	return usSinceLast;
}

uint32_t OpTh::getFrame() {
   _newFrame = 0;
   return _frame;
}

uint8_t OpTh::newFrame() {
   return _newFrame;
}

uint8_t OpTh::error() {
   uint8_t tmpError = _error;
   _error = 0;
   return tmpError;
}


/* Return message type (a three-bit number) as a byte. */
uint8_t OpTh::getMsgType() {
   uint32_t tempFrame2 = _frame;
   tempFrame2 >>= 28;                 // shift 28 bits into oblivion
   uint8_t msgType = tempFrame2 & 7;  // take only three bits (masking with B111)
   _newFrame = 0;
   return msgType;
}

/* Return data ID as a single byte. */
uint8_t OpTh::getDataId() {
   uint32_t tempFrame2 = _frame;
   tempFrame2 >>= 16;
   _newFrame = 0;
   return (uint8_t)tempFrame2;         // type casting
}

/* Return data value as an unsigned int. */
uint16_t OpTh::getDataValue() {
   _newFrame = 0;
   return (uint16_t)_frame;           // type casting
}

uint8_t OpTh::getDataValueHB() {
   _newFrame = 0;
   uint32_t tempFrame2 = _frame;
   tempFrame2 >>= 8;
   return (uint8_t)tempFrame2;           // type casting
}

uint8_t OpTh::getDataValueLB() {
   _newFrame = 0;
   return (uint8_t)_frame;           // type casting
}

/* Return parity as a single byte. */
uint8_t OpTh::getParity() {
   uint32_t tempFrame2 = _frame;
   tempFrame2 >>= 31;                 // shift 31 bits into oblivion
   _newFrame = 0;
   return tempFrame;
}

/* Return 1 if it's a master frame, 0 if it's a slave frame. */
uint8_t OpTh::isMaster() {
   if (getMsgType() <= 3) {
      return 1;
   }
   return 0;
}

// add variables
