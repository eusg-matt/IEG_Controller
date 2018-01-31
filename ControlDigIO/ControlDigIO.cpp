/*ControlDigIO.cpp
*Library to control and report on the state of outputs driven by an 8-bit shift register.
*Control includes setting all outputs high or low and individual control of each output.
*Created By: Matt North (matt.north@stfc.ac.uk)
*Created On: 20-07-2016
*Version 0.3
*Version Date: 14-07-2017
*----------------------------------------------------------
*Revision History:
*Added SetAllOutputs() for diagnsotics
*Changed bit_number=3 to bit_number=0 for correct buffervalve() operation
*
*----------------------------------------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// 74HC595 output mappings using shiftout() LSBFIRST
//
// bit_number	74HC595		Physical Output	(PCB PC3577M/1)
//	   0		   Qh	     Buffer Valve
//	   1           Qg		 Pump Valve
//	   2 		   Qf		 Gas Valve
//     3           Qe        Fault LED
//     4           Qd        Single Shot LED
//	   5           Qc	     Start/Stop Flow LED
//	   6		   Qb	     Pump, Purge, Fill LED
//	   7 		   Qa		 Pump LED	
*/

#include "Arduino.h"
#include "ControlDigIO.h"

const byte latchPin = 11;		//SER pin
const byte clockPin = 12;		//SRCLK pin
const byte dataPin = 10; 		//RCLK pin
const byte pump_pb=7;			//Pump Push button
const byte fill_pb=8;			//Pump, Purge & Fill Push Button
const byte flow_pb=9;			//Flow Gas Push Button
const byte shot_pb=6;			//Buffer single shot push button
const int debounce_delay=500;	//Switch debounce delay time
byte control_byte = 0;
byte pb_flags = 0;


void pindec(){
	pinMode(latchPin, OUTPUT);
	pinMode(dataPin, OUTPUT);
	pinMode(clockPin, OUTPUT);
	pinMode(pump_pb, INPUT);
	pinMode(fill_pb, INPUT);
	pinMode(flow_pb, INPUT);
	pinMode(shot_pb, INPUT);
}

void ClearAllOutputs(){
	pindec();
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, 0);	//switch all OFF
    digitalWrite(latchPin, HIGH);  
}

void SetAllOutputs(){
	pindec();
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, 255);	//switch all ON
    digitalWrite(latchPin, HIGH);  
}

//Valve Control
byte SetValves_Default(){
	GasValve(0);
	BufferValve(0);
	PumpValve(0);
    return 0;
}

byte SetValves_Pump(){
	GasValve(0);
	BufferValve(0);
	PumpValve(1);
    return 1;
}

byte SetValves_CCRShot(){
      GasValve(0);
      BufferValve(1);
      PumpValve(0);
      return 2;
}

byte SetValves_PumpAll(){
      GasValve(0);
      BufferValve(1); 
      PumpValve(1);
      return 3;
}

byte SetValves_FillBuffer(){
      GasValve(1);
      BufferValve(0); 
      PumpValve(0);
      return 4;
}

byte SetValves_FillBuffer_PumpCCR(){
      GasValve(1);
      BufferValve(0);
      PumpValve(1);
      return 5;
}

byte SetValves_FlowGas(){
      GasValve(1);
      BufferValve(1);
      PumpValve(0);
      return 6;
}

void PumpValve(boolean control_state){
	pindec();
	byte bit_number = 1;
	if(control_state==1){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
		digitalWrite(latchPin, HIGH);
		}
	else{
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
	}
}

void GasValve(boolean control_state){
	pindec();
	byte bit_number = 2;
	if(control_state==1){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
		digitalWrite(latchPin, HIGH);
		}
	else{
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
	}
}

void BufferValve(boolean control_state){
	pindec();
	byte bit_number = 0;
	if(control_state==1){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
		digitalWrite(latchPin, HIGH);
		}
	else{
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
	}
}

//LED Control
void ClearAllLEDs(){
	pindec();
	for(byte bit_number = 3; bit_number<=7; bit_number++){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number)); //switch OFF
		digitalWrite(latchPin, HIGH);
	}
}

void ClearButtonLEDs(){
	pindec();
	for(byte bit_number = 4; bit_number<=7; bit_number++){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number)); //switch OFF
		digitalWrite(latchPin, HIGH);
	}
}

void SetAllLEDs(){
	pindec();
	for(byte bit_number = 3; bit_number<=7; bit_number++){
		digitalWrite(latchPin, LOW);
		shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number)); //switch ON
		digitalWrite(latchPin, HIGH);
	}
}

void Pump_LEDSet(){
	Pump_LED(1);
    Fill_LED(0);
    Flow_LED(0);
	Shot_LED(0);
}

void Fill_LEDSet(){
	Pump_LED(0);
    Fill_LED(1);
    Flow_LED(0);
	Shot_LED(0);
}

void Flow_LEDSet(){
	Pump_LED(0);
    Fill_LED(0);
    Flow_LED(1);
	Shot_LED(0);
}

void Shot_LEDSet(){
	Pump_LED(0);
    Fill_LED(0);
    Flow_LED(0);
	Shot_LED(1);
}

void Fault_LED(boolean control_state){
	pindec();
	byte bit_number = 3;
  if(control_state == 1){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
    digitalWrite(latchPin, HIGH);
  }
  else{
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
  }
}


void Shot_LED(boolean control_state){
	pindec();
	byte bit_number = 4;
  if(control_state == 1){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));  //switch ON
    digitalWrite(latchPin, HIGH);
  }
  else{
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
  }
}

void Flow_LED(boolean control_state){
	pindec();
	byte bit_number = 5;
  if(control_state == 1){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));  //switch ON
    digitalWrite(latchPin, HIGH);
  }
  else{
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
  }
}

void Fill_LED(boolean control_state){
	pindec();
	byte bit_number = 6;
  if(control_state == 1){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
    digitalWrite(latchPin, HIGH);
  }
  else{
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
  }
}

void Pump_LED(boolean control_state){
	pindec();
	byte bit_number = 7;
  if(control_state == 1){
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitSet(control_byte, bit_number));	//switch ON
    digitalWrite(latchPin, HIGH);
  }
  else{
	digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, LSBFIRST, bitClear(control_byte, bit_number));	//switch OFF
    digitalWrite(latchPin, HIGH);
  }
}


byte PushButton_State(){
  pindec();
  if(digitalRead(fill_pb)==HIGH){
    delay(debounce_delay);
    if(digitalRead(fill_pb)==HIGH){
      Fill_LEDSet();						//Light fill button LED
	  return 1;                             //Pump, Purge & Fill Exchange Gas into Sample Volume
    }
  }
  else if(digitalRead(pump_pb)==HIGH){
    delay(debounce_delay);
    if(digitalRead(pump_pb)==HIGH){
		Pump_LEDSet();						//Light pump button LED
        return 2;                           //Pump down Sample Volume
    }
  }
  else if(digitalRead(flow_pb)==HIGH){
    delay(debounce_delay);
    if(digitalRead(flow_pb)==HIGH){
      Flow_LEDSet();							//Light flow button LED
	  return 3;                             //Flow Exchange Gas to Sample Volume
    }
  }
  else if(digitalRead(shot_pb)==HIGH){
    delay(debounce_delay);
    if(digitalRead(shot_pb)==HIGH){
      Shot_LEDSet();							//Light single buffer shot button LED
	  return 4;                             //Single Buffer Shot of Exchange Gas to Sample Volume
    }
  }
  else{
    return 0;                               //Default State
  }
  
}