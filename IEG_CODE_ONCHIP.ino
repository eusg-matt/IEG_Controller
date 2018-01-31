#include <ControlDigIO.h>

//23/10/2017
//
//Tank vac interlock is no connected in shunt with pumping valve to inihbit switch voltage to open pump vaulve when tank vac is above threshold or powered off.
//Singlegauge is set to SP1_Low=1e-3 mBar, SP1_High=1.1e-3 mBar.
//New sample transducer added, 345mBar working range, 1.4 bar proof pressure.
//Due to I/O / circuit constraints. SMC vac gauges on buffer and sample are to be interfaced as NPN vac switches.
//Using spare pin6 and configuring A5 as digital pin, both configured with internal pullups to interface with NPN.
//Vac siwtch HIGH when pressure switch setting exceeded, LOW when not.

//This code controls exchange gas for a CCR in the following modes:
//  --Pump Purge & Fill - Purge Volume and provide exchange gas
//  --Pump - Remove exchange gas during elevated temperatures
//  --Flow Gas - Flow exchange gas whilst stick is being removed from cold CCR
//  --Single Shot - Add a single shot of buffer volume into the sample volume
//
//This controller can be operated in both remote and local mode. The former accepting Serial commands via RS232
//from a local PC, the latter using buttons on the HMI.
//
//Revision History:
//Removed all tank vc interlock code.
//A5 configured as digital input with pullup for buffer vac switch
//Input configured as digital input with pullup for sample vac switch
//change purge_pressure code to operate on siwtch state
//change single shot buffer pressure to operate on switch state
//Pump and purge function does no fail if pumping phase doesn;t reach target, this provides the required flexibility to deal with transducer measurement tolerances.

//Error codes added:
//---------Error Codes---------------//
//  0  No Error - Dormant State
//  1  Not used
//  2  Sample vol did not pump to vac target, sample volume leak detected.
//  3  Sample vol did not pump to vac target, no vaccum.
//  4  Buffer vol did not fill from gas source.
//  5  Sample vol not filled, fill iterations reached.
//  6  Sample vol not filled from gas source in time.
//  7  Sample vol did not pump to vac target in time.   
//  8  Buffer or Sample Vol Leaking.
//  9  Single shot did not increase the sample volume pressure / No He Source.
//  10 Manual stop of pump down process.
//  11 Vacuum tank interlock is not made.                                      ++NOT IMPLEMENTED++
//  12 Sample vol reached vac target, sample volume leak detected.
//  13 Transducer disconnected or sensor break.                                

//Changes to be tested:
// Pump mode now leaves the sample space pumping continuosly until another process is initiated.
// During pump down it gets suck while checking if it has taken too long to pump down, doesn't return out of the pump down function.
// Transdcuer and vac switch changes hardware and code changes.
// Check transducer break operation

//Valve and LED's are now controled via a shift register controlled by the ControlOutputs library
//Vac interlock added - Active Low.

//Tank vac interlock and abort button need a big improvement - tank vac if lost during pump isn't detected, button push abort isn't great.
//Serial abort command has been implemented

//Changes to be made:
//03/11/2017
//If sensor is not connected need to limit funtionalitity - curently only pump down function doesnt run.
//after reaching the pumping target the leak check is now done with the pump still running as the pump down process leaves pump on, fill leak check remains standard.
//process error light is illuminated in some cases when there is no error. error reporting though IOC is correct. 


//Package functions into libraries reducing the size and complexity of the main program file
//ValveStatus needs to be passed back by the IO library
//When new hardware designed, include monitoring the status of the tank vac ilk.
//Provide a mech for users to change the pump and fill targets, and timeouts through the IOC.
//Review memory usage / create memory map (what dynamic memory is there a memory useage issue in some functions?)
//Review potential time rollover issues.

//Arduino UNO R3
//String for Serial Comms Result
String DeviceID="IEG0";  //see deviceID list for instrument/equipment cross ref.
String SerialPacket;
//Operating Variables
int ValveStatus, ErrorStatus, OpMode=0;
//Transducer PinOuts & Variables
const int sample_transducer=A5;
int sample_pressure=0;
//Pin Outs for Switches, Indicators, Heater & Valves 
const byte sample_vac_switch=5;
const byte sample_pr_switch=13;
const byte buffer_vac_switch=A4;
boolean buffer_vac_state=0;
boolean sample_vac_state=0;
boolean sample_pr_state=0;
//Pumping time values
unsigned long pump_check_limit=30000; //30 seconds
const long pump_time_limit=360000; //6 minutes
//Fill time values
const long time_to_fill_CCR=120000; //2 minutes
const int time_to_fill_buffer=10000; //10 seconds
const int flow_gas_min_period=10000; //10 seconds
const int leak_check_period=10000; //10 seconds
//Number of purge cycles
const byte purge_cycles=2;
const byte max_fill_iterations=20;
boolean purge_failed=true;
//Pressure setpoints (in ADC values)
//Omega transducer, sample targets
const int fill_pressure=312;    //should be in the region of 50mbar from the hard vacuum - read from omega trasnducer
const int pump_target=211; //211 is equivalent to c. 5mBar +/- c. 5mBar
//SMC Vac Switch, buffer target
//He regulator at 8psi, flow at >10 l/min, room temperature, standard IVC tail. One buffer fill increases the IVC pressure by ~16mBar. 
boolean flow_flag=false;
const byte shift_enable = 4;    //Shift Register OE pin
//-------------------------------------------------------------------//
//Chip Setup
void setup(){
//Digital Input Declerations
  //Switch Inputs
  pinMode(sample_vac_switch, INPUT_PULLUP);  //normally high
  pinMode(sample_pr_switch, INPUT);  //AT SOME POINT HARDWARE SHOULD CHANGE SO THAT THIS INPUT IS DEALT WITH LIKE THE OTHERS WITH INTERNAL PULLUP NOT THROUGH OPTO.
  pinMode(buffer_vac_switch, INPUT_PULLUP);  //normally high
  //enable the shift register
  pinMode(shift_enable, OUTPUT);
  digitalWrite(shift_enable, LOW); 
//UART Setup
  Serial1.begin(9600);  //SERIAL_8N1 - 8 data bits, no parity, 1 stop bit
  while(!Serial1);
//Set a defualt state and wait 5s for the transducers to power up.
  SetValves_Default();
  ClearAllLEDs();
  OpMode=0;
  delay(5000);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++Main Program+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void loop(){

  //check_sensor
  sensor_break();
  
  //Ensure operational mode state is latched while flowing gas, OpMode is driven by the FlowGas() function
  if(OpMode!=3){
    OpMode=0;
  }

  //Light Fault LED if an error is reported.
  if(ErrorStatus!=0){
    Fault_LED(1);
  }
  else{
    Fault_LED(0);
  }

  //Poll HMI & Comms
  int HMICmd=PushButton_State();
  delay(100);
  int SerialCmd=GetSerialPacket();
  delay(100);

  //Pump, Purge & Fill Process
  if(HMICmd==1 || SerialCmd==1){

    ErrorStatus=0;
    OpMode=1;
    Fill_LEDSet();

    //Pump & Purge Process
    for(int i=0; i<purge_cycles; i++){
      if(PumpDownSampleVol()){              //PUMP
        if(PurgeSampleVolFromSource()){      //PURGE
          purge_failed=false;
        }
        else{
          //Sample Volume did not fill
          purge_failed=true;
          ClearAllLEDs();
          break;
        }
      }
      else{
        //Sample Volume did not pump down
        purge_failed=true;
        ClearAllLEDs();
        break;
      }
    }
    
    //Exchange gas fill process
    if(!purge_failed){   
      if(PumpDownSampleVol()){                                      //PRE-FILL PUMP
        if(FillSampleVolFromBuffer(fill_pressure)){                 //FILL
          if(!Leak_Check(sample_transducer, leak_check_period)){    //LEAK CHECK
            //Process ends well and exits
            SetValves_Default();
            ClearAllLEDs();
            return;
          }
          else{
            FlowGas();
            ErrorStatus=8;
          }
        }
        else{
         //Sample volume not filled, complete pump down sample volume which error checking then leave controller in constant pump down mode.
         PumpDownSampleVol();
         Pump_LEDSet();
        }
      }
    }
  }
     
     //----------------------------------Hot Stick - Furnace Mode-----------------------------------------------------------------------------------------------
     //Stick furnace can ony be operated inside a CCR when the exchange gas is removed otherwise heat transfer through the exchange gas may damage the CCR seal
     //In hot mode the controller evacuates the sample volume
     
  else if(HMICmd==2 || SerialCmd==2){                                                                                                //Remove exchange gas and heat sample

     ErrorStatus=0;
     OpMode=2;
     Pump_LEDSet();
     while(PumpDownSampleVol());     //stay looping in this function until the function returns false; process error or abort command
     ClearButtonLEDs();
  }
     
     //-------------------------------------Sample Change----------------------------------------------------------------------------------------------------------
     //During a sample change the exchange gas must flow until during the sample change process to ensure that ice does not form inside the CCR
     //Flowing a gas for a sample change is executed when the CCR/stick teperature is below the threshold defined by 'ccr_temp_limit'
     //If the CCR/stick temp is too high flow gas does not execute and flags an error.
  else if(HMICmd==3 || SerialCmd==3){
    
    //OpMode is managed in the flow gas function to ensure latching
    ErrorStatus=0;
    FlowGas();      //FLOW
    
  }
     //----------------------------------Single Shot Fill------------------------------------------------//
     //Fill buffer volume to nwp and then fill sample vol with a single shot from the buffer
  else if(HMICmd==4 || SerialCmd==4){

    ErrorStatus=0;
    OpMode=4;
    Shot_LEDSet();
    FillSampleVolSingleShot();  //BUFFER SHOT FILL
    ClearButtonLEDs();
  }    
   
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
//---------------------------Functions-----------------------------//
//-----------------------------------------------------------------//

//Function to check that serial packet meets standard and decodes packet
int GetSerialPacket(){

SerialPacket="\0";

int datalength=4; 
char header='&';
char footer='!';
char temp[datalength+1];

if(Serial1.available()>0){
  for(int n=0; n<64; n++){
    if(Serial1.read()==header){
      for(int i=0; i<=datalength; i++){
        char readChar=Serial1.read();
        if(i==datalength && readChar==footer){
          Serial1.println("&ACK!");
          SerialPacket=temp;
          if(SerialPacket=="OPM1"){
            Serial1.print('&');
            Serial1.print(DeviceID);
            Serial1.println(",OPM1!");
          return 1;                             //Pump, Purge & Fill Exchange Gas into Sample Volume
          }
          else if(SerialPacket=="OPM2"){
            Serial1.print('&');
            Serial1.print(DeviceID);
            Serial1.println(",OPM2!");
          return 2;                             //Pump down Sample Volume
          }
          else if(SerialPacket=="OPM3"){
            Serial1.print('&');
            Serial1.print(DeviceID);            
            Serial1.println(",OPM3!");
          return 3;                             //Flow Exchange Gas to Sample Volume
          }
          else if(SerialPacket=="OPM4"){
            Serial1.print('&');
            Serial1.print(DeviceID);            
            Serial1.println(",OPM4!");
          return 4;                             //Single Buffer Shot of Exchange Gas to Sample Volume
          }
          else if(SerialPacket=="STS0"){
            SendSerialResponse();
          return 5;                             //Request to send status message
          }
          else if(SerialPacket=="KILL"){
            Serial1.print('&');
            Serial1.print(DeviceID);            
            Serial1.println(",KILL!");
          return 6;                             //Abort message, functionality needs to be written into code, return 6
          }
          else{
          return 0;                             //Default State
          }
        }
        else if(i==datalength && readChar!=footer){
          Serial1.println("&NAK!");
          return 0;
        }
        else{
          temp[i]=readChar;
          temp[i+1]='\0';
        }
      }
    }
  }
}

}

//mods to remove the buffer transducer and include the bugger and sample vac switch states
void SendSerialResponse(){
    buffer_vac_state = digitalRead(buffer_vac_switch);
    sample_vac_state = digitalRead(sample_vac_switch);
    sample_pr_state = digitalRead(sample_pr_switch);
    sample_pressure=modeFilter(sample_transducer, 100);
    //These two function calls add a 17.6ms delay before the following serial message is sent

    Serial1.print('&');
    Serial1.print(DeviceID);
    Serial1.print(',');
    Serial1.print("OPM");
    Serial1.print(OpMode);
    Serial1.print(',');
    Serial1.print("VST");
    Serial1.print(ValveStatus);
    Serial1.print(',');
    Serial1.print("ERR");
    Serial1.print(ErrorStatus);
    Serial1.print(',');
    Serial1.print("BPH");
    Serial1.print(buffer_vac_state);
    Serial1.print(',');
    Serial1.print("SPL");    
    Serial1.print(sample_vac_state);
    Serial1.print(',');   
    Serial1.print("SPH");
    Serial1.print(sample_pr_state);
    Serial1.print(',');
    Serial1.print("SPR");
    Serial1.print(sample_pressure);
    Serial1.println('!');
}

//-----------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------------//
//Function to pump down the sample volume; includes process error checks :
//No pump connected
//Leak in pumping volume
//Inadequate pumping capacity
//if pump did not reach target in given time it continues to pump and returns a true state.
boolean PumpDownSampleVol(){
  unsigned long start_time;      //absolute time the pump valve has been open

  SetValves_Pump();                                                      //Open pump valve START PUMPING VOLUME DOWN
  start_time = millis();                                                 //Pump Start TimeStamp
  
  //Vacuum pressure not below SMC threshold
  while(!digitalRead(sample_vac_switch)){
    //poll status of HMI and serial for an abort instruction
    int serial_abort_check = GetSerialPacket();
//    Serial1.println("check for abort1");
    delay(500);
    
    if(sensor_break()){
      SetValves_Default();
      return false;
    }
    
    if(serial_abort_check==6 || PushButton_State()==2){   //Abort pump if recieve a 'KILL' serial message or pump button pushed on HMI
      SetValves_Default();                                //Close all valves (stop pumping volume)
      ErrorStatus=10;                                     //Set Error Code
      return false;                                       //Return from function
    }
    //if pump check time limit is reached flag error
//    Serial1.println("pump check limit");
//    delay(500);
    if(millis()-start_time >= pump_check_limit){
//        Serial1.println(start_time-millis());
//        delay(500);
        SetValves_Default();                                              //Close all valves (Stop pumping volume)
        ErrorStatus=3;                                                    //Set error code
        return false; 
    }
  }

  //Vac pressure below SMC threshold - system pumping down
  while(digitalRead(sample_vac_switch)){
    //check for abort commands
    int serial_abort_check = GetSerialPacket();
//    Serial1.println("check for abort2");
    delay(500);
    
    if(sensor_break()){
      SetValves_Default();
      return false;
    }
    
    if(serial_abort_check==6 || PushButton_State()==2){   //Abort pump if recieve a pump serial message or button push
      SetValves_Default();                                //Close all valves (stop pumping volume)
      ErrorStatus=10;                                     //Set Error Code
      return false;                                       //Return from function
    }    
    
    //pumping target reached
    if(modeFilter(sample_transducer, 100) <= pump_target){
//      SetValves_Default();                                      //+++++++++++++++++++++++++++++++++recently commentd out
      if(Leak_Check(sample_transducer, leak_check_period)){   //check for a leak in the sample volume
//        Serial1.println("leak, flow gas");
        FlowGas();                                            //leak is found flow gas
        ErrorStatus=2;
        return false;
        }
      else{
//        Serial1.println("success!");
        SetValves_Pump();
        return true;
        }
      }
    
    //exceeded the time limit
    if(millis()-start_time>=pump_time_limit){
      SetValves_Default();                                    //close all valves; stop pumping sample vol                                           
      if(Leak_Check(sample_transducer, leak_check_period)){   //check for a leak in the sample volume
//        Serial1.println("leak, flow gas");
        FlowGas();                                            //leak is found flow gas
        ErrorStatus=2;                                        //set error code
        return false;                                         //retunr form function
        }
      else{
//        Serial1.println("pump forever!");
        SetValves_Pump();                                     //leak is not found pump until further command
        ErrorStatus=7;                                        //set error code
        return true;                                         //return from function     
        }  
    }
  }
}

//Function to fill the buffer volume from gas bottle
//
//Future Development:
//Need to prevent over pressurisation and what to do if overpressurised - add 1.5Bar PRV to buffer (what is used on CCR sample volume?
boolean FillBufferVolume(){

//  ErrorStatus=0;              //reset error code  
  long start_time = millis();  //timestamp for process
  
  while(digitalRead(buffer_vac_switch)){   //check status of vac switch

    SetValves_FillBuffer();    //open valve to fill buffer
    GetSerialPacket();          //check serial commands
    delay(500);
      
    if(millis()-start_time>=time_to_fill_buffer){  //timeout process
      SetValves_Default();      //close all valves; stop filling buffer
      ErrorStatus=4;        //set error
      return false;    //return from function
    }
  }
  SetValves_Default();  //close all valves; stop filling buffer
  return true;  //return from function
}

//aditional comments added
boolean FillSampleVolSingleShot(){
  
  ErrorStatus=0;        //clear error status
  boolean state=false;  //clear function status flag

  if(sensor_break()){
    return false;
  }
  
  if(FillBufferVolume()){
    int pressure_check=modeFilter(sample_transducer, 100);  //check sample pressure
    SetValves_CCRShot();  //set valves to move gas from buffer volume to smaple volume
    
    //wait some time and check for serial data
    delay(1000);
    GetSerialPacket();
    delay(500);
    
    //close all valves
    SetValves_Default();
    
    //check that pressure in sample volume has increased
    if(modeFilter(sample_transducer, 100) > pressure_check || digitalRead(sample_vac_switch)){
      return true;  //return from function
    }
    else{
      ErrorStatus=9;  //set error status
      return false;  //return from function
    }
  }   
}

//Function to fill the sample volume from buffer
//
//Future Development:
//Need to prevent over pressurisation and what to do if overpressurised - add a PRV similar to that used on the CCR sample volume
boolean FillSampleVolFromBuffer(int target_pressure){
  
  //reset variables for function
//  ErrorStatus=0;
  boolean state=false;
  int loop_cnt = 0;

  if(sensor_break()){
    return false;
  }
  
  while(modeFilter(sample_transducer, 100) < target_pressure){
    
    FillSampleVolSingleShot();
        
    loop_cnt++;
    
    if(loop_cnt==max_fill_iterations){
      SetValves_Default();
      ErrorStatus=5;
      return false;
    }
  }
  
  return true;
  
}

//modified to operate with vac switch rather than transducer
boolean PurgeSampleVolFromSource(){
  
//  ErrorStatus=0;
  boolean state=false;
  
  long start_time = millis();

  //if gas source is not connected return with error
  if(!FillBufferVolume()){
    SetValves_Default();  //close all valves; stop filling sample vol
    ErrorStatus=9;        //set error register
    return state=false;    //return from function
  }

  //complete this loop while the pressure switch is under pressure i.e. sample volume not filled.
  while(!digitalRead(sample_pr_switch)){
    
    SetValves_FlowGas();                      //set valves to flow gas into sample volume
    GetSerialPacket();                       //check serial port while flowing
    delay(500);

    if(millis()-start_time>=time_to_fill_CCR){  //if timed out then return with error
      SetValves_Default();  //close all valves; stop filling sample vol
      ErrorStatus=6;        //set error register
      return state=false;    //return from function
      }
    }

  //sample volume is indicated full close valves and return.
  SetValves_Default();  //close all valves; stop filling
  return state=true;  //return from function
}

boolean FillSampleVolFromSource(int target_pressure){
  
//  ErrorStatus=0;  
  boolean state=false;
  
  long start_time = millis();

  if(sensor_break()){
    return false;
  }
  
  while(modeFilter(sample_transducer, 100) < target_pressure){
    SetValves_FlowGas();
//    GetSerialPacket();
//    delay(500);
    if(millis()-start_time>=time_to_fill_CCR){
      SetValves_Default();
      ErrorStatus=6;
      return state=false;
    }
  }
  SetValves_Default();
  return state=true;
}

//Checks for leak in a volume by acquiring the signal twice over a defined period and making a comparison.
//revised to include the ability for the syste4m to resposnd to comms messages. (No more NaN!)
boolean Leak_Check(int transducer, int period){
  
//  Serial1.println("leak check");
  
//  ErrorStatus=0;
  boolean state = false;

  if(sensor_break()){
    return false;
  }

  int first_pressure_check = modeFilter(transducer, 100);

  long wait_time = millis() + period;
  
  while(millis()-wait_time < 0 ){
    GetSerialPacket();
    delay(500);
  }
  
  int second_pressure_check = modeFilter(transducer, 100);
  
  if(second_pressure_check<(first_pressure_check-10)){    //if pressure as decreased by atleast c.10mbar then a leak has been detected
    ErrorStatus=8;
    
    return state=true;
  }
  else{
    return state=false;
  }
}

//Flows gas until flow gas command/button is sent/pressed.
void FlowGas(){

//  ErrorStatus=0;
  Flow_LEDSet();
  
  if(flow_flag==false){
    if(FillBufferVolume()){     //Check the gas source by filling into the buffer before opening valves to flow
      OpMode=3;
      SetValves_FlowGas();
      flow_flag=true;
      return;
    }
    else{
      OpMode=0;
      ClearButtonLEDs();
      SetValves_Default;
      flow_flag=false;
    }
  }
  else{
    OpMode=0;
    ClearButtonLEDs();
    SetValves_Default();
    flow_flag=false;
  }
}

//Function to find mode value of 100 samples points
//Taken and modified from http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/
//Modified to provide a method for changing the size of the data set and the sampling period
int modeFilter(int sensorpin, int num_reads){
   // read multiple values and sort them to take the mode
   int sortedValues[num_reads]; //integer array
   
   //Initialise Array to zeros
   memset(sortedValues,0,sizeof(sortedValues));
   
   for(int i=0;i<num_reads;i++){
     int value = analogRead(sensorpin);
     int j;
     if(value<sortedValues[0] || i==0){
        j=0; //insert at first position
     }
     else{
       for(j=1;j<i;j++){
          if(sortedValues[j-1]<=value && sortedValues[j]>=value){
            // j is insert position
            break;
          }
       }
     }
     for(int k=i;k>j;k--){
       // move all values higher than current reading up one position
       sortedValues[k]=sortedValues[k-1];
     }
     sortedValues[j]=value; //insert current reading
   }
   
   //return scaled mode of 10 values
   int returnval = 0;
   for(int i=num_reads/2-5;i<(num_reads/2+5);i++){
     returnval +=sortedValues[i];
   }
   
   return returnval/10;
}

//function to check that sample transucer is connected
//transducer is 4-20mA so a low or non adc reading is an error state.
boolean sensor_break(){
  if(modeFilter(sample_transducer, 10) >= 100){
      if(ErrorStatus == 13){
        ErrorStatus = 0; 
        return false;
      }
      else{
        return false;
      }
  }
  else{
    ErrorStatus = 13;
    return true;
  }
}
