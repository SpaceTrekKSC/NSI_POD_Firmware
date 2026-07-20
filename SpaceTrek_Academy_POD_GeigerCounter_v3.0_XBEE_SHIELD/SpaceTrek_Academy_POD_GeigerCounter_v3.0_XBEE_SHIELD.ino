/*
 * Space Trek Academy
 * Near Space Investigation POD
 * Geiger Counter example for an Arduino Uno or MEGA 2560 with
 * an XBEE shield attached.  Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then
 * replace the shield to test with the Flight Computer (FC).
 *
 * This example counts fast digital pulses using a hardware INTERRUPT.
 * It works with any device that produces short digital pulses, such
 * as a Geiger tube, an anemometer, or a reed switch.  The pulse-source
 * output is wired to D2 (an interrupt pin on the Uno) with grounds
 * common.  Every countTime window (60 s) the pulses are latched and a
 * rolling average over the last 15 windows is computed.
 *   val_01 = counts in the last 60 s window (counts per minute)
 *   val_02 = rolling average counts over up to 15 windows
 *
 * Author: Andrew Gafford
 * Date: Mar. 20th, 2025
 *
 */

//Included Libraries
#include <Arduino.h>
//Include any additional libraries needed here
//No external library is needed for this example.

//settings
#define PODID         1                 //Must be 1, 2 or 3.  This is the PODID that this experiment will resond to when the FC asks for pod data
#define maxMessage    96                //maximum number of bytes that can be in the message sent to the FC


//Add any custom #define settings your program needs here
#define geigerPin     2                 //Digital pin the pulse source is connected to (must be an interrupt pin: D2 or D3 on the Uno)
#define countTime     60000             //The length of a counting window in miliseconds (60000 => counts-per-minute)
#define numCounts     15                //The number of windows to keep for the rolling average


//================Special Bytes=====================
//These are used for the communication protocol with the FC
//Do NOT make any changes here
#define specialByte 250
#define ACKMarker 251
#define slaveStartMarker 252
#define slaveEndMarker 253
#define masterStartMarker 254
#define masterEndMarker 255
//=====================================================




//Setup any sensor or device objects here depending on what libraries you are using
//No sensor object is needed; the pulses are counted directly by the interrupt.




//=========Variables Used for FC Communication=========
//These are used for the communication protocol with the Flight Computer (FC)
//Do NOT make any changes here
byte bytesRecvd = 0;                  //not used currently
byte dataSentNum = 0;                 // the transmitted value of the number of bytes in the package i.e. the 2nd byte received
byte dataRecvCount = 0;               //not used currently
byte numDataBytes = 0;                //the number of data bytes to send to the FC.  Currently it is always set to the same number but future versions may change that

byte dataRecvd[maxMessage];           //not used currently
byte dataSend[maxMessage];            //a buffer for the data to send to the FC
byte encodeBuffer[maxMessage];        //a buffer for encoding the data given the special bytes that are reserved for communication protocol

byte dataSendCount = 0;               // the number of 'real' bytes to be sent to the PC
byte dataTotalSend = 0;               // the number of bytes to send to PC taking account of encoded bytes

boolean returnData = false;           //a flag for if need to return data to the FC
boolean reciveCommand = false;        //a flag for if recived command from FC

byte xbeeInBuffer = 0;                //NSI fix: was char = "" (stored a garbage pointer byte that could match another POD's id)               //a buffer for data recived on the XBEE
byte inCharBuffer = 0;                //a buffer to store a recived byte

//POD data to send to flight computer
byte data[40];                        //A buffer to store ten 4 byte floating point numbers

//variables to store sensor data in
union u_float{                        //unions are used to access the data as floating points or as each of the four bytes individually
  float value;
  byte bytes[4];
} val_01;                             //val_01

union u_float val_02;                 //and so on...
union u_float val_03;                 //..
union u_float val_04;                 //.
union u_float val_05;
union u_float val_06;
union u_float val_07;
union u_float val_08;
union u_float val_09;
union u_float val_10;
//==================================================


//User variables
uint32_t timer = 0;                       //a variable for the timer
volatile uint16_t count = 0;              //pulses counted since the last window (updated inside the ISR)
float average = 0.0;                      //rolling average of counts over the buffered windows
uint16_t countBuffer[numCounts] = {0};    //ring buffer holding the last numCounts window totals
bool bufferFull = false;                  //true once the ring buffer has wrapped at least once
uint32_t numReadings = 0;                 //how many windows have been recorded before the buffer filled
uint16_t bufferLocation = 0;              //current write position in the ring buffer

void setup(){
  Serial.begin(57600);                    //Turns on serial port that will talk to the FC via the XBEE

  //setup any pinMode() that are needed for your experiment
  pinMode(geigerPin, INPUT_PULLUP);       //pulse source pulls this pin LOW on each event

  //Setup or start any sensors or devices you have that need to be setup here
  attachInterrupt(digitalPinToInterrupt(geigerPin), geigerISR, FALLING);   //count every falling edge


  //set val_XX variables to 0.0
  val_01.value = 0.0;
  val_02.value = 0.0;
  val_03.value = 0.0;
  val_04.value = 0.0;
  val_05.value = 0.0;
  val_06.value = 0.0;
  val_07.value = 0.0;
  val_08.value = 0.0;
  val_09.value = 0.0;
  val_10.value = 0.0;

  timer = millis();                 //set the timer variable to the current number of miliseconds that have passed since the processor startted
                                    //It is good to do this at the end of the setup() function so the timer starts just befor loop() starts executing
}

void loop() {
  //USE A TIMER!!!
  //Do not do any blocking commands in this program like delay()!!!
  //The program needs to be able to get to the part where it checks
  //for a command from the FC. If you put large delays here it will
  //cause timing problems. The delay() command is a bad command.

  if((millis() - timer) >= countTime){            //check if a full counting window has passed
    timer = millis();                             //reset the timer to current cpu millis()

    countBuffer[bufferLocation++] = count;
    if(bufferLocation == numCounts){ bufferLocation = 0; bufferFull = true; }
    average = 0.0;
    if(bufferFull){ for(int i=0;i<numCounts;i++){ average += float(countBuffer[i]); } average = average/float(numCounts); }
    else { numReadings++; for(int i=0;i<numReadings;i++){ average += float(countBuffer[i]); } average = average/float(numReadings); }
    val_01.value = float(count);
    val_02.value = average;
    count = 0;
  }



  //===========XBEE FC Communication=========================
  // DO NOT MAKE ANY CHNAGES
  //XBEE Serial...
  while(Serial.available()){
    inCharBuffer = Serial.read();
    if(inCharBuffer == masterStartMarker){            //if character recived is masterStartMarker then set reciveCommand to true
      reciveCommand = true;
    }
    else if(inCharBuffer == masterEndMarker){         // if character is the mastEndMarker then set recive command to faluse and returnData to true
      reciveCommand = false;
      returnData = true;
      delay(10);
    }  
    else{                                             //else for any other character recived on the xbee serial port do the following
      if(reciveCommand){                              //check if reciveCommand is true then 
        xbeeInBuffer = inCharBuffer;                  //store the next byte in the xbeeInBuffer if so
      }
      //do nothing                                    //don't do anything if not reciving a command from the flight computer.  This will happen when other PODs are sending data over the xbee serial port
    }
  }

  if(returnData){                           //if returnData is true then the flight computer has finished sending a command and wants a response from one of the PODs
    if(xbeeInBuffer == PODID){              //if it wants the response from this POD send it the encoded data
      numDataBytes = storeData();           //store the sensor values to data
      encodeData(numDataBytes);             //encode data to be sent as binary data
      sendData(numDataBytes);               //send the encoded data
    }
    xbeeInBuffer = 0;                       //clear the xbeeInBuffer
    returnData = false;                     //set returnData back to false
  }
  //=================================================================================

  //You should probably have all of you stuff that needs to be in loop() before the XBEE FC section...

}//ens of loop()

//Add any custom functions below here
//The interrupt service routine simply increments the pulse counter.
//It must be short and must not call delay() or Serial functions.
void geigerISR(){ count++; }

//===========================================================================
//=========================BINARY DATA FUNCTIONS=============================
//========================DO NOT MAKE ANY CHANGES============================
//===========================================================================

//this function stores the sensor variable data into the byte array for transfering data 
int storeData(){
  data[0] = val_01.bytes[0];
  data[1] = val_01.bytes[1];
  data[2] = val_01.bytes[2];
  data[3] = val_01.bytes[3];

  data[4] = val_02.bytes[0];
  data[5] = val_02.bytes[1];
  data[6] = val_02.bytes[2];
  data[7] = val_02.bytes[3];

  data[8] = val_03.bytes[0];
  data[9] = val_03.bytes[1];
  data[10] = val_03.bytes[2];
  data[11] = val_03.bytes[3];

  data[12] = val_04.bytes[0];
  data[13] = val_04.bytes[1];
  data[14] = val_04.bytes[2];
  data[15] = val_04.bytes[3];

  data[16] = val_05.bytes[0];
  data[17] = val_05.bytes[1];
  data[18] = val_05.bytes[2];
  data[19] = val_05.bytes[3];

  data[20] = val_06.bytes[0];
  data[21] = val_06.bytes[1];
  data[22] = val_06.bytes[2];
  data[23] = val_06.bytes[3];

  data[24] = val_07.bytes[0];
  data[25] = val_07.bytes[1];
  data[26] = val_07.bytes[2];
  data[27] = val_07.bytes[3];

  data[28] = val_08.bytes[0];
  data[29] = val_08.bytes[1];
  data[30] = val_08.bytes[2];
  data[31] = val_08.bytes[3];

  data[32] = val_09.bytes[0];
  data[33] = val_09.bytes[1];
  data[34] = val_09.bytes[2];
  data[35] = val_09.bytes[3];

  data[36] = val_10.bytes[0];
  data[37] = val_10.bytes[1];
  data[38] = val_10.bytes[2];
  data[39] = val_10.bytes[3];

  return 40;  //returns the number of bytes in data
}

void encodeData(byte numBytes){
  // Copies to encodeBuffer[] all of the data in data[]
  // and converts any bytes of specialByte (250) or more into a pair of bytes, 250 0, 250 1, 250 2, 250 3, 250 4, or 250 5 as appropriate
  dataTotalSend = 0;
  for (byte n = 0; n < numBytes; n++) {
    if (data[n] >= specialByte) {
      encodeBuffer[dataTotalSend] = specialByte;
      dataTotalSend++;
      encodeBuffer[dataTotalSend] = data[n] - specialByte;
    }
    else {
      encodeBuffer[dataTotalSend] = data[n];
    }
    dataTotalSend++;
  }
}

void sendData(byte sendCount) {
  //sendCount must be less than 250
  if(sendCount > 249){
    //indicate an error...
    Serial.println("ERROR: sendCount greater than 249");
    return;
  }
      
  // expects to find encoded data in encodeBuffer
  Serial.write(slaveStartMarker);
  Serial.write(sendCount);
  Serial.write(encodeBuffer, dataTotalSend);
  Serial.write(slaveEndMarker);
}

//===========================================================================
//=======================END BINARY DATA FUNCTIONS===========================
//===========================================================================

//You should probably put any custom functions you have above the Binary Data Functions...


