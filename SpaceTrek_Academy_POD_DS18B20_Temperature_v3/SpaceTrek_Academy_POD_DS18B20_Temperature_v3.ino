/*
 * Space Trek Academy
 * Near Space Investigation POD
 * EXAMPLE: DS18B20 1-Wire Digital Temperature Sensor
 *
 * Blank Starter program for using an Arduino Uno or MEGA 2560 with
 * an XBEE shield attached.  Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then
 * replace the shield to test with the Flight Computer (FC).
 *
 * ---------------------------------------------------------------------------
 * WHAT THIS POD MEASURES
 * ---------------------------------------------------------------------------
 * A Maxim/Dallas DS18B20 waterproof 1-Wire digital temperature probe. It
 * reports temperature directly as a digital value (no analog reading and no
 * calibration curve needed) over a single data wire. This example uses ONE
 * sensor and reads it by index 0 on the bus.
 *
 * ---------------------------------------------------------------------------
 * WIRING (DS18B20 -> Arduino Uno)
 * ---------------------------------------------------------------------------
 *   DS18B20 RED    (VDD)  -> 5V
 *   DS18B20 BLACK  (GND)  -> GND
 *   DS18B20 YELLOW (DATA) -> Digital pin 2  (ONE_WIRE_BUS)
 *
 *   REQUIRED PULL-UP: a 4.7k ohm resistor between the DATA line (pin 2)
 *   and 5V. The 1-Wire bus is open-drain, so without this pull-up the
 *   sensor cannot communicate and you will read -127. This is the single
 *   most common wiring mistake with the DS18B20.
 *
 *   (This is a 1-Wire device, NOT I2C -- the A4/A5 SDA/SCL pins are unused.)
 *
 * ---------------------------------------------------------------------------
 * REQUIRED ARDUINO LIBRARIES (install via the Library Manager)
 * ---------------------------------------------------------------------------
 *   - "OneWire"           by Paul Stoffregen
 *   - "DallasTemperature" by Miles Burton
 *
 * ---------------------------------------------------------------------------
 * DATA CHANNELS SENT TO THE FLIGHT COMPUTER
 * ---------------------------------------------------------------------------
 *   val_01 = temperature in degrees Celsius     (deg C)
 *   val_02 = temperature in degrees Fahrenheit  (deg F)  = C * 9/5 + 32
 *   val_03 .. val_10 = unused (left at 0.0)
 *
 * ---------------------------------------------------------------------------
 * TEACHING POINT: THE NON-BLOCKING CONVERSION PATTERN
 * ---------------------------------------------------------------------------
 * A DS18B20 does not answer instantly. At 12-bit resolution it needs up to
 * ~750 ms to convert temperature after you ask for it. The easy library
 * call sensors.requestTemperatures() normally BLOCKS for that whole time,
 * which would freeze loop() and make this POD miss the Flight Computer's
 * polling. The delay() command is a bad command -- and a hidden 750 ms
 * block is even worse.
 *
 * Instead we call sensors.setWaitForConversion(false) in setup(), then run
 * a tiny 2-state machine driven by the timer:
 *
 *   State 0 (REQUEST): call sensors.requestTemperatures() -- it now returns
 *            immediately -- record the millis() time, and advance to state 1.
 *   State 1 (READ):    wait until at least 800 ms have elapsed since the
 *            request (giving the sensor time to finish), then read
 *            sensors.getTempCByIndex(0), store the result, and go back to
 *            state 0 to start the next request.
 *
 * The main loop never blocks, so the XBEE / FC communication keeps running
 * smoothly. If a read comes back DEVICE_DISCONNECTED_C (-127) the sensor did
 * not answer (bad wiring / missing pull-up), so we KEEP the previous value
 * instead of publishing garbage.
 *
 * Author: Andrew Gafford
 * Date: Mar. 20th, 2025
 * 
 */

//Included Libraries
//---------------------------------------------------------------------------
// LIBRARY DOWNLOADS  (install on YOUR computer -- nothing is bundled with this sketch)
//   Arduino IDE: Tools > Manage Libraries..., search the quoted name, click Install
//   (or download the .ZIP from the link, then Sketch > Include Library > Add .ZIP Library)
//   "OneWire"            https://github.com/PaulStoffregen/OneWire
//   "DallasTemperature"  https://github.com/milesburton/Arduino-Temperature-Control-Library
//---------------------------------------------------------------------------
#include <Arduino.h>
//Include any additional libraries needed here
#include <OneWire.h>
#include <DallasTemperature.h>

//settings
#define PODID         1                 //Must be 1, 2 or 3.  This is the PODID that this experiment will resond to when the FC asks for pod data
#define maxMessage    96                //maximum number of bytes that can be in the message sent to the FC


//Add any custom #define settings your program needs here
#define TIMER_TIME    400               //The number of miliseconds between timer cycles.  Kept short so the
                                        //state machine advances promptly; a full temperature reading still
                                        //spans two-plus cycles because of the 800 ms conversion wait below.
#define ONE_WIRE_BUS  2                 //DS18B20 DATA pin (needs a 4.7k pull-up resistor to 5V)
#define CONVERSION_WAIT 800             //ms to wait after a request before reading (DS18B20 12-bit needs ~750ms)


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
OneWire oneWire(ONE_WIRE_BUS);              //1-Wire bus on the DATA pin
DallasTemperature sensors(&oneWire);        //DallasTemperature driver riding on that 1-Wire bus



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
//add your variables here                 //you don't have to make them global but it makes life easy sometimes...
byte tempState = 0;                        //non-blocking state machine: 0 = request a conversion, 1 = wait then read
uint32_t tempRequestTime = 0;             //millis() timestamp of when the conversion was requested

void setup(){
  Serial.begin(57600);                    //Turns on serial port that will talk to the FC via the XBEE
  
  //setup any pinMode() that are needed for your experiment
  //pinMode(PIN-NUMBER, MODE);
  //pinMode(PIN-NUMBER, MODE);
  //pinMode(PIN-NUMBER, MODE);

  //Setup or start any sensors or devices you have that need to be setup here
  sensors.begin();                        //start the DallasTemperature driver / scan the 1-Wire bus
  sensors.setWaitForConversion(false);    //CRITICAL: make requestTemperatures() non-blocking (do NOT leave this true)

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

  if((millis() - timer) >= TIMER_TIME){           //check if enough time has passed for the timer
    timer = millis();                             //reset the timer to current cpu millis()

    //Do any of the things needed to collect your data here.
    //Remember not to do any blocking things!
    //Some devices are slow and may take too long to respond and cause timing issues.
    //Ask for help if it is not working as expected.

    //Non-blocking DS18B20 read (see the header comment for the full explanation):
    if(tempState == 0){                            //STATE 0: ask the sensor to start a conversion
      sensors.requestTemperatures();               //returns immediately because setWaitForConversion(false)
      tempRequestTime = millis();                  //remember when we asked
      tempState = 1;                               //advance to the read state
    }
    else{                                          //STATE 1: read once the conversion has had enough time
      if((millis() - tempRequestTime) >= CONVERSION_WAIT){
        float c = sensors.getTempCByIndex(0);      //read the finished conversion (index 0 = first sensor on the bus)
        if(c != DEVICE_DISCONNECTED_C){            //-127 means the sensor did not answer -> keep the previous value
          val_01.value = c;                        //temperature in degrees Celsius
          val_02.value = c * 9.0 / 5.0 + 32.0;     //temperature in degrees Fahrenheit
        }
        tempState = 0;                             //go back to start the next conversion
      }
    }

    //val_01.value = your first data item as a floating point;      //cast it to a float if needed, but it must be a float
    //val_02.value = your next data item;
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
// void customFunction(){
  //
  //
//

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



