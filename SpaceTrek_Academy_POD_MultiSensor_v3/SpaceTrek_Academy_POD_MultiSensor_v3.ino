/*
 * Space Trek Academy
 * Near Space Investigation POD
 * MULTI-SENSOR example -- three different sensor TYPES on one POD.
 *
 * This example shows how to mix buses and fill several val channels while
 * NEVER using a blocking delay() in loop().  It reads, on every timer tick:
 *   1) a SLOW 1-Wire sensor (DS18B20) using an async request/read state machine,
 *   2) a simple ANALOG pin (photoresistor / LDR), and
 *   3) an I2C sensor (BME280).
 * The DS18B20 needs ~750ms to finish a conversion.  Instead of blocking, we call
 * setWaitForConversion(false) and use a small state machine: request the reading on
 * one timer cycle, then read the finished value on a later cycle (>=800ms later).
 * Because the timer fires once per second, the read always happens well after the
 * conversion has completed, and loop() is free to service the Flight Computer.
 *
 * -------------------- SENSORS & WIRING --------------------
 * (1) DS18B20  1-Wire temperature probe
 *       - DATA  -> Arduino pin 2
 *       - a 4.7k ohm pull-up resistor between DATA (pin 2) and +5V is REQUIRED
 *       - VDD   -> +5V   (GND -> GND)
 *       - Libraries: "OneWire" + "DallasTemperature"
 *       - Reports: val_01 = temperature (deg C)
 *
 * (2) Photoresistor / LDR  (analog light sensor, no library)
 *       - Wire the LDR and a fixed resistor (e.g. 10k) as a divider:
 *         +5V -- LDR -- (node -> A0) -- 10k -- GND
 *       - Analog input A0 reads the divider node
 *       - Reports: val_02 = light level (raw ADC counts, 0-1023)
 *
 * (3) BME280  pressure / humidity / temperature over I2C
 *       - SDA -> A4,  SCL -> A5   (the Uno's I2C pins)
 *       - VIN -> +5V (module has a regulator),  GND -> GND
 *       - I2C address is 0x76 on most breakouts, 0x77 on some (both are tried)
 *       - Libraries: "Adafruit BME280 Library" + "Adafruit Unified Sensor"
 *       - Reports: val_03 = pressure (hPa), val_04 = humidity (%),
 *                  val_05 = temperature (deg C)
 *
 * Everything runs together without any blocking delay(), demonstrating how to
 * combine a slow 1-Wire device, a plain analog pin, and an I2C sensor on one POD.
 *
 * Blank Starter program for using an Arduino Uno or MEGA 2560 with
 * an XBEE shield attached.  Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then
 * replace the shield to test with the Flight Computer (FC).
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
//   "OneWire"                  https://github.com/PaulStoffregen/OneWire
//   "DallasTemperature"        https://github.com/milesburton/Arduino-Temperature-Control-Library
//   "Adafruit BME280 Library"  https://github.com/adafruit/Adafruit_BME280_Library
//   "Adafruit Unified Sensor"  https://github.com/adafruit/Adafruit_Sensor
//   "Adafruit BusIO"           https://github.com/adafruit/Adafruit_BusIO
//---------------------------------------------------------------------------
#include <Arduino.h>
//Include any additional libraries needed here
#include <OneWire.h>            //1-Wire bus driver (used by the DS18B20)
#include <DallasTemperature.h>  //high-level DS18B20 temperature driver
#include <Wire.h>               //I2C bus (used by the BME280)
#include <Adafruit_Sensor.h>    //Adafruit Unified Sensor base class
#include <Adafruit_BME280.h>    //BME280 pressure/humidity/temperature driver

//settings
#define PODID         1                 //Must be 1, 2 or 3.  This is the PODID that this experiment will resond to when the FC asks for pod data
#define maxMessage    96                //maximum number of bytes that can be in the message sent to the FC


//Add any custom #define settings your program needs here
#define TIMER_TIME    1000              //The number of miliseconds to wait before running the timer if statment again
#define ONE_WIRE_BUS  2                 //DS18B20 1-Wire data pin (needs a 4.7k pull-up to +5V)
#define LDR_PIN       A0                //photoresistor / LDR analog input pin
#define DS18B20_CONV_MS 800             //ms to wait after a request before reading the DS18B20 (>= its ~750ms conversion)


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
OneWire oneWire(ONE_WIRE_BUS);            //1-Wire bus on pin 2
DallasTemperature ds18b20(&oneWire);      //DS18B20 driver on that bus
Adafruit_BME280 bme;                      //BME280 over I2C

//State variables for the non-blocking DS18B20 read and BME280 status
bool     dsWaiting     = false;           //true while a DS18B20 conversion is in flight
uint32_t dsRequestTime = 0;               //millis() when the current conversion was started
bool     bmeOK         = false;           //true if the BME280 initialised successfully




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

void setup(){
  Serial.begin(57600);                    //Turns on serial port that will talk to the FC via the XBEE
  
  //setup any pinMode() that are needed for your experiment
  //pinMode(PIN-NUMBER, MODE);
  //pinMode(PIN-NUMBER, MODE);
  //pinMode(PIN-NUMBER, MODE);

  //Setup or start any sensors or devices you have that need to be setup here
  //Start the DS18B20 1-Wire temperature sensor in NON-BLOCKING mode
  ds18b20.begin();
  ds18b20.setResolution(12);              //12-bit resolution (~750ms conversion time)
  ds18b20.setWaitForConversion(false);    //requestTemperatures() returns immediately -> no blocking

  //Start the BME280.  Most breakouts answer at 0x76; some at 0x77.
  bmeOK = bme.begin(0x76);
  if(!bmeOK){
    bmeOK = bme.begin(0x77);              //fallback I2C address
  }
  if(!bmeOK){
    Serial.println(F("BME280 not found (check wiring/address)"));  //message only, do NOT block
  }


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


    //val_01.value = your first data item as a floating point;      //cast it to a float if needed, but it must be a float
    //val_02.value = your next data item;

    //---- (1) DS18B20 1-Wire temperature: non-blocking async state machine ----
    //If a conversion was requested last cycle and enough time has passed, read it.
    if(dsWaiting && (millis() - dsRequestTime) >= DS18B20_CONV_MS){
      float dsTempC = ds18b20.getTempCByIndex(0);
      if(dsTempC != DEVICE_DISCONNECTED_C){       //ignore the -127 "not connected" sentinel
        val_01.value = dsTempC;                   //val_01 = DS18B20 temperature (deg C)
      }
      dsWaiting = false;
    }
    //If we are not currently waiting on a conversion, kick off a new one.
    if(!dsWaiting){
      ds18b20.requestTemperatures();              //returns immediately (setWaitForConversion(false))
      dsRequestTime = millis();
      dsWaiting = true;
    }

    //---- (2) Photoresistor / LDR on A0: plain analog read, no library ----
    val_02.value = (float)analogRead(LDR_PIN);    //val_02 = light level (raw 0-1023)

    //---- (3) BME280 over I2C (SDA=A4, SCL=A5): read if it initialised ----
    if(bmeOK){
      val_03.value = bme.readPressure() / 100.0F; //val_03 = pressure (hPa)
      val_04.value = bme.readHumidity();          //val_04 = humidity (%)
      val_05.value = bme.readTemperature();       //val_05 = temperature (deg C)
    }
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



