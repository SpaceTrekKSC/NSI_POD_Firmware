/*
 * Space Trek Academy
 * Near Space Investigation POD
 * MPU6050 6-Axis MOTION example (I2C) for an Arduino Uno / MEGA 2560 with
 * an XBEE shield attached.  Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then
 * replace the shield to test with the Flight Computer (FC).
 *
 * ---------------------------------------------------------------------
 * SENSOR: InvenSense MPU6050 (GY-521 breakout)
 *   3-axis accelerometer + 3-axis gyroscope + on-chip temperature.
 *   One sensor fills MANY channels.
 *
 * WIRING (I2C):
 *   MPU6050 VCC  -> 3.3V or 5V (GY-521 boards have an onboard regulator)
 *   MPU6050 GND  -> GND
 *   MPU6050 SDA  -> A4  (Uno SDA)   / pin 20 (MEGA 2560 SDA)
 *   MPU6050 SCL  -> A5  (Uno SCL)   / pin 21 (MEGA 2560 SCL)
 *   MPU6050 AD0  -> GND (I2C address 0x68, the library default)
 *   The GY-521 module already has SDA/SCL pull-up resistors on board,
 *   so no external pull-ups are required.
 *
 * REQUIRED ARDUINO LIBRARIES (install via Library Manager):
 *   - "Adafruit MPU6050"
 *   - "Adafruit Unified Sensor"   (dependency, pulled in automatically)
 *   - "Adafruit BusIO"            (dependency, pulled in automatically)
 *   Wire.h is part of the Arduino core (already installed).
 *
 * DATA CHANNELS sent to the Flight Computer (all floats):
 *   val_01 = acceleration X (m/s^2)
 *   val_02 = acceleration Y (m/s^2)
 *   val_03 = acceleration Z (m/s^2)
 *   val_04 = gyro X          (rad/s)
 *   val_05 = gyro Y          (rad/s)
 *   val_06 = gyro Z          (rad/s)
 *   val_07 = chip temperature (deg C)
 *   val_08..val_10 = unused (0.0)
 * ---------------------------------------------------------------------
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
//   "Adafruit MPU6050"         https://github.com/adafruit/Adafruit_MPU6050
//   "Adafruit Unified Sensor"  https://github.com/adafruit/Adafruit_Sensor
//   "Adafruit BusIO"           https://github.com/adafruit/Adafruit_BusIO
//---------------------------------------------------------------------------
#include <Arduino.h>
//Include any additional libraries needed here
#include <Wire.h>                 //Arduino core I2C library
#include <Adafruit_MPU6050.h>     //MPU6050 driver
#include <Adafruit_Sensor.h>      //Adafruit Unified Sensor base class (sensors_event_t)

//settings
#define PODID         1                 //Must be 1, 2 or 3.  This is the PODID that this experiment will resond to when the FC asks for pod data
#define maxMessage    96                //maximum number of bytes that can be in the message sent to the FC


//Add any custom #define settings your program needs here
#define TIMER_TIME    100               //The number of miliseconds to wait before running the timer if statment again (10 Hz motion sampling)


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
Adafruit_MPU6050 mpu;                 //the MPU6050 motion sensor object
bool mpuReady = false;                //true once mpu.begin() succeeds; keeps unused channels safe if the sensor is missing




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
  if(!mpu.begin()){                       //begin() also starts I2C (Wire); returns false if the chip is not found
    Serial.println("MPU6050 not found - check wiring/address");  //note it but do NOT block; unused channels stay 0.0
  }
  else{
    mpuReady = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   //+/-8 g full scale (good for launch/landing shocks)
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);        //+/-500 deg/s full scale
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     //on-chip low-pass filter to smooth vibration
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

    if(mpuReady){                                 //only read if the sensor initialised successfully
      sensors_event_t a, g, temp;                 //event structs the Adafruit library fills in
      mpu.getEvent(&a, &g, &temp);                //single non-blocking I2C read of accel, gyro and temperature

      val_01.value = a.acceleration.x;            //accel X (m/s^2)
      val_02.value = a.acceleration.y;            //accel Y (m/s^2)
      val_03.value = a.acceleration.z;            //accel Z (m/s^2)

      val_04.value = g.gyro.x;                    //gyro X (rad/s)
      val_05.value = g.gyro.y;                    //gyro Y (rad/s)
      val_06.value = g.gyro.z;                    //gyro Z (rad/s)

      val_07.value = temp.temperature;            //chip temperature (deg C)
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



