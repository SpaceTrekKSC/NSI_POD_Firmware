/*
 * Space Trek Academy
 * Near Space Investigation POD
 * Blank Starter program for using an Arduino Uno or MEGA 2560 with
 * an XBEE shield attached.  Most XBEE shields block the USB serial
 * port. You need to remove the shield to program the board and then 
 * replace the shield to test with the Flight Computer (FC).
 * 
 * Author: Andrew Gafford
 * Date: Mar. 20th, 2025
 * 
 */

/*
 * ===========================================================================
 * EXAMPLE: ANALOG SENSORS (no external library required)
 * ===========================================================================
 * This example shows how to use analogRead() and how to convert a raw
 * 0-1023 ADC reading into meaningful units. It reads two simple analog
 * sensors: a photoresistor (LDR) for relative light level, and a TMP36
 * analog temperature sensor for temperature in degrees C.
 *
 * Assumes a standard 5V Arduino Uno: the ADC reference is 5.0V and a full
 * scale reading is 1023 counts, so:
 *     voltage = (rawReading / 1023.0) * 5.0
 *
 * SENSOR 1 - Photoresistor / LDR (light level) on analog pin A0
 *   Wire the LDR as the top half of a voltage divider:
 *
 *        5V ----[ LDR ]----+----[ 10k fixed R ]---- GND
 *                          |
 *                          +----> A0
 *
 *   The LDR goes from 5V to A0; the fixed ~10k resistor goes from A0 to GND.
 *   In bright light the LDR resistance drops and the A0 voltage rises; in
 *   darkness the A0 voltage falls. No pull-up is needed - the fixed resistor
 *   forms the divider. val_01 reports the raw counts and val_02 the divider
 *   voltage as a relative light level.
 *
 * SENSOR 2 - TMP36 analog temperature sensor on analog pin A1
 *   Hold the TMP36 with its flat/labeled face toward you and the three pins
 *   pointing down:
 *
 *        TMP36 (flat face toward you)
 *          |   |   |
 *         5V  A1  GND
 *        (left)(mid)(right)
 *
 *     Left pin   -> 5V
 *     Middle pin -> A1  (analog output)
 *     Right pin  -> GND
 *
 *   The TMP36 outputs 10 mV per degree C with a 500 mV (0.5V) offset at 0 C,
 *   so:  tempC = (voltageA1 - 0.5) * 100.0
 *
 * REQUIRED ARDUINO LIBRARIES: none (uses only the built-in analogRead()).
 * On an Uno the analog pins are A0..A5; I2C SDA/SCL live on A4/A5 but are
 * NOT used here, so all of A0..A5 are free for analog sensors.
 *
 * CHANNELS SENT TO THE FLIGHT COMPUTER:
 *   val_01 = LDR raw ADC reading (0-1023 counts, as a float)
 *   val_02 = LDR divider voltage (Volts)
 *   val_03 = TMP36 temperature (degrees C)
 *   val_04..val_10 = unused (0.0)
 * ===========================================================================
 */

//Included Libraries
#include <Arduino.h>
//Include any additional libraries needed here
//This example uses only the built-in analogRead(), so no extra libraries are needed.

//settings
#define PODID         1                 //Must be 1, 2 or 3.  This is the PODID that this experiment will resond to when the FC asks for pod data
#define maxMessage    96                //maximum number of bytes that can be in the message sent to the FC


//Add any custom #define settings your program needs here
#define TIMER_TIME    1000              //The number of miliseconds to wait before running the timer if statment again

#define LDR_PIN       A0                //Photoresistor voltage-divider output (LDR from 5V to A0, 10k from A0 to GND)
#define TMP36_PIN     A1                //TMP36 middle pin (left=5V, mid=A1, right=GND)
#define ADC_REF       5.0               //ADC reference voltage on a 5V Uno
#define ADC_COUNTS    1023.0            //Full-scale ADC counts (10-bit ADC)


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
//The analog sensors are read directly with analogRead(), so no sensor objects are needed.




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
  pinMode(LDR_PIN, INPUT);                //analog input for the LDR voltage divider
  pinMode(TMP36_PIN, INPUT);              //analog input for the TMP36 temperature sensor

  //Setup or start any sensors or devices you have that need to be setup here
  //Analog sensors need no initialization - analogRead() works right away.


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


    //analogRead() is fast (~100us), so it is safe to read directly here.

    //--- Photoresistor / LDR on A0 ---
    int ldrRaw = analogRead(LDR_PIN);                       //raw 0-1023 counts
    float ldrVoltage = (ldrRaw / ADC_COUNTS) * ADC_REF;     //convert counts to volts

    //--- TMP36 temperature sensor on A1 ---
    int tmp36Raw = analogRead(TMP36_PIN);                   //raw 0-1023 counts
    float tmp36Voltage = (tmp36Raw / ADC_COUNTS) * ADC_REF; //convert counts to volts
    float tempC = (tmp36Voltage - 0.5) * 100.0;             //TMP36: 10mV/C with 0.5V offset at 0C

    //val_01.value = your first data item as a floating point;      //cast it to a float if needed, but it must be a float
    //val_02.value = your next data item;
    val_01.value = (float)ldrRaw;                           //LDR raw counts (0-1023)
    val_02.value = ldrVoltage;                              //LDR divider voltage (V)
    val_03.value = tempC;                                   //TMP36 temperature (deg C)
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



