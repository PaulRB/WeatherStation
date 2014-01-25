/*
  SHT2x - A Humidity Library for Arduino.

  Supported Sensor modules:
    SHT21-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor
	SHT2x-Breakout Module - http://www.misenso.com/products/001
	
  Created by Christopher Ladden at Modern Device on December 2009.
  
  Modified by www.misenso.com on October 2011:
	- code optimisation
	- compatibility with Arduino 1.0

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef SHT2X_H
#define SHT2X_H

#include <inttypes.h>

typedef enum {
    eSHT2xAddress = 0x40,
} HUM_SENSOR_T;

typedef enum {
    eTempHoldCmd		= 0xE3,
    eRHumidityHoldCmd	= 0xE5,
    eTempNoHoldCmd      = 0xF3,
    eRHumidityNoHoldCmd = 0xF5,
} HUM_MEASUREMENT_CMD_T;

class SHT2xClass
{
  private:
	uint16_t readSensor(uint8_t command);

  public:
    float GetHumidity(void);
    float GetTemperature(void);
};

extern SHT2xClass SHT2x;

#endif

/*
  SHT2x - A Humidity Library for Arduino.

  Supported Sensor modules:
    SHT21-Breakout Module - http://www.moderndevice.com/products/sht21-humidity-sensor
	SHT2x-Breakout Module - http://www.misenso.com/products/001
	
  Created by Christopher Ladden at Modern Device on December 2009.
  Modified by Paul Badger March 2010
  
  Modified by www.misenso.com on October 2011:
	- code optimisation
	- compatibility with Arduino 1.0

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
//#include <Wire.h>
//#include "Arduino.h"
//#include "SHT2x.h"



/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return float - The relative humidity in %RH
 **********************************************************/
float SHT2xClass::GetHumidity(void)
{
	return (-6.0 + 125.0 / 65536.0 * (float)(readSensor(eRHumidityHoldCmd)));
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float SHT2xClass::GetTemperature(void)
{
	return (-46.85 + 175.72 / 65536.0 * (float)(readSensor(eTempHoldCmd)));
}


/******************************************************************************
 * Private Functions
 ******************************************************************************/

uint16_t SHT2xClass::readSensor(uint8_t command)
{
    uint16_t result;

    Wire.beginTransmission(eSHT2xAddress);	//begin
    Wire.write(command);					//send the pointer location
    delay(100);
    Wire.endTransmission();               	//end

    Wire.requestFrom(eSHT2xAddress, 3);
    while(Wire.available() < 3) {
      ; //wait
    }

    //Store the result
    result = ((Wire.read()) << 8);
    result += Wire.read();
	result &= ~0x0003;   // clear two low bits (status bits)
    return result;
}

SHT2xClass SHT2x;

/****************************************************************************
* BMP085.h - BMP085/I2C (Digital Pressure Sensor) library for Arduino       *
* Copyright 2010-2012 Filipe Vieira & various contributors                  *
*                                                                           *
* This file is part of BMP085 Arduino library.                              *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with BMP085 Breakout                               *
* SDA   -> pin 20   (no pull up resistors)                                  *
* SCL   -> pin 21   (no pull up resistors)                                  *
* XCLR  -> not connected                                                    *
* EOC   -> not connected                                                    *
* GND   -> pin GND                                                          *
* VCC   -> pin 3.3V                                                         *
* NOTE: SCL and SDA needs pull-up resistors for each I2C bus.               *
*  2.2kOhm..10kOhm, typ. 4.7kOhm                                            *
*****************************************************************************/
#ifndef BMP085_h
#define BMP085_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//#include "WProgram.h"
#endif

#define BMP085_ADDR                 0x77     //0x77 default I2C address
#define BUFFER_SIZE                 3

#define AUTO_UPDATE_TEMPERATURE     true    //default is true
        // when true, temperature is measured everytime pressure is measured (Auto).
        // when false, user chooses when to measure temperature (just call calcTrueTemperature()).
        // used for dynamic measurement to increase sample rate (see BMP085 modes below).
       
/* ---- Registers ---- */
#define CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define CAL_B1            0xB6  // R   Calibration data (16 bits)
#define CAL_B2            0xB8  // R   Calibration data (16 bits)
#define CAL_MB            0xBA  // R   Calibration data (16 bits)
#define CAL_MC            0xBC  // R   Calibration data (16 bits)
#define CAL_MD            0xBE  // R   Calibration data (16 bits)
#define CONTROL           0xF4  // W   Control register 
#define CONTROL_OUTPUT    0xF6  // R   Output registers 0xF6=MSB, 0xF7=LSB, 0xF8=XLSB

// unused registers
#define SOFTRESET         0xE0
#define VERSION           0xD1  // ML_VERSION  pos=0 len=4 msk=0F  AL_VERSION pos=4 len=4 msk=f0
#define CHIPID            0xD0  // pos=0 mask=FF len=8
                                // BMP085_CHIP_ID=0x55

/************************************/
/*    REGISTERS PARAMETERS          */
/************************************/
// BMP085 Modes
#define MODE_ULTRA_LOW_POWER    0 //oversampling=0, internalsamples=1, maxconvtimepressure=4.5ms, avgcurrent=3uA, RMSnoise_hPA=0.06, RMSnoise_m=0.5
#define MODE_STANDARD           1 //oversampling=1, internalsamples=2, maxconvtimepressure=7.5ms, avgcurrent=5uA, RMSnoise_hPA=0.05, RMSnoise_m=0.4
#define MODE_HIGHRES            2 //oversampling=2, internalsamples=4, maxconvtimepressure=13.5ms, avgcurrent=7uA, RMSnoise_hPA=0.04, RMSnoise_m=0.3
#define MODE_ULTRA_HIGHRES      3 //oversampling=3, internalsamples=8, maxconvtimepressure=25.5ms, avgcurrent=12uA, RMSnoise_hPA=0.03, RMSnoise_m=0.25
                  // "Sampling rate can be increased to 128 samples per second (standard mode) for
                  // dynamic measurement.In this case it is sufficient to measure temperature only 
                  // once per second and to use this value for all pressure measurements during period."
                  // (from BMP085 datasheet Rev1.2 page 10).
                  // To use dynamic measurement set AUTO_UPDATE_TEMPERATURE to false and
                  // call calcTrueTemperature() from your code. 
// Control register
#define READ_TEMPERATURE        0x2E 
#define READ_PRESSURE           0x34 
//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)



class BMP085 {
public:  
  BMP085();
  
  // BMP initialization
  void init();                                              // sets current elevation above ground level to 0 meters
  void init(byte _BMPMode, int32_t _initVal, bool _centimeters);   // sets a reference datum
                                                            // if _centimeters=false _initVal is Pa
  // Who Am I
  byte getDevAddr();
  
  // BMP mode  
  byte getMode();        
  void setMode(byte _BMPMode);                   // BMP085 mode 
  // initialization
  void setLocalPressure(int32_t _Pa);            // set known barometric pressure as reference Ex. QNH
  void setLocalAbsAlt(int32_t _centimeters);     // set known altitude as reference
  void setAltOffset(int32_t _centimeters);       // altitude offset
  void sethPaOffset(int32_t _Pa);                // pressure offset
  void zeroCal(int32_t _Pa, int32_t _centimeters);// zero Calibrate output to a specific Pa/altitude 
  // BMP Sensors
  void getPressure(int32_t *_Pa);                // pressure in Pa + offset  
  void getAltitude(int32_t *_centimeters);       // altitude in centimeters + offset  
  void getTemperature(int32_t *_Temperature);    // temperature in Cº   
  void calcTrueTemperature();                    // calc temperature data b5 (only needed if AUTO_UPDATE_TEMPERATURE is false)  
  void calcTruePressure(int32_t *_TruePressure);    // calc Pressure in Pa     
  // dummy stuff
   void dumpCalData();                           // debug only

  void writemem(uint8_t _addr, uint8_t _val);
  void readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]);
  
  private:
  
  int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;               // cal data  
  uint16_t ac4,ac5,ac6;                     // cal data
  int32_t b5, oldEMA;                                      // temperature data
  
  uint8_t _dev_address;
  byte _buff[BUFFER_SIZE];                      // buffer  MSB LSB XLSB
  int16_t _oss;                                     // OverSamplingSetting
  int16_t _pressure_waittime[4];                    // Max. Conversion Time Pressure is ms for each mode
  
  int32_t _cm_Offset, _Pa_Offset;
  int32_t _param_datum, _param_centimeters;

  void getCalData();        
  

};

#endif

/****************************************************************************
* BMP085.cpp - BMP085/I2C (Digital Pressure Sensor) library for Arduino     *
* Copyright 2010-2012 Filipe Vieira & various contributors                  *
*                                                                           *
* This file is part of BMP085 Arduino library.                              *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/
/****************************************************************************
* Tested on Arduino Mega with BMP085 Breakout                               *
* SDA   -> pin 20   (no pull up resistors)                                  *
* SCL   -> pin 21   (no pull up resistors)                                  *
* XCLR  -> not connected                                                    *
* EOC   -> not connected                                                    *
* GND   -> pin GND                                                          *
* VCC   -> pin 3.3V                                                         *
* NOTE: SCL and SDA needs pull-up resistors for each I2C bus.               *
*  2.2kOhm..10kOhm, typ. 4.7kOhm                                            *
*****************************************************************************/
//#include <Wire.h>
//#include <BMP085.h>
#include <cmath>

BMP085::BMP085() {
  _dev_address = BMP085_ADDR;
  _pressure_waittime[0] = 5; // These are maximum convertion times.
  _pressure_waittime[1] = 8; // It is possible to use pin EOC (End Of Conversion)
  _pressure_waittime[2] = 14;// to check if conversion is finished (logic 1) 
  _pressure_waittime[3] = 26;// or running (logic 0) insted of waiting for convertion times.
  _cm_Offset = 0;
  _Pa_Offset = 0;               // 1hPa = 100Pa = 1mbar
  
  oldEMA = 0;
}

void BMP085::init() {  
  init(MODE_STANDARD, 0, true);
}

void BMP085::init(byte _BMPMode, int32_t _initVal, bool _Unitmeters){     
  getCalData();               // initialize cal data
  calcTrueTemperature();      // initialize b5
  setMode(_BMPMode);
  _Unitmeters ? setLocalAbsAlt(_initVal) : setLocalPressure(_initVal); 
}

byte BMP085::getDevAddr() {   
  return _dev_address;
}

byte BMP085::getMode(){
  return _oss;
}

void BMP085::setMode(byte _BMPMode){
  _oss = _BMPMode;
}

void BMP085::setLocalPressure(int32_t _Pa){   
  int32_t tmp_alt;
 
  _param_datum = _Pa;   
  getAltitude(&tmp_alt);    // calc altitude based on current pressure   
  _param_centimeters = tmp_alt;
}

void BMP085::setLocalAbsAlt(int32_t _centimeters){  
  int32_t tmp_Pa;
 
  _param_centimeters = _centimeters;   
  getPressure(&tmp_Pa);    // calc pressure based on current altitude
  _param_datum = tmp_Pa;
}

void BMP085::setAltOffset(int32_t _centimeters){
  _cm_Offset = _centimeters;
}

void BMP085::sethPaOffset(int32_t _Pa){
  _Pa_Offset = _Pa;
}

void BMP085::zeroCal(int32_t _Pa, int32_t _centimeters){
  setAltOffset(_centimeters - _param_centimeters);    
  sethPaOffset(_Pa - _param_datum);    
}

void BMP085::getPressure(int32_t *_Pa){   
  int32_t TruePressure;

  calcTruePressure(&TruePressure); 
  *_Pa = TruePressure / pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;
  // converting from float to int32_t truncates toward zero, 1010.999985 becomes 1010 resulting in 1 Pa error (max).  
  // Note that BMP085 abs accuracy from 700...1100hPa and 0..+65ºC is +-100Pa (typ.)
}

void BMP085::getAltitude(int32_t *_centimeters){
  int32_t TruePressure;

  calcTruePressure(&TruePressure); 
  *_centimeters =  4433000 * (1 - pow((TruePressure / (float)_param_datum), 0.1903)) + _cm_Offset;  
  // converting from float to int32_t truncates toward zero, 100.999985 becomes 100 resulting in 1 cm error (max).
}

void BMP085::getTemperature(int32_t *_Temperature) {
  calcTrueTemperature();                            // force b5 update
  *_Temperature = ((b5 + 8) >> 4);
}

void BMP085::calcTrueTemperature(){
  int32_t ut,x1,x2;

  //read Raw Temperature
  writemem(CONTROL, READ_TEMPERATURE);
  delay(5);                                         // min. 4.5ms read Temp delay
  readmem(CONTROL_OUTPUT, 2, _buff); 
  ut = ((int32_t)_buff[0] << 8 | ((int32_t)_buff[1]));    // uncompensated temperature value
  
  // calculate temperature
  x1 = ((int32_t)ut - ac6) * ac5 >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + md);
  b5 = x1 + x2;
}

void BMP085::calcTruePressure(int32_t *_TruePressure) {
  int32_t up,x1,x2,x3,b3,b6,p;
  uint32_t b4,b7;
  int32_t tmp; 

  #if AUTO_UPDATE_TEMPERATURE
  calcTrueTemperature();        // b5 update 
  #endif 
 
 //read Raw Pressure
  writemem(CONTROL, READ_PRESSURE+(_oss << 6));
  delay(_pressure_waittime[_oss]);    
  readmem(CONTROL_OUTPUT, 3, _buff);  
  up = ((((int32_t)_buff[0] <<16) | ((int32_t)_buff[1] <<8) | ((int32_t)_buff[2])) >> (8-_oss)); // uncompensated pressure value
  
  // calculate true pressure
  b6 = b5 - 4000;             // b5 is updated by calcTrueTemperature().
  x1 = (b2* (b6 * b6 >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = ac1;
  tmp = (tmp * 4 + x3) << _oss;
  b3 = (tmp + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> _oss);
  p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *_TruePressure = p + ((x1 + x2 + 3791) >> 4);
}

void BMP085::dumpCalData() {
  Serial.println("---cal data start---");
  Serial.print("ac1:");
  Serial.println(ac1,DEC);
  Serial.print("ac2:");
  Serial.println(ac2,DEC);
  Serial.print("ac3:");
  Serial.println(ac3,DEC);
  Serial.print("ac4:");
  Serial.println(ac4,DEC);
  Serial.print("ac5:");
  Serial.println(ac5,DEC);
  Serial.print("ac6:");
  Serial.println(ac6,DEC); 
  Serial.print("b1:");
  Serial.println(b1,DEC);
  Serial.print("b2:");
  Serial.println(b2,DEC); 
  Serial.print("mb:");
  Serial.println(mb,DEC);
  Serial.print("mc:");
  Serial.println(mc,DEC);
  Serial.print("md:");
  Serial.println(md,DEC);
  Serial.println("---cal data end---");
}

//PRIVATE methods

void BMP085::getCalData() {
  readmem(CAL_AC1, 2, _buff);
  ac1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  readmem(CAL_AC2, 2, _buff);
  ac2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  readmem(CAL_AC3, 2, _buff);
  ac3 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  readmem(CAL_AC4, 2, _buff);
  ac4 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
  readmem(CAL_AC5, 2, _buff);
  ac5 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
  readmem(CAL_AC6, 2, _buff);
  ac6 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1])); 
  readmem(CAL_B1, 2, _buff);
  b1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
  readmem(CAL_B2, 2, _buff);
  b2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
  readmem(CAL_MB, 2, _buff);
  mb = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  readmem(CAL_MC, 2, _buff);
  mc = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  readmem(CAL_MD, 2, _buff);
  md = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
}


void BMP085::writemem(uint8_t _addr, uint8_t _val) {
  Wire.beginTransmission(_dev_address);   // start transmission to device 
  Wire.write(_addr); // send register address
  Wire.write(_val); // send value to write  
  Wire.endTransmission(); // end transmission
}

void BMP085::readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
  Wire.beginTransmission(_dev_address); // start transmission to device 
  Wire.write(_addr); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(_dev_address); // start transmission to device 
  Wire.requestFrom(_dev_address, _nbytes);// send data n-bytes read
  uint8_t i = 0; 
  while (Wire.available()) {
    __buff[i] = Wire.read(); // receive DATA
    i++;
  }
  Wire.endTransmission(); // end transmission
}

// Weather Station
// PaulRB
// Jan 2014

// Fine Offset Wind sensor: 1 rev/sec = 1.49 mph
// Fine Offset Rain guage: 1 tip = 0.3 mm

#define LDR A0
#define windDirPin A1
#define windSpeedPin D2
#define rainGuagePin D3
#define LED D7

BMP085 dps = BMP085();

char BMP085Pressure[10], BMP085Temperature[10], SHT21humidity[10], SHT21Temperature[10], lightLevel[10], windSpd[10], windDir[10], rainFall[10];

//Windspeed resistance values:
//N: 3.9K
//NNE: 3.1K
//NE: 16K
//ENE: 14.1K
//E: 120K
//ESE: 42.2K
//SE: 65.2K
//SSE: 21.9K
//S: 33K
//SSW: 6.6K
//SW: 8.2K
//WSW: 0.9K
//W: 1K
//WNW: 0.7K
//NW: 2.2K
//NNW: 1.4K

const int dirValues[17] = {144, 168, 204, 280, 368, 436, 528, 620,
                           708, 780, 816, 868, 908, 936, 968, 1000, 1023};
const char dirCompass[17][4] = {"WNW", "WSW", "-W-", "NNW", "N-W", "NNE", "-N-", "SSW",
                                "S-W", "ENE", "N-E", "SSE", "-S-", "ESE", "S-E", "-E-", "!?!"};

volatile unsigned long rainEvents = 0;
volatile unsigned long windEvents = 0;
volatile unsigned long nextRainEvent = 0;
volatile unsigned long nextWindEvent = 0;

void rainGuageEvent() {
    unsigned long m = millis();
    if (m > nextRainEvent) {
        //Serial.println("Rain!");
        rainEvents++;
        nextRainEvent = m + 10;
    }
}

void windSpeedEvent() {
    unsigned long m = millis();
    if (m > nextWindEvent) {
        //Serial.println("Wind!");
        windEvents++;
        nextWindEvent = m + 10;
    }
}

unsigned long windMeasureStart;

void setup() {
    
    Serial.begin(38400);
    Wire.begin();
    dps.init(MODE_STANDARD, 28300, true);
    
    pinMode (LED, OUTPUT);
    
    pinMode (LDR, INPUT);
    
    pinMode(rainGuagePin, INPUT_PULLUP);
    attachInterrupt(rainGuagePin, rainGuageEvent, CHANGE);
    pinMode(windSpeedPin, INPUT_PULLUP);
    attachInterrupt(windSpeedPin, windSpeedEvent, CHANGE);
    pinMode(windDirPin, INPUT);
    windMeasureStart = millis();
    windEvents = 0;

    Spark.variable("temp1", &BMP085Temperature, STRING);
    Spark.variable("press", &BMP085Pressure, STRING);
    Spark.variable("temp2", &SHT21Temperature, STRING);
    Spark.variable("humid", &SHT21humidity, STRING);
    Spark.variable("light", &lightLevel, STRING);
    Spark.variable("wspeed", &windSpd, STRING);
    Spark.variable("wdir", &windDir, STRING);
    Spark.variable("rain", &rainFall, STRING);
    

}

void loop() {
    
    digitalWrite(LED, HIGH);

    sprintf(SHT21humidity, "%.1f%%", SHT2x.GetHumidity());
    sprintf(SHT21Temperature, "%.1fC", SHT2x.GetTemperature());
    int32_t x;
    dps.getPressure(&x);
    sprintf(BMP085Pressure, "%.1fHPa", x / 100.0);
    dps.getTemperature(&x);
    sprintf(BMP085Temperature, "%.1fC", x / 10.0);
    sprintf(lightLevel, "%0.1f%%", analogRead(LDR) / 40.96);
    x = analogRead(windDirPin);
    Serial.println(x);
    x = x >>2;
    int i = 0;
    while (x > dirValues[i]) i++;
    sprintf(windDir, "%s", dirCompass[i]);
    unsigned long now = millis();
    if (now >= windMeasureStart + 5000) {
        sprintf(windSpd, "%0.1fmph", 1490.0 * windEvents / (4.0 * (now - windMeasureStart)) );
        windMeasureStart = now;
        windEvents = 0;
    }

    //Serial.print (" % Rain "); Serial.print (3 * 1000 / rainRate / 10);
    //Serial.print (" mm/hr Wind "); Serial.print (1490 / windSpeed);
    sprintf(rainFall, "%ld", rainEvents);
    //Serial.println(SHT21humidity);
    //Serial.println(SHT21Temperature);
    //Serial.println(BMP085Pressure);
    //Serial.println(BMP085Temperature);
    //Serial.println(lightLevel);
    //Serial.println(windSpeed);
    //Serial.println(windDir);
    //Serial.println(rainFall);
    
    digitalWrite(LED, LOW);
}

