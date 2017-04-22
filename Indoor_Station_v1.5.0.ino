/****************************************************************************************************************************
*
* LoRa IOT Home Environment Monitoring System
* 
* Indoor Station Software
* by Rod Gatehouse
*
* Distributed under the terms of the MIT License:
* http://www.opensource.org/licenses/mit-license
*
* VERSION 1.5.0
* April 8, 2017
*
*****************************************************************************************************************************/

#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <OneWire.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>

#include <avr/sleep.h>                                  // libraries need to enable sleep routines for Feather 32u4
#include <avr/power.h>
#include <avr/wdt.h>

#define LED       13                                    // pin definition for LED on Feather 32u4 RFM95 LoRa Radio board

#define RFM95_CS  8                                     // LoRa Radio pin connections on Feather 32u4 RFM95 LoRa Radio board
#define RFM95_RST 4
#define RFM95_INT 7

#define VBATPIN   A9                                    // analog pin on Feather 32u4 RFM95 LoRa Radio used to monitor power supply (battery) voltage

#define RF95_FREQ 915.0                                 // define center frequency for LoRa Radio
RH_RF95 rf95(RFM95_CS, RFM95_INT);                      // create LoRa Radio instance

Adafruit_SHT31 sht31 = Adafruit_SHT31();                // create SHT31 temperature & humidity sensor instance

typedef union
{
  float floatingPoint;                                  // this union allows floting point numbers to be sent as binary representations over the wireless link
  byte binary[sizeof(floatingPoint)];                   // "sizeof" function takes care of determining the number of bytes needed for the floating point number
} binaryFloat;

binaryFloat airTemp;                                    // ***************************************************************************
binaryFloat gndTemp;                                    // remote stations can send up to five floating point values as defined here
binaryFloat humidity;                                   // Outside stations will typically send all five values
binaryFloat pressure;                                   // Inside stations will typically send airTemp and humidity values
binaryFloat batVolt;                                    // ***************************************************************************


int stateTime;                                          // keeps State Time for MAIN EVENT LOOP

/*  each Remote Station requires a unique ID number to be correctly identified by the LoRa IOT Gateway. In the current software version, the IDs are hard-coded in the 
 *  Remote Station software as below. Remote Station software needs to be recompiled with a unique Remote Station ID and downloaded to each Remote Station. In future
 *  software versions, this will be handled automatically between the LoRa IOT Gateway and the Remote Stations, allowing the system to be plug and play.
 */
//const byte stationID = B00000001;                     // Basement
//const byte stationID = B00000010;                     // Second Floor
  const byte stationID = B00000011;                     // First Floor, in this case the software would be compiled for the first floor station

byte wirelessPacket[21];                                // defines the size of the packet to be sent by the Remote Stations: 5 floats x 4 bytes per float + 1 byte for Remote Station ID

int minute10 = 74;                                      // 32u4 watchdog timer is configured to wake up every ~8 seconds, so 74 x 8 seconds = 592 seconds = 9.9 minutes
                                                        // this defines the reporting period of the Remote Stations; it doesn't need to be exact, just approximately every ten minutes
                                                        // it can be changed for more or less frequent updates.


/****************************************************************************************************************************/
/******************** SETUP *************************************************************************************************/
/****************************************************************************************************************************/

void setup()
{
  pinMode(RFM95_RST, OUTPUT);                           // LoRa Radio reset pin initialization
  digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RFM95_RST, LOW);                         // reset LoRa radio
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();
  rf95.setFrequency(RF95_FREQ);                         // initialize radio to 915-MHz   
  rf95.setTxPower(23, false);                           // set TX power to max = 23dBm = 200mW
  
  sht31.begin(0x44);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                  // select watchdog timer mode
  MCUSR &= ~(1 << WDRF);                                // reset status flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);                   // enable configuration changes
  WDTCSR = (1<< WDP0) | (1 << WDP3);                    // set prescalar = 9 for ~8 seconds sleep intervals (maximum)
  WDTCSR |= (1 << WDIE);                                // enable interrupt mode
  sleep_enable();                                       // enable sleep mode ready for use

  stateTime = 0;                                        // initialize State Time
  
  readSensors();                                        // read sensors
  delay(1000);                                          // wait one second
  readSensors();                                        // then read again. The first read sometimes returns a garbage value, so read twice when starting up
  buildWirelessPacket();                                // build a wireless packet
  sendWirelessPacket();                                 // send wireless packet, useful to make sure Remote Station is working correctly when it is first powered up
}


/****************************************************************************************************************************/
/******************** MAIN PROGRAM LOOP *************************************************************************************/
/****************************************************************************************************************************/

/*  the MAIN PROGRAM LOOP is a time-based event loop that every ten minutes reads sensors, builds a wirless packet, 
 *  and sends the packet */
 
void loop()
{
  if(stateTime == minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }

  else if(stateTime == 2*minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }
  
  else if(stateTime == 3*minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }

  else if(stateTime == 4*minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }
  
  else if(stateTime == 5*minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
  }
  
  else if(stateTime == 6*minute10)
  {
    readSensors();
    buildWirelessPacket();
    sendWirelessPacket();
    stateTime = 0;
  }

  sleep_mode();                                         // go to sleep every time through MAIN PROGRAM LOOP
}


/****************************************************************************************************************************/
/******************** ROUTINES **********************************************************************************************/
/****************************************************************************************************************************/


ISR(WDT_vect) /* Watch Dog Timer Interrupt Service Routine (ISR) */
{
  stateTime++;                                          // ISR increments State Time and cause MAIN PROGRAM LOOP to be executed every ~8 seconds
}                                     


void readSensors(void)  /* reads all sensor values */
{                                                       // read sensor data into floating point values in binaryFloat union struct for float - binary byte mapping
  airTemp.floatingPoint = sht31.readTemperature();      // read temperature
  humidity.floatingPoint = sht31.readHumidity();        // read humidity
  gndTemp.floatingPoint = 0;                            // Not equipped for indoor sensors
  pressure.floatingPoint = 0;                           // Not equipped for indoor sensors
  batVolt.floatingPoint = battVoltage();                // read battery or power supply voltage
}


void buildWirelessPacket(void)  /* builds wireless packet to be transmitted by LoRa Radio */
{                                                       
  for(int n=0; n<4; n++)                                // read binary values from binaryFloat union struct into wireless packet
  {
    wirelessPacket[n] = airTemp.binary[n];
    wirelessPacket[n+4] = gndTemp.binary[n];
    wirelessPacket[n+8] = humidity.binary[n];
    wirelessPacket[n+12] = pressure.binary[n];
    wirelessPacket[n+16] = batVolt.binary[n];
  }
  wirelessPacket[20] = stationID;                       // add Remote Station ID to wireless packet
}


void sendWirelessPacket() /* wakes up LoRa Radio and transmits wireless packet */
{
  rf95.send(wirelessPacket, sizeof(wirelessPacket));  
  delay(10);
  rf95.waitPacketSent();
  rf95.sleep();                                         // put LoRa Radio into sleep mode
}


float battVoltage(void) /* reads and scales battery or power supply voltage */
{ 
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;                                    // divided by 2 on the Feather Board, so multiply back
  measuredvbat *= 3.3;                                  // multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;                                 // convert to voltage
  return(measuredvbat);
}

/******************** END OF PROGRAM ****************************************************************************************/

