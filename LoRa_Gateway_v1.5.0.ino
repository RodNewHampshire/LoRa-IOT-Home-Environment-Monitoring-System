
/****************************************************************************************************************************
*
* LoRa IOT Home Environment Monitoring System
* 
* LoRa IOT Gateway Software
* by Rod Gatehouse
*
* Distributed under the terms of the MIT License:
* http://www.opensource.org/licenses/mit-license
*
* VERSION 1.5.0
* April 8, 2017
*
*****************************************************************************************************************************/

#include "Adafruit_LiquidCrystal.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "AIO_Account.h"                                                        // contains Adafruit IO account information, not currently on SD Memory Card
#include "EEPROMex.h"
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <RH_RF95.h>
#include <SD.h>
#include <SPI.h>
#include <Temboo.h>
#include <TimeLib.h>
#include <util.h>
#include "Wire.h"

int currentVersion  = 10500;                                                    // current software version X.YY.ZZ, i.e., 1.05.00

/******************** for LCD ***********************************************************************************************/

Adafruit_LiquidCrystal lcd(0);                                                  // initialize LCD library instance
byte setLCDBackLight;                                                           // variable to keep track of LCD backlight state, 1 = ON, 0 = OFF

/******************** for Push Buttons **************************************************************************************/
                                                                                // digital I/O pin definitions for push button switches
#define UP_BUTTON    16                                                         // referred to as "UP"
#define DOWN_BUTTON  17                                                         // referred to as "DOWN"
#define ENTR_BUTTON  18                                                         // referred to as "ENTER"
#define FUNC_BUTTON  19                                                         // referred to as "FUNC"

#define DEBOUNCE    250                                                         // debounce delay, used for all four push buttons, equal to number of milliseconds, e.g., 250 = 250ms

byte funcPress;                                                                 // variable to keep track of FUNC press

/******************** for Ethernet ******************************************************************************************/

byte mac[] = {0x80, 0xA2, 0xDA, 0x0F, 0xFA, 0x8A};                              // Arduino Ethernet shields no longer have MACIDs assigned so include a dummy MAC ID for DHCP
#define W5500_SS     10                                                         // W5500 SPI slave select pin
EthernetClient client;                                                          // initialize the Ethernet client library instance for Temboo email and spreadsheet Choreos

/******************** for SD Card *******************************************************************************************/

#define SD_SS         4                                                         // SD Card SPI slave select pin
File myFile;                                                                    // initialize SD File library instance

/******************** for Time Keeping **************************************************************************************/

unsigned int localPort = 8888;                                                  // NTP port to listen for UDP packets; NTP = Network Time Protocol
char timeServer[] = "pool.ntp.org";                                             // NTP server pool
const int NTP_PACKET_SIZE = 48;                                                 // NTP time stamp
byte packetBuffer[ NTP_PACKET_SIZE];                                            // NTP packet buffer
unsigned long epoch;                                                            // seconds since January 1, 1990
EthernetUDP Udp;                                                                // initialize UDP library instance for NTP

const char* TIME_ZONES[] =                                                      // worldwide time zone offsets from UTC by major cities
{                                                                         
  "Baker Island -12",                                                            
  "Amer. Samoa  -11",                                                           
  "Honolulu     -10",                                                           
  "Anchorage    - 9",                                                           
  "Los Angeles  - 8",                                                           
  "Phoenix      - 7",                                                           
  "Chicago      - 6",                                                           
  "New York     - 5",                                                          
  "Halifax      - 4",                                                           
  "Buenos Aires - 3",                                                           
  "Sth Georgia  - 2",                                                           
  "Greenland    - 1",                                                           
  "London         0",                                                           
  "Paris        + 1",                                                           
  "Cairo        + 2",                                                           
  "Moscow       + 3",                                                           
  "Dubai        + 4",                                                           
  "Karachi      + 5",                                                           
  "Dhaka        + 6",                                                           
  "Bangkok      + 7",                                                           
  "Singapore    + 8",                                                           
  "Tokyo        + 9",                                                           
  "Sydney       +10",                                                           
  "Noumea       +11",                                                           
  "Auckland     +12"                                                            
};                                                           

long  timeZoneOffset;                                                           // time zone offset value relative to UTC, Set Time Zone function accessed via FUNC button, initialized from EEPROM
long  dayLightSavings;                                                          // 1 = day light savings, 0 = no day light savings, Set Daylight Saving function accessed via FUNC button, initialized from EEPROM
unsigned long localizedEpoch;                                                   // NTP epoch adjusted for timezone and daylight savings

long lastSecond;                                                                // used with millis() to determine passing of one second
byte sysSecond = 0;                                                             // counts seconds, rolls over when reaches 60 and increments sysMinute
byte sysMinute = 0;                                                             // counts minutes, rolls over when reaches 60 and increments sysHour
byte sysHour = 0;                                                               // counts hours, rolls over when reaches 24

/******************** for Adafruit.io ***************************************************************************************/

#define AIO_SERVER      "io.adafruit.com"                                       // refer to Adafruit tutorial https://learn.adafruit.com/mqtt-adafruit-io-and-you
#define AIO_SERVERPORT  1883
EthernetClient client_io;
Adafruit_MQTT_Client mqtt(&client_io, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish Air_Temp = Adafruit_MQTT_Publish(&mqtt,               AIO_USERNAME "/feeds/outside-air-temp");
Adafruit_MQTT_Publish Gnd_Temp = Adafruit_MQTT_Publish(&mqtt,               AIO_USERNAME "/feeds/outside-gnd-temp");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt,               AIO_USERNAME "/feeds/outside-humidity");
Adafruit_MQTT_Publish Basement_Temp = Adafruit_MQTT_Publish(&mqtt,          AIO_USERNAME "/feeds/basement-temp");
Adafruit_MQTT_Publish Basement_Humidity = Adafruit_MQTT_Publish(&mqtt,      AIO_USERNAME "/feeds/basement-humidity");
Adafruit_MQTT_Publish Second_Floor_Temp = Adafruit_MQTT_Publish(&mqtt,      AIO_USERNAME "/feeds/2nd-floor-temp");
Adafruit_MQTT_Publish Second_Floor_Humidity = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/2nd-floor-humidity");
Adafruit_MQTT_Publish First_Floor_Temp = Adafruit_MQTT_Publish(&mqtt,       AIO_USERNAME "/feeds/1st-floor-temp");
Adafruit_MQTT_Publish First_Floor_Humidity = Adafruit_MQTT_Publish(&mqtt,   AIO_USERNAME "/feeds/1st-floor-humidity");
Adafruit_MQTT_Publish Grow_Tent_Temp = Adafruit_MQTT_Publish(&mqtt,         AIO_USERNAME "/feeds/grow-tent-temp");

/******************** for LoRa Radio ****************************************************************************************/

#define RFM95_SS    8                                                           // SPI slave select pin (CS)
#define RFM95_RST   9                                                           // reset pin (RST)
#define RFM95_INT   2                                                           // interrupt (G0)
#define RF95_FREQ   915.0                                                       // center frequency, defaults to 915-MHz in this code
RH_RF95 rf95(RFM95_SS, RFM95_INT);                                              // create radio library instance

/******************** for Station Data **************************************************************************************/

#define MAX_STATIONS 10                                                         // defines maximum number of remote stations
byte activeStations;                                                            // number of stations provisioned, Stations Active function accessed via FUNC button, initialized from EEPROM, less than or equal to MAX_STATIONS

typedef union
{
  float floatingPoint;                                                          // this union allows floting point numbers to be sent as binary representations over the wireless link
  byte binary[sizeof(floatingPoint)];                                           // "sizeof" function takes care of determining the number of bytes needed for the floating point number
} binaryFloat;

binaryFloat airTemp;                                                            // ***************************************************************************
binaryFloat gndTemp;                                                            // remote stations can send up to five floating point values as defined here
binaryFloat humidity;                                                           // Outside stations will typically send all five values
binaryFloat pressure;                                                           // Inside stations will typically send airTemp and humidity values
binaryFloat batVolt;                                                            // ***************************************************************************

typedef struct
{ // station data structure
  float data[5];                                                                // float array containing the float values described above
  int reportMinute;                                                             // keep track of the minute in the hour a report is received, useful for debugging stations
  int lastContactTimer;                                                         // track time since last contact
  unsigned long lastContactEpoch;                                               // time of last contact
  byte alerts;                                                                  // tracks if alerts enabled / disabled for this station
  int humiThreshold;                                                            // station humidity alert threshold
  int stnRSSI;                                                                  // keep track of the signal strength (RSSI) for each remote; Display RSSI funcion accessed via FUNC button to display RSSI on LCD
  char stnName[10];                                                             // assign a name for each station; Set Station Name function accessed via FUNC button, initialized from EEPROM
} stationData;

stationData station[MAX_STATIONS];                                              // create station data array sized for MAX_STATIONS

enum                                                                            // enum for alerts OFF and ON
{                                                                              
  OFF,                                                                         
  ON                                                                           
};                                                                              

const char* alertMode[] = {                                                     // character array for displaying alert "ON" and "OFF" status on the LCD
  "OFF",                                                                        
  "ON"                                                                         
};                                                                              

enum                                                                            // enum to index the five floating point variables in each station data structure
{                                                                               
  AIR_TEMP,                                                                     
  GND_TEMP,                                                                                                                                         
  HUMI,                                                                        
  PRES,                                                                         
  BATT                                                                         
};

byte stationNum = 0;                                                            // ID number of the last station that reported
byte stationDisplayNum;                                                         // ID number of the station data currently displayed on the LCD
String lastStationDisplay;                                                      // current station data displayed on LCD line 2, stored here so it can be restored after being written over by other functions
byte stationDisplayMode;                                                        // stores LCD Station Display mode based on enum below, initialized in EEPROM routines

enum                                                                            // enum to index LCD display modes for station data
{                                                                               // START_DISPLAY_MODES and END_DISPLAY_MODES are used to define the start and end of mode list when cycling through modes
  START_DISPLAY_MODES,                                                          
  LAST_STATION_TEMP,                                                            
  LAST_STATION_HUMI,                                                            
  FIXED_STATION_TEMP,                                                           
  FIXED_STATION_HUMI,                                                           
  CYCLE_STATION_TEMP,                                                           
  CYCLE_STATION_HUMI,                                                           
  END_DISPLAY_MODES                                                             
};                              

const char* DISPLAY_MODE[] = {                                                  // strings for displaying LCD Station Display modes
  " ",                                                                          // dummy strings are included in first and last place to align strings with the above enum array for LCD display modes
  "Last Stn Temp",                                                              
  "Last Stn Humi",                                                              
  "Fixed Stn Temp",                                                             
  "Fixed Stn Humi",                                                             
  "Cycle Stn Temp",                                                             
  "Cycle Stn Humi",                                                             
  " "                                                                           
};                              

enum                                                                            // enum for indexing alerts
{                                                                              
  LOW_BATTERY,                                                                  
  LOST_CONTACT,                                                                
  HIGH_HUMIDITY                                                                 
};                              

#define CONTACT_TIMEOUT 6                                                       // defines the number of 10 minute periods before a "lost contact" alert is sent. 6 = 6x10 = 60 minutes
float BATT_LOW = 3.7;                                                           // Low battery alert threshold
float pressCal;                                                                 // pressure sensor offset value to correct for altitude, Set Pressure Calibration function accessed via FUNC button, initialized from EEPROM
byte fahrenheit;                                                                // flag to set either Fahrenheit or Centigrade temperature display, Set Degrees F or C function accessed via FUNC button, initialized from EEPROM
const char* temperatureUnits[] = {" C", " F"};                                  // character array for C or F string for temperature units display

/******************** for Gateway Authentication ****************************************************************************/

String gatewayData[12];                                                         // Temboo, Google gmail and Google Sheets authentification information is read from SD card during set up

enum
{
  TEMBOO_ACCOUNT,                                                               // enum to index authentication data stored on SD card
  TEMBOO_APP_KEY_NAME,                                                          
  TEMBOO_APP_KEY,                                                               
  SS_TITLE,                                                                     
  SS_REFRESH_TOKEN,                                                             
  SS_CLIENT_SECRET,                                                              
  SS_CLIENT_ID,                                                                 
  EM_SUBJECT,                                                                   
  EM_PASSWORD,                                                                  
  EM_USERNAME,                                                                  
  EM_TO_ADDRESS                                                                 
};

/******************** for EEPROM ********************************************************************************************/

typedef struct                                                                  // structure containing variables to be stored in EEPROM
{                                                                              
  int Version;                                                                  // software version, if EEPROM version doesn't match software loaded, EEPROM values are initialized to default values below; if software version matches, stored EEPROM values are used to initialize variables
  long TimeZoneOffset;                                                          // time zone offset, EEPROM initialized to US Eastern Time Zone (-5)
  long DayLightSavings;                                                         // day light savings, EEPROM initialized to 1 for day light savings
  float PressCal;                                                               // pressure calibration factor, EEPROM initialized to 0.36
  char Station[MAX_STATIONS][10];                                               // array for station names, EEPROM initialized to "station"
  int StationHumiThreshold[MAX_STATIONS];                                       // station humidity alert threshold, EEPROM initialized to 50%
  byte StationAlert[MAX_STATIONS];                                              // station alert ON / OFF, EEPROM initialized OFF
  byte setBackLight;                                                            // back light on/off setting, EEPROM initialized to ON
  byte Fahrenheit;                                                              // Fahrenheit/Centigrade setting, EEPROM initialized to Fahrenheit
  byte StationDisplayMode;                                                      // sets LCD Station Display mode, EEPROM initialized to mode 1, display last station reporting temperature
  byte StationDisplayNum;                                                       // station number for LCD Station Display modes, EEPROM initialized to 0
  byte ActiveStations;                                                          // number of active (provisioned) stations, EEPROM initialized to 1
} eepromData;                                                                   

eepromData savedData;

int eepromAddress = 0;                                                          // EEPROM start address


/******************** for LCD / Push Button Functions ***********************************************************************/

enum                                                                            // enum to menu functions
{                                                                               // menu items are displayed in the order of this enum, the displayed order can be changed by rearranging this enum
  START_FUNCTIONS,                                                              // START_FUNCTIONS and END_FUNCTIONS are used to define the start and end of the function list when cycling through the functions
  SET_BACK_LIGHT,                                                               
  SET_DISPLAY_MODE,                                                             
  DISPLAY_RSSI,                                                                 
  DISPLAY_BATT, 
  SET_ALERTS,                                                                                                                 
  SET_HUMI_THRES,   
  SET_NUMBER_STATIONS,                                                              
  SET_STATION_NAMES,                                                             
  SET_DAYLIGHT_SAVINGS,                                                         
  SET_TIME_ZONE,                                                                
  SET_PRESS_SENSOR_CAL,                                                         
  SET_DEGREES_F_C,                                                              
  END_FUNCTIONS                                                                 
};        


/****************************************************************************************************************************/
/******************** SETUP *************************************************************************************************/
/****************************************************************************************************************************/

void setup()
{
  pinMode(53, OUTPUT);                                                          // althoguh pin 53 SPI Slave Select pin is not used, reportedly this is necessary to ensure the MEGA SPI stays in Master mode
  digitalWrite(53, HIGH);

                                                                                // put slave select line for all SPI peripherals into high "deselected" state to avoid possible SPI bus conflicts
  pinMode(SD_SS, OUTPUT);                                                       // SD Card
  digitalWrite(SD_SS, HIGH);                                                    
                                                                                
  pinMode(W5500_SS, OUTPUT);                                                    // W5500 Ethernet chip
  digitalWrite(W5500_SS, HIGH);                                                 
                                                                                
  pinMode(RFM95_SS, OUTPUT);                                                    // LoRa Radio
  digitalWrite(RFM95_SS, HIGH);                                                 

  pinMode(RFM95_RST, OUTPUT);                                                   // LoRa Radio reset pin initialization
  digitalWrite(RFM95_RST, HIGH);

  pinMode(UP_BUTTON, INPUT_PULLUP);                                             // initialize pins for push button switches
  pinMode(DOWN_BUTTON, INPUT_PULLUP);                                           
  pinMode(FUNC_BUTTON, INPUT_PULLUP);                                           
  pinMode(ENTR_BUTTON, INPUT_PULLUP);                                        

  SD.begin(SD_SS);                                                              // initialize SD Card driver with designated SPI slave select pin

  lcd.begin(16, 2);                                                             // initialize LCD driver for 16 x 2 display
  updateDisplayLine12("LoRa Gateway", String(currentVersion));                  // show spalsh screen with software version for 2 seconds during startup
  delay(2000);

  eepromRead();                                                                 // initialize variables from EEPROM
  RFM95Initialize();                                                            // initialize LoRa Radio
  ethernetInitialize();                                                         // initialize Ethernet connection
  readGatewayData();                                                            // read authentication data from SD card
  sendStatus();                                                                 // send an email to make sure Temboo email Choreo is running properly
  spreadsheetUpdate();                                                          // send a spreadsheet update to make sure Temboo spreadsheet Choreo is running properly
  AIOconnect();                                                                 // bring up Adafruit IO MQTT broker connection

  for (int n = 0; n < MAX_STATIONS; n++)                                        // initialize station variables that trigger alerts to prevent false alerts before initial station data is received
  {
    station[n].lastContactTimer = 0;                                            // set the Last Contact Timers to zero
    station[n].data[BATT] = 4.0;                                                // initialize station battery voltage tp 3.7V
    station[n].data[HUMI] = 0.0;                                                // initialize station humidity to 0%
  }

  attachInterrupt(digitalPinToInterrupt(FUNC_BUTTON), func, FALLING);           // initialize interrupt for FUNC button
  funcPress = 0;                                                                // reset funcPress before entering main loop
  ntpTimeUpdate();                                                              // update system time to NTP time immediately before entering the main loop
}


/****************************************************************************************************************************/
/******************** MAIN PROGRAM LOOP *************************************************************************************/
/****************************************************************************************************************************/

void loop()                                                                     // MAIN PROGRAM LOOP
{
  if (funcPress == 1)                                                           // check if FUNC pressed; FUNC Interrupt Service Routine (ISR) sets funcPress when FUNC is pressed
  {
    detachInterrupt(digitalPinToInterrupt(FUNC_BUTTON));                        // if FUNC pressed, disable further interrupts while this interrupt is serviced
    if (setLCDBackLight == 0)                                                   // check if LCD backlight is turned off
    {
      lcd.setBacklight(1);                                                      // if backlight is turned off, turn it on and set backlight flag
      setLCDBackLight = 1;
    }
    else Function();                                                            // if backlight is already turned on, execute Function routine
    attachInterrupt(digitalPinToInterrupt(FUNC_BUTTON), func, FALLING);         // enable interrupts when finished with Functon routine
    updateStationDisplay();                                                     // restore LCD station display
    funcPress = 0;                                                              // clear FUNC flag
  }

  if (setLCDBackLight == 0) lcd.setBacklight(0);                                // if backlight was set to off during execution of the Function routine, turn it off now

  if (systemTimeUpdate())                                                       // call the system time update routine to see if one second has passed since last call
  {                                                                             // enter time-based state machine every second to see if it's time to execute periodic functions
    
    switch (sysSecond)                                                          // OUTER SWITCH / CASE TO IMPLEMENT A TIME BASED EVENT LOOP TO EXECUTE FUNCTIONS ONCE PER MINUTE AT A GIVEN SECOND IN THE MINUTE
    {
      case 0:                                                                   // ZERO SECOND OF EACH MINUTE
        {
          if (!mqtt.ping())                                                     // the Adafruit MQTT broker needs to be pinged once per minute to keep the connection alive
          {
            if (!mqtt.connected())                                              // if the ping is not acknowledged, check the MQTT connection
            {
              AIOconnect();                                                     // if not connected, try to reconnect
              updateDisplayLine2(lastStationDisplay);                           // restore line 2 of the display
            }
          }
          
          switch (sysMinute)                                                    // INNER SWITCH / CASE TO IMPLEMENT A TIME BASED EVENT LOOP TO EXECUTE FUNCTIONS ONCE PER HOUR AT A GIVEN MINUTE IN THE HOUR
          {
            case 0:                                                             // ON THE HOUR
              {
                AIOpublish();                                                   // update Adafruit IO MQTT feed
                updateDisplayLine2(lastStationDisplay);                         // restore line 2 of the display
                checkAlerts();                                                  // check if an alerts needs to be sent
                break;
              }
            case 2:                                                             // TWO MINUTES PAST THE HOUR
              {
                spreadsheetUpdate();                                            // update spreadsheet hourly
                updateDisplayLine2(lastStationDisplay);
                break;
              }
            case 3:                                                             // THREE MINUTES PAST THE HOUR
              {
                ntpTimeUpdate();                                                // update system time to NTP time hourly; the system can lose time as different functions are executed, so we update via NTP each hour
                updateDisplayLine2(lastStationDisplay);  
                break;
              }
            case 4:                                                             // THREE MINUTES PAST THE HOUR
              {
                AIOconnect();                                                   // check MQTT broker connection hourly, if not connected, this function reconnects
                updateDisplayLine2(lastStationDisplay);  
                break;
              }
            case 10:                                                            // 10 MINUTES PAST THE HOUR
              {
                AIOpublish();                              
                updateDisplayLine2(lastStationDisplay);
                checkAlerts();   
                break;
              }
            case 20:                                                            // 20 MINUTES PAST THE HOUR
              {
                AIOpublish();                                                   
                updateDisplayLine2(lastStationDisplay);                         
                checkAlerts();                                                  
                break;
              }
            case 30:                                                            // 30 MINUTES PAST THE HOUR
              {
                sendStatus();                                                   // send update via email (text message) hourly
                AIOpublish();                                                  
                updateDisplayLine2(lastStationDisplay);                   
                checkAlerts();                                
                break;
              }
            case 40:                                                            // 40 MINUTES PAST THE HOUR
              {
                RFM95Initialize();                                              // initialize LoRa Radio once each hour; experienced RFM95 periodic crashes/lock ups, this fixed the problem
                AIOpublish();                                     
                updateDisplayLine2(lastStationDisplay);               
                checkAlerts();                                         
                break;
              }
            case 50:                                                            // 50 MINUTES PAST THE HOUR
              {
                AIOpublish();                                 
                updateDisplayLine2(lastStationDisplay);                    
                checkAlerts();                                   
                break;
              }
          }
          break;
        }

      case 50:                                                                  // FIFTIETH SECOND OF EACH MINUTE                            
        {                                                                       // update LCD Station Display here for fixed and cycle display modes
          if ((stationDisplayMode == CYCLE_STATION_TEMP) 
              || (stationDisplayMode == CYCLE_STATION_HUMI))
          {
            if (++stationDisplayNum == activeStations) stationDisplayNum = 0;   // if in cycle display mode, increment the station number to be displayed
          }
          updateStationDisplay();
          break;
        }
      default:
        {
          break;
        }
    }
  }

  digitalWrite(RFM95_SS, LOW);                                                  // check the LoRa Radio for a received packet every time through "loop()"
  if (rf95.available()) getRadioPacket();                                       // if there is a packet, go and get the data
  digitalWrite(RFM95_SS, HIGH);

                                                                                // if in Last Station Display mode, check to see if a new report has been received and display it
  if ((stationDisplayMode == LAST_STATION_TEMP) 
      || (stationDisplayMode == LAST_STATION_HUMI))
  {
    if (stationDisplayNum != stationNum)                                          
    {
      stationDisplayNum = stationNum;
      updateStationDisplay();
    }
  }
}


/****************************************************************************************************************************/
/******************** ROUTINES **********************************************************************************************/
/****************************************************************************************************************************/

/******************** Interrupt Service Routines ****************************************************************************/

void func(void)                                                                 // FUNC push button switch Interrupt Service Routine (ISR)
{
  funcPress = 1;                                                                // all the ISR does is set the funcPress flag
}


/******************** Ethernet Initialization Routines **********************************************************************/

void ethernetInitialize(void)
{
  if (Ethernet.begin(mac))                                                      // expecting IP address assignment via DHCP using a dummy MACID
  {
    clearDisplayLine1(F("Ethernet online"));
    lcd.setCursor(0, 1);
    lcd.print("IP:");                                                           // display the assigned IP address
    lcd.print(Ethernet.localIP());
  }
  Udp.begin(localPort);                                                         // if UDP port fails to begin it will show up as an NTP failure
  delay(2000);
}


/******************** RFM95W LoRa Radio Routines ****************************************************************************/

boolean getRadioPacket(void) /* read a packet from the LoRa Radio */
{
  byte buf[RH_RF95_MAX_MESSAGE_LEN];                                            // create a buffer for the RFM95W packet
  byte len = sizeof(buf);                                                       // number of bytes in the buffer
  if (rf95.recv(buf, &len))                                                     // if a transmission has been received...
  {
    for (int n = 0; n < 4; n++)                                                 // read the bytes from the buffer into union struct for binary - float byte mapping
    {
      airTemp.binary[n] = buf[n];
      gndTemp.binary[n] = buf[n + 4];
      humidity.binary[n] = buf[n + 8];
      pressure.binary[n] = buf[n + 12];
      batVolt.binary[n] = buf[n + 16];
    }
    stationNum = buf[20];                                                       // retrieve the ID of the station sending the transmission

    if (fahrenheit)                                                             // convert temperatures to fahrenheit if selected
    {
      gndTemp.floatingPoint = gndTemp.floatingPoint * 9 / 5 + 32;
      airTemp.floatingPoint = airTemp.floatingPoint * 9 / 5 + 32;
    }
    pressure.floatingPoint = (pressure.floatingPoint / 3386.39) + pressCal;     // convert Pascal to inches of mercury; comment out for Pascal

    station[stationNum].data[AIR_TEMP] = airTemp.floatingPoint;                 // move all data into the corresponding station data structure
    station[stationNum].data[GND_TEMP] = gndTemp.floatingPoint;
    station[stationNum].data[HUMI] = humidity.floatingPoint;
    station[stationNum].data[PRES] = pressure.floatingPoint;
    station[stationNum].data[BATT] = batVolt.floatingPoint;
    station[stationNum].stnRSSI = rf95.lastRssi();                              // save RSSI
    station[stationNum].reportMinute = sysMinute;                               // record minute of the hour report was received
    station[stationNum].lastContactTimer = 0;                                   // reset last contact timer to zero each time a report is received
    station[stationNum].lastContactEpoch = localizedEpoch;                      // record time of last report
    return true;
  }
  else return false;
}


void checkAlerts(void)  /* check if alerts need to be sent, called every ten minutes in the MAIN PROGRAM LOOP */
{  
  for (byte n = 0; n < activeStations; n++)                                     // scan alertable station data to see if an alert needs to be sent
  {
    if (++station[n].lastContactTimer > CONTACT_TIMEOUT)        sendAlert(LOST_CONTACT, n);   // increment last contact timers before testing for LOST CONTACT alert
    if ((station[n].data[BATT] - BATT_LOW) < 0)                 sendAlert(LOW_BATTERY, n);    // check station battery levels for LOW BATTERY alert
    if ((station[n].data[HUMI] - station[n].humiThreshold) > 0) sendAlert(HIGH_HUMIDITY, n);  // check station humidity for HIGH HUMIDITY alert
  }
}


void RFM95Initialize(void)
{
  digitalWrite(RFM95_SS, LOW);                                                  // select LoRa radio on SPI bus
  digitalWrite(RFM95_RST, LOW);                                                 // reset LoRa radio
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  clearDisplayLine1(F("LoRa Initialize"));
  if (!rf95.init()) updateDisplayLine2(F("Failed"));
  else
  {
    updateDisplayLine2(F("Success"));
    delay(2000);
    clearDisplayLine1(F("LoRa Frequency"));
    if (!rf95.setFrequency(RF95_FREQ)) updateDisplayLine2(F("Failed"));         // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    else updateDisplayLine2(String(RF95_FREQ, 0) + "-MHz");                     // initialize radio to 915-MHz
    rf95.setTxPower(23, false);                                                 // set TX power to max = 23dBm = 200mW
    digitalWrite(RFM95_SS, HIGH);                                               // deselect LoRa radio on SPI bus
    delay(2000);
    lcd.clear();
  }
}


/******************** Temboo Routines ***************************************************************************************/

/*  The LoRa IOT Home Environmental Monitoring System sends SMS (text) messages by sending an email to a mobile network
 *  operators email to SMS Gateway. For example, to send an SMS message to a Sprint phone, send an email to
 *  5551234567@messaging.sprintpcs.com where 5551234567 is the target mobile phone number. The total message length is
 *  160 characters, including subject.
 */

void sendStatus()
{
  String status_message;
                                                                                // build up the status message to be sent, for email to SMS, the length of the message is limited to 160 characters in total
  status_message = (
                     "Time: " + String(sysHour) + ":" + String(sysMinute) + ":" + String(sysSecond) + "\r\n"
                     + station[0].stnName + String(station[0].data[AIR_TEMP], 0) + temperatureUnits[fahrenheit] + " " + String(station[0].data[HUMI], 0) + " %" + "\r\n"
                     + station[1].stnName + String(station[1].data[AIR_TEMP], 0) + temperatureUnits[fahrenheit] + " " + String(station[1].data[HUMI], 0) + " %" + "\r\n"
                     + station[2].stnName + String(station[2].data[AIR_TEMP], 0) + temperatureUnits[fahrenheit] + " " + String(station[2].data[HUMI], 0) + " %" + "\r\n"
                     + station[3].stnName + String(station[3].data[AIR_TEMP], 0) + temperatureUnits[fahrenheit] + " " + String(station[3].data[HUMI], 0) + " %");
  sendEmail(status_message);
}


void sendAlert(byte alert, byte id)
{
  String alert_message;
  if (station[id].alerts == OFF) return;                                        // if alerts OFF for a station do nothing and return
  switch (alert)
  {
    case LOW_BATTERY:
      {
        alert_message = (                                                       // build up the alert message to be sent, for email to SMS, the length of the message is limited to 160 characters in total
                          "*****ALERT*****" + String("\r\n")                   
                          + String(station[id].stnName)
                          + " Low battery" + "\r\n"
                          + String(station[id].data[BATT], 1) + " volts");
        break;
      }
    case LOST_CONTACT:
      {
        alert_message = (                                                       // build up the alert message to be sent, for email to SMS, the length of the message is limited to 160 characters in total
                          "*****ALERT*****" + String("\r\n")        
                          + String(station[id].stnName)
                          + " Last contact:" + "\r\n"
                          + String(month(station[id].lastContactEpoch)) + "/" + String(day(station[id].lastContactEpoch)) + "/" + String(year(station[id].lastContactEpoch))
                          + " " + String(hour(station[id].lastContactEpoch)) + ":" + String(minute(station[id].lastContactEpoch)) + ":" + String(second(station[id].lastContactEpoch)));
        break;
      }
    case HIGH_HUMIDITY:
      {
        alert_message = (                                                       // build up the alert message to be sent, for email to SMS, the length of the message is limited to 160 characters in total
                          "*****ALERT*****" + String("\r\n")
                          + String(station[id].stnName)
                          + " High Humidity:" + "\r\n"
                          + String(station[id].data[HUMI], 1) + " %");
        break;
      }
  }

  sendEmail(alert_message);
}


void sendEmail(String message)                                                  // THIS FUNCTION IS GENERATED BY THE TEMBOO WEBSITE: https://temboo.com
{
  TembooChoreo SendEmailChoreo (client);

  // Invoke the Temboo client
  SendEmailChoreo.begin();

  // Set Temboo account credentials
  SendEmailChoreo.setAccountName(gatewayData[TEMBOO_ACCOUNT]);
  SendEmailChoreo.setAppKeyName(gatewayData[TEMBOO_APP_KEY_NAME]);
  SendEmailChoreo.setAppKey(gatewayData[TEMBOO_APP_KEY]);

  // Set Choreo inputs
  SendEmailChoreo.addInput("MessageBody", message);
  SendEmailChoreo.addInput("Subject", gatewayData[EM_SUBJECT]);                 // stored on SD card, read during initialiation
  SendEmailChoreo.addInput("Password", gatewayData[EM_PASSWORD]);               // stored on SD card, read during initialization
  SendEmailChoreo.addInput("Username", gatewayData[EM_USERNAME]);               // stored on SD card, read during initialization
  SendEmailChoreo.addInput("ToAddress", gatewayData[EM_TO_ADDRESS]);            // stored on SD card, read during initialization

  // Identify the Choreo to run
  SendEmailChoreo.setChoreo("/Library/Google/Gmail/SendEmail");

  // Run the Choreo; when results are available, print them to serial
  SendEmailChoreo.run();

  while (SendEmailChoreo.available())
  {
    char c = SendEmailChoreo.read();
    Serial.print(c);
  }
  SendEmailChoreo.close();
}


void spreadsheetUpdate()
{
  String new_data, hour_data;
  
  updateDisplayLine12(F("Spreadsheet"), F("Update..."));
                                                                                // the first column of the spreadsheet displays the hour an update is sent to Google Drive 
                                                                                // for the 12AM update, the date (MM/DD/YYYY) is substituted for the hour              
  if (sysHour == 0) hour_data = String(month(localizedEpoch)) 
                        + "/" + String(day(localizedEpoch)) 
                        + "/" + String(year(localizedEpoch));
  else hour_data = String(sysHour);

  new_data = (hour_data + "," +                                                 // each line in "new_data" corresponds to a column in a Google Sheet, so in this case 19 columns
              String(sysMinute) + "," +                                         // columns are separate by commas   
              String(station[0].data[AIR_TEMP], 1) + "," +                      // the first row of each column on the Google Sheet must contain a name for the Sheet to update correctly
              String(station[0].data[GND_TEMP], 1) + "," +                      // additional stations and data elements can be added as needed to this String
              String(station[0].data[HUMI], 1) + "," +
              String(station[0].data[PRES], 2) + "," +
              String(station[0].data[BATT], 1) + "," +
              String(station[0].reportMinute) + "," +
              String(station[1].data[AIR_TEMP], 1) + "," +
              String(station[1].data[HUMI], 1) + "," +
              String(station[1].reportMinute) + "," +
              String(station[2].data[AIR_TEMP], 1) + "," +
              String(station[2].data[HUMI], 1) + "," +
              String(station[2].reportMinute) + "," +
              String(station[3].data[AIR_TEMP], 1) + "," +
              String(station[3].data[HUMI], 1) + "," +
              String(station[3].reportMinute) + "," +
              String(station[4].data[AIR_TEMP], 1) + "," +
              String(station[4].reportMinute)
             );
  SendSpreadsheetUpdate(new_data);
  lcd.clear();
}


void SendSpreadsheetUpdate(String rowData)                                      // THIS FUNCTION IS GENERATED BY THE TEMBOO WEBSITE: https://temboo.com
{
  TembooChoreo AppendRowChoreo (client);

  // Invoke the Temboo client
  AppendRowChoreo.begin();

  // Set Temboo account credentials
  AppendRowChoreo.setAccountName(gatewayData[TEMBOO_ACCOUNT]);
  AppendRowChoreo.setAppKeyName(gatewayData[TEMBOO_APP_KEY_NAME]);
  AppendRowChoreo.setAppKey(gatewayData[TEMBOO_APP_KEY]);

  // Set Choreo inputs
  AppendRowChoreo.addInput("RowData", rowData);
  AppendRowChoreo.addInput("SpreadsheetTitle", gatewayData[SS_TITLE]);          // stored on SD card, read during initialization
  AppendRowChoreo.addInput("RefreshToken", gatewayData[SS_REFRESH_TOKEN]);      // stored on SD card, read during initialization
  AppendRowChoreo.addInput("ClientSecret", gatewayData[SS_CLIENT_SECRET]);      // stored on SD card, read during initialization
  AppendRowChoreo.addInput("ClientID", gatewayData[SS_CLIENT_ID]);              // stored on SD card, read during initialization

  // Identify the Choreo to run
  AppendRowChoreo.setChoreo("/Library/Google/Spreadsheets/AppendRow");

  // Run the Choreo; when results are available, print them to serial
  AppendRowChoreo.run();

  while (AppendRowChoreo.available())
  {
    char c = AppendRowChoreo.read();
    Serial.print(c);
  }
  AppendRowChoreo.close();
}


/******************** Time Keeping Routines *********************************************************************************/

/*  The LoRa IOT Home Environmental Monitoring System uses Network Timing Protocol (NTP) to establish accurate time of day,
 *  and date.
 *  
 *  The ntpTimeUpdate routine is called every one minute past the hour to update system time. In addition, ntpTimeUpdate
 *  is called after the LCD Menu functions are accessed as the system time keeping (systemTimeUpdate) stops while in the 
 *  LCD Menu function mode.
 *  
 *  systemTimeUpdate is used to increment system time in between NTP time updates.
 */

boolean systemTimeUpdate(void)
{
  if (millis() - lastSecond >= 1000)                                            // Keep track of milliseconds since Arduino started
  {
    lastSecond += 1000;                                                         // Increment lastSecond every 1000mS
    ++localizedEpoch;                                                           // Increment localized epoch every 1s
    if (++sysSecond > 59)                                                       // Increment seconds
    {
      sysSecond = 0;                                                            // reset seconds if one minute has elapsed
      if (++sysMinute > 59)                                                     // Increment minutes
      {
        sysMinute = 0;                                                          // reset minutes if one hour has elapsed
        if (++sysHour > 23)                                                     // Increment hours
        {
          sysHour = 0;                                                          // reset hours if 24 hours have elapsed
        }
      }
    }
    lcd.setCursor(0, 0);                                                        // time is displayed in Line 1 of the LCD HH:MM:SS
    if (sysHour < 10) lcd.print("0");                                           // add leading "0" when needed
    lcd.print(sysHour);
    lcd.print(':');
    if (sysMinute < 10) lcd.print("0");
    lcd.print(sysMinute);
    lcd.print(':');
    if (sysSecond < 10) lcd.print("0");
    lcd.print(sysSecond);
    return true;
  }
  else return false;
}


boolean ntpTimeUpdate(void)
{
  lcd.clear();
  lcd.print(F("NTP: "));
  if (getNTPtime())
  {
    localizedEpoch = epoch + 3600 * (timeZoneOffset + dayLightSavings);         // adjust epoch for timezone and daylight savings, in this way TimeLib.h function cane be used to calculate local time and date
    sysSecond = second(localizedEpoch);
    sysMinute = minute(localizedEpoch);
    sysHour = hour(localizedEpoch);

    lcd.print(month(localizedEpoch));                                           // display localized MM/DD/YYYY and HH:MM:SS before returning
    lcd.print('/');
    lcd.print(day(localizedEpoch));
    lcd.print('/');
    lcd.print(year(localizedEpoch));
    lcd.setCursor(5, 1);
    if (sysHour < 10) lcd.print("0");                                           // add leading "0" when needed
    lcd.print(sysHour);
    lcd.print(':');
    if (sysMinute < 10) lcd.print("0");                                         // add leading "0" when needed
    lcd.print(sysMinute);
    lcd.print(':');
    if (sysSecond < 10) lcd.print("0");                                         // add leading "0" when needed
    lcd.print(sysSecond);
    delay(2000);
    sysSecond = sysSecond + 4;                                                  // add some time to sysSecond to compensate for network and program delays
    lcd.clear();
    lastSecond = millis();                                                      // update lastSecond before returning
    return true;
  }
  else
  {
    updateDisplayLine2(F("NTP Update Fail"));
    ethernetInitialize();                                                       // if the NTP update fails, let's reinitialize the Ethernet Shield as it could be the cause
    return false;
  }
}


byte getNTPtime(void)                                                          // THIS IS 3RD PARTY CODE: created 4 Sep 2010 by Michael Margolis modified 9 Apr 2012 by Tom Igoe
{
  byte status = 0;
  sendNTPpacket(timeServer);                                                   // send an NTP packet to a time server
  delay(1000);                                                                 // wait to see if a reply is available

  if (Udp.parsePacket())                                                       // We've received a packet, read the data from it
  {
    status = 1;
    Udp.read(packetBuffer, NTP_PACKET_SIZE);                                   // read the packet into the buffer

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);         // the timestamp starts at byte 40 of the received packet and is four bytes,
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);          // or two words long. Extract the two words.
    unsigned long secsSince1900 = highWord << 16 | lowWord;                    // combine the four bytes (two words) into a long integer this is NTP time (seconds since Jan 1 1900):
    const unsigned long seventyYears = 2208988800UL;                           // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    epoch = secsSince1900 - seventyYears;                                      // subtract seventy years:
  }
  return (status);
}


void sendNTPpacket(char* address)                                              // THIS IS 3RD PARTY CODE: created 4 Sep 2010 by Michael Margolis modified 9 Apr 2012 by Tom Igoe
{                                                                              // send an NTP request to the time server at the given address
  memset(packetBuffer, 0, NTP_PACKET_SIZE);                                    // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;                                                // LI, Version, Mode
  packetBuffer[1] = 0;                                                         // Stratum, or type of clock
  packetBuffer[2] = 6;                                                         // Polling Interval
  packetBuffer[3] = 0xEC;                                                      // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123);                                              //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


/******************** EEPROM Routines ***************************************************************************************/

void eepromRead(void)
{
  clearDisplayLine1(F("EEPROM"));
  EEPROM.readBlock(eepromAddress, savedData);                                   // READ EEPROM DATA INTO DATA ARRAY
  
  if (savedData.Version != currentVersion)                                      // If new version of software or Arduino Mega not previously initialized, write default values to EEPROM
  {                                                                               
    timeZoneOffset = -5;                                                        // default US Eastern Standard Time, edited via LCD, stored in EEPROM
    dayLightSavings = 1;                                                        // default on, edited via LCD, stored in EEPROM, one for daylight savings time, zero if not daylight savings
    pressCal = 0.36;                                                            // default local pressure calibration factor, edited via LCD, stored in EEPROM
    for (byte n = 0; n < MAX_STATIONS; n++)
    {
      strncpy(station[n].stnName, "Station  ", 9); station[n].stnName[9] = 0x00;  // default station name set to "Station"
      station[n].humiThreshold = 50.0;                                          // default 50% humidity threshold for station humidity alerts
      station[n].alerts = OFF;                                                  // default alerts OFF for all stations
    }
    setLCDBackLight = 1;
    fahrenheit = 1;                                                             // default Fahrenheit
    stationDisplayMode = 1;                                                     // default to 1 for display temperature from last station to report
    stationDisplayNum = 0;                                                      // default station display is 0
    activeStations = 1;                                                         // default to 1 active station
    updateDisplayLine2(F("No Data Found"));
    delay(1000);
    updateDisplayLine2(F("Initializing..."));
    eepromUpdate();                                                             // write default values to EEPROM
    delay(1000);
  }
  
  updateDisplayLine2(F("Reading Data..."));                                     // UPDATE VARIABLES FROM EEPROM DATA ARRAY
  timeZoneOffset = savedData.TimeZoneOffset;
  dayLightSavings = savedData.DayLightSavings;
  pressCal = savedData.PressCal;
  for (byte n = 0; n < MAX_STATIONS; n++)
  {
    memcpy(station[n].stnName, savedData.Station[n], 10);
    station[n].humiThreshold = savedData.StationHumiThreshold[n];
    station[n].alerts = savedData.StationAlert[n];
  }
  setLCDBackLight = savedData.setBackLight;
  fahrenheit = savedData.Fahrenheit;
  stationDisplayMode = savedData.StationDisplayMode;
  stationDisplayNum = savedData.StationDisplayNum;
  activeStations = savedData.ActiveStations;
  delay(2000);
}


void eepromUpdate(void)                                                         // WRITE VARIABLES TO EEPROM DATA ARRAY AND THEN UPDATE EEPROM
{
  savedData.Version = currentVersion;
  savedData.TimeZoneOffset = timeZoneOffset;
  savedData.DayLightSavings = dayLightSavings;
  savedData. PressCal = pressCal;
  for (byte n = 0; n < MAX_STATIONS; n++)
  {
    memcpy(savedData.Station[n], station[n].stnName, 10);
    savedData.StationHumiThreshold[n] = station[n].humiThreshold;
    savedData.StationAlert[n] = station[n].alerts;
  }
  savedData.setBackLight = setLCDBackLight;
  savedData.Fahrenheit = fahrenheit;
  savedData.StationDisplayMode = stationDisplayMode;
  savedData.StationDisplayNum = stationDisplayNum;
  savedData.ActiveStations = activeStations;
  EEPROM.updateBlock(eepromAddress, savedData);
}


/******************** Adafruit IO Routines **********************************************************************************/

/*Refer to the Adafruit website for a tutorial on setting up an Adafruit IO dashboard and associated feeds */

void AIOconnect()                                                             // THIS FUNCTION IS FROM ADAFRUIT WEBSITE: https://learn.adafruit.com/mqtt-adafruit-io-and-you
{                                                                             // with some changes
  int8_t ret;

  clearDisplayLine1(F("Adafruit IO"));
  if ((ret = mqtt.connect()) != 0)
  {
    switch (ret)
    {
      case 1: updateDisplayLine2(F("Wrong protocol"));                  break;
      case 2: updateDisplayLine2(F("ID rejected"));                     break;
      case 3: updateDisplayLine2(F("Server unavail."));                 break;
      case 4: updateDisplayLine2(F("Bad user/passwrd"));                break;
      case 5: updateDisplayLine2(F("Not authnticated"));                break;
      case 6: updateDisplayLine2(F("Failed to subscr"));                break;
      default: updateDisplayLine2("Conn. Failed " + String(ret, DEC));  break;
    }
    if (ret >= 0)                                                           // do not do a disconnect for -1 return, calling disconnect on ret = -1 apparently causes a fault
    {
      mqtt.disconnect();
    }
  }
  else updateDisplayLine2(F("MQTT Connected!"));
  delay(2000);
  lcd.clear();
}


void AIOpublish()                                                             // THIS FUNCTION IS FROM ADAFRUIT WEBSITE: https://learn.adafruit.com/mqtt-adafruit-io-and-you
{                                                                             // Adafruit currently allows publishing of ten feeds to Adafruit IO
                                                                              // each line below starting with an "if" statement is a feed definition, these lines can be 
                                                                              // edited to change the name of the feed and the station and data set associated with the feed
  updateDisplayLine12(F("Adafruit IO MQTT"), F("Updating..."));
  if (!Air_Temp.publish((int32_t)(station[0].data[AIR_TEMP])))          updateDisplayLine2("Feed update fail");;
  if (!Gnd_Temp.publish((int32_t)(station[0].data[GND_TEMP])))          updateDisplayLine2("Feed update fail");
  if (!Humidity.publish((int32_t)(station[0].data[HUMI])))              updateDisplayLine2("Feed update fail");
  if (!Basement_Temp.publish((int32_t)(station[1].data[AIR_TEMP])))     updateDisplayLine2("Feed update fail");
  if (!Basement_Humidity.publish((int32_t)(station[1].data[HUMI])))     updateDisplayLine2("Feed update fail");
  if (!Second_Floor_Temp.publish((int32_t)(station[2].data[AIR_TEMP]))) updateDisplayLine2("Feed update fail");
  if (!Second_Floor_Humidity.publish((int32_t)(station[2].data[HUMI]))) updateDisplayLine2("Feed update fail");
  if (!First_Floor_Temp.publish((int32_t)(station[3].data[AIR_TEMP])))  updateDisplayLine2("Feed update fail");
  if (!First_Floor_Humidity.publish((int32_t)(station[3].data[HUMI])))  updateDisplayLine2("Feed update fail");
  if (!Grow_Tent_Temp.publish((int32_t)(station[4].data[AIR_TEMP])))    updateDisplayLine2("Feed update fail");
  delay(2000);
  lcd.clear();
}


/******************** LCD / Push Button Menu Routines ***********************************************************************/

void Function() /* cycle through LCD Main Menu options until one is selected, or LCD MAIN MENU LOOP is exited */
{
  byte functionNum = 1;                                                       // start with menu item 1

  functionDisplay(functionNum);                                               // display menu item 1
  delay(DEBOUNCE);

  while (1)                                                                   // LCD MAIN MENU LOOP
  {
    if (!digitalRead(UP_BUTTON))                                              // if UP pressed
    {
      functionNum++;                                                          // increment menu option by 1
      if (functionNum == END_FUNCTIONS) functionNum = START_FUNCTIONS + 1;    // if end of menu options reached, wrap around to start again
      functionDisplay(functionNum);                                           // display menu option
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // if DOWN pressed
    {
      functionNum--;                                                          // decrement menu option by 1
      if (functionNum == START_FUNCTIONS) functionNum = END_FUNCTIONS - 1;    // if start of menu options reached, wrap around to end again
      functionDisplay(functionNum);                                           // display menu option
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // if ENTER pressed
    {
      delay(DEBOUNCE);
      functionExecute(functionNum);                                           // execute selected menu option
    }

    if (!digitalRead(FUNC_BUTTON))                                            // if FUNC pressed exit LCD MAIN MENU LOOP
    {
      updateDisplayLine12(F("EEPROM"), F("Updating..."));                     // but first save persistent settings that changed to EEPROM
      eepromUpdate();
      delay(500);
      eepromRead();
      delay(500);
      break;
    }
  }                                                                           // when finished with LCD Main Menu options
  lcd.clear();                                                                // clear LCD
  ntpTimeUpdate();                                                            // and get NTP time update before returning to time-based state machine
}


void functionDisplay(byte functionToDisplay)  /* display LCD Main Menu options */
{
  clearDisplayLine1(F("Select Function:"));
  switch (functionToDisplay)
  {
    case 0:
      {
        break;
      }
    case SET_BACK_LIGHT:
      {
        updateDisplayLine2(F("Set Back Light"));                              // update line 2 of LCD with selected menu option
        break;
      }
    case SET_NUMBER_STATIONS:
      {
        updateDisplayLine2(F("Set Number Stns"));
        break;
      }
    case SET_DISPLAY_MODE:
      {
        updateDisplayLine2(F("Set Display Mode"));
        break;
      }
    case SET_ALERTS:
      {
        updateDisplayLine2(F("Set Alerts"));
        break;
      }
    case SET_DAYLIGHT_SAVINGS:
      {
        updateDisplayLine2(F("Set Daylght Svng"));
        break;
      }
    case SET_TIME_ZONE:
      {
        updateDisplayLine2(F("Set Time Zone"));
        break;
      }
    case SET_HUMI_THRES:
      {
        updateDisplayLine2(F("Set Humi Theshld"));
        break;
      }
    case SET_PRESS_SENSOR_CAL:
      {
        updateDisplayLine2(F("Set Pressure Cal"));
        break;
      }
    case SET_STATION_NAMES:
      {
        updateDisplayLine2(F("Set Station Names"));
        break;
      }
    case DISPLAY_RSSI:
      {
        updateDisplayLine2(F("Display RSSI"));
        break;
      }
    case DISPLAY_BATT:
      {
        updateDisplayLine2(F("Display Battery"));
        break;
      }
    case SET_DEGREES_F_C:
      {
        updateDisplayLine2(F("Set Temp. Units"));
        break;
      }
  }
}


void functionExecute(byte functionToExecute) /* call appropriate routine when an LCD Main Menu option is selected */
{
  switch (functionToExecute)
  {
    case 0:
      {
        break;
      }
    case SET_BACK_LIGHT:
      {        
        clearDisplayLine1(F("Set Back Light"));                               // clear LCD and display selected menu option in line 1
        doSetBackLight();                                                     // call routine to execute selected menu option
        break;
      }
    case SET_NUMBER_STATIONS:
      {
        clearDisplayLine1(F("Set Number Stns"));
        doNumberStations();
        break;
      }
    case SET_DISPLAY_MODE:
      {
        clearDisplayLine1(F("Set Display Mode"));
        doDisplayMode();
        break;
      }
    case SET_ALERTS:
      {
        clearDisplayLine1(F("Set Alerts"));
        doEnableAlerts();
        break;
      }
    case SET_DAYLIGHT_SAVINGS:
      {
        clearDisplayLine1(F("Set Daylght Svng"));
        doSetDST();
        break;
      }
    case SET_TIME_ZONE:
      {
        clearDisplayLine1(F("Set Time Zone"));
        doSetTimeZone();
        break;
      }
    case SET_HUMI_THRES:
      {
        clearDisplayLine1(F("Set Humi Theshld"));
        doSetHumidityThreshold();
        break;
      }
    case SET_PRESS_SENSOR_CAL:
      {
        clearDisplayLine1(F("Set Pressure Cal"));
        doPressureCal();
        break;
      }
    case SET_STATION_NAMES:
      {
        clearDisplayLine1(F("Set Station Name"));
        doStationNames();
        break;
      }
    case DISPLAY_RSSI:
      {
        clearDisplayLine1(F("Display RSSI"));
        doDisplayRSSI();
        break;
      }
    case DISPLAY_BATT:
      {
        clearDisplayLine1(F("Display Battery"));
        doDisplayBattery();
        break;
      }
    case SET_DEGREES_F_C:
      {
        clearDisplayLine1(F("Set Temp. Units"));
        doDegreesF_C();
        break;
      }
  }
  functionDisplay(functionToExecute);
}


void doDisplayRSSI(void)  /* display Received Signal Strength Indicator (RSSI) from LoRa Radio */
{
  byte n = 0;
  
  updateDisplayLine12(String(station[n].stnName)                              // start by displaying RSSI for station 0
    + " RSSI:", String(station[n].stnRSSI, DEC) + " dBm");
  
  while (1)                                                                   // STATION RSSI DISPLAY LOOP
  {

    if (!digitalRead(UP_BUTTON))                                              // press UP to cycle up through active stations
    {
      if (++n == activeStations) n = 0;
      updateDisplayLine12(String(station[n].stnName) 
        + " RSSI:", String(station[n].stnRSSI, DEC) + " dBm");
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to cycle down through active stations
    {
      if (--n < 0) n = activeStations - 1;
      updateDisplayLine12(String(station[n].stnName) 
        + " RSSI:", String(station[n].stnRSSI, DEC) + " dBm");
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to exit routine and return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doDisplayBattery(void) /* display battery voltage or power supply voltage of Feather board remote stations */
{
  byte n = 0;                                                                 
  
  updateDisplayLine12(String(station[n].stnName)                              // start by displaying battery voltage for station 0
    + " Batt.:", String(station[n].data[BATT], 2) + " volts");
  
  while (1)                                                                   // STATION BATTERY VOLTAGE DISPLAY LOOP
  {

    if (!digitalRead(UP_BUTTON))                                              // press UP to cycle up through active stations
    {
      if (++n == activeStations) n = 0;
      updateDisplayLine12(String(station[n].stnName) 
        + " Batt.:", String(station[n].data[BATT], 2) + " volts");
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to cycle down through active stations
    {
      if (--n < 0) n = activeStations - 1;
      updateDisplayLine12(String(station[n].stnName) 
        + " Batt.:", String(station[n].data[BATT], 2) + " volts");
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to exit routine and return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doNumberStations(void) /* set number of active stations in the system, number of active stations is always <= MAX_STATIONS */
{
  byte n = activeStations;                                                    // get current number of active stations, default is 1, there always has to be at least one active station
  
  updateDisplayLine2(String(n, DEC));
  
  while (1)                                                                   // SET NUMBER ACTIVE STATIONS LOOP
  {
    if (!digitalRead(UP_BUTTON))                                              // press UP to increment number of active stations
    {
      if (++n > MAX_STATIONS) n = MAX_STATIONS;
      updateDisplayLine2(String(n, DEC));
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to decrement number of active stations
    {
      if (--n <  1) n = 1;
      updateDisplayLine2(String(n, DEC));
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to save new number of active stations and return to LCD MAIN MENU LOOP
    {
      activeStations = n;
      delay(DEBOUNCE);
      break;
    }
  }
}


void doDisplayMode(void)  /* set station display mode for line 2 of LCD */
{
  byte n = stationDisplayMode;                                                // get current display mode

  updateDisplayLine2(DISPLAY_MODE[n]);                                        // and display it

  while (1)                                                                   // SET STATION DISPLAY MODE LOOP
  {
    if (!digitalRead(UP_BUTTON))                                              // press UP to cycle up through display mode options
    {
      if (++n == END_DISPLAY_MODES) n = 1;
      updateDisplayLine2(DISPLAY_MODE[n]);
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to cycle down through display mode options
    {
      if (--n == START_DISPLAY_MODES) n = END_DISPLAY_MODES - 1;
      updateDisplayLine2(DISPLAY_MODE[n]);
      delay(DEBOUNCE);
    }
    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to save display mode option and return to LCD MAIN MENU LOOP
    {
      stationDisplayMode = n;
      delay(DEBOUNCE);
      break;
    }
  }
                                                                              
  if ((stationDisplayMode == FIXED_STATION_TEMP)                              // if a Fixed Station Display Mode has been selected, station now needs to be specified
      || (stationDisplayMode == FIXED_STATION_HUMI))
  {
    byte n = stationDisplayNum;                                               // get number of station currently being displayed

    updateDisplayLine12(F("Select Station:"), station[n].stnName);

    while (1)                                                                 // SET STATION FOR FIXED DISPLAY
    {
      if (!digitalRead(UP_BUTTON))                                            // press UP to increment station number
      {
        if (++n == activeStations) n = 0;
        updateDisplayLine2(station[n].stnName);
        delay(DEBOUNCE);
      }

      if (!digitalRead(DOWN_BUTTON))                                          // press DOWN to decrement station number
      {
        if (--n <  0) n = activeStations - 1;
        updateDisplayLine2(station[n].stnName);
        delay(DEBOUNCE);
      }

      if (!digitalRead(ENTR_BUTTON))                                          // press ENTER to save station number for fixed display and return to LCD MAIN MENU LOOP
      {
        stationDisplayNum = n;
        delay(DEBOUNCE);
        break;
      }
    }
  }
}


void doEnableAlerts(void) /* enable and disable alerts for each station */
{
  byte n = stationDisplayNum;                                                 // get the number of the station currently being displayed
                                                                              
  updateDisplayLine12(String(station[n].stnName)                              // display the alert status for this station
    + " Alert:", alertMode[station[n].alerts]);

  while (1)                                                                   // SET ALERTS STATION SELECT LOOP
  {
    if (!digitalRead(UP_BUTTON))                                              // press UP to increment station number and display alert status
    {
      if (++n == activeStations) n = 0;
      updateDisplayLine12(String(station[n].stnName) 
        + " Alert:", alertMode[station[n].alerts]);
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to decrement station number and display alert status
    {
      if (--n <  0) n = activeStations - 1;
      updateDisplayLine12(String(station[n].stnName) 
        + " Alert:", alertMode[station[n].alerts]);
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to select station for editing alert status
    {
      delay(DEBOUNCE);
      while (1)                                                               // ALERT ON / OFF LOOP
      {
        if (!digitalRead(UP_BUTTON))                                          // press UP to cycle alert ON and OFF
        {
          if (station[n].alerts == ON) station[n].alerts = OFF;
          else (station[n].alerts = ON);
          updateDisplayLine2(alertMode[station[n].alerts]);
          delay(DEBOUNCE);
        }

        if (!digitalRead(DOWN_BUTTON))                                        // press DOWN to cycle alert ON and OFF
        {
          if (station[n].alerts == ON) station[n].alerts = OFF;
          else (station[n].alerts = ON);
          updateDisplayLine2(alertMode[station[n].alerts]);
          delay(DEBOUNCE);
        }
        if (!digitalRead(ENTR_BUTTON))                                        // press ENTER to save alert status, and select another station for alert status editing
        {
          delay(DEBOUNCE);
          break;
        }
      }
    }

    if (!digitalRead(FUNC_BUTTON))                                            // press FUNC to return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doSetHumidityThreshold(void) /* set humidity threshold for each station, exceeding threshold causes an alert if station alerts are enabled */
{
  {
    byte n = stationDisplayNum;                                               // get the number of the station currently being displayed
                                                                              

    updateDisplayLine12(String(station[n].stnName)                            // display the humidity threshold for this station
      + " Thres:", String(station[n].humiThreshold) + " %");

    while (1)                                                                 // SET HUMIDITY THRESHOLD STATION SELECTION LOOP
    {
      if (!digitalRead(UP_BUTTON))                                            // press UP to increment station number
      {
        if (++n == activeStations) n = 0;
        updateDisplayLine12(String(station[n].stnName)                        // display the humidity threshold for this station
          + " Thres:", String(station[n].humiThreshold) + " %");
        delay(DEBOUNCE);
      }

      if (!digitalRead(DOWN_BUTTON))                                          // press DOWN to decrement station number
      {
        if (--n <  0) n = activeStations - 1;
        updateDisplayLine12(String(station[n].stnName)                        // display the humidity threshold for this station
          + " Thres:", String(station[n].humiThreshold) + " %");
        delay(DEBOUNCE);
      }

      if (!digitalRead(ENTR_BUTTON))                                          // press ENTER to change humidity threshold for selected station
      {
        delay(DEBOUNCE);
        while (1)                                                             // SET STATION HUMIDITY THRESHOLD LOOP
        {
          if (!digitalRead(UP_BUTTON))                                        // press UP to increment humidity threshold by 1%
          {
            if (++station[n].humiThreshold > 100) station[n].humiThreshold = 0;
            updateDisplayLine2(String(station[n].humiThreshold) + " %");
            delay(DEBOUNCE);
          }

          if (!digitalRead(DOWN_BUTTON))                                      // press DOWN to decrement humidity threshold by 1%
          {
            if (--station[n].humiThreshold < 0) station[n].humiThreshold = 100;
            updateDisplayLine2(String(station[n].humiThreshold) + " %");
            delay(DEBOUNCE);
          }
          if (!digitalRead(ENTR_BUTTON))                                      // press ENTER to save humidity threshold, and select another station for humidity threshold editing 
          {
            delay(DEBOUNCE);
            break;
          }
        }
      }

      if (!digitalRead(FUNC_BUTTON))                                          // press FUNC to return to LCD MAIN MENU LOOP
      {
        delay(DEBOUNCE);
        break;
      }
    }
  }
}


void doSetDST(void) /* set day light savings ON and OFF */
{
  if (dayLightSavings) updateDisplayLine2("ON");                              // display current day light savings status
  else updateDisplayLine2("OFF");

  while (1)                                                                   // SET DAYLIGHT SAVINGS LOOP
  {
    if (!digitalRead(UP_BUTTON) | !digitalRead(DOWN_BUTTON))                  // if UP or DOWN pressed, change state of day light savings from ON to OFF, or OFF to ON
    {
      if (dayLightSavings)
      {
        dayLightSavings = 0;
        updateDisplayLine2(F("OFF"));
      }
      else
      {
        dayLightSavings = 1;
        updateDisplayLine2(F("ON"));
      }
      delay(DEBOUNCE);
    }
    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to save day light savings setting and return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doSetTimeZone(void)  /* set world time zone */
{
  int temp = 12 + timeZoneOffset;                                             // get current time zone offset

  updateDisplayLine2(TIME_ZONES[temp]);                                       // and display time zone major city and offset from UTC
  
  while (1)                                                                   // SET TIME ZONE LOOP
  {

    if (!digitalRead(UP_BUTTON))                                              // press UP to cycle up through time zones
    {
      if (++temp > 24) temp = 0;
      updateDisplayLine2(TIME_ZONES[temp]);                                   // display time zone major city and offset from UTC
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to cycle down through time zones
    {
      if (--temp < 0) temp = 24;
      updateDisplayLine2(TIME_ZONES[temp]);                                   // display time zone major city and offset from UTC
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to save selected time zone and return to LCD MAIN MENU LOOP
    {
      timeZoneOffset = temp - 12;
      delay(DEBOUNCE);
      break;
    }
  }
}


void doPressureCal(void)  /* set pressure sensor calibration offset, compensations pressure readings for station altitude */ 
{
  updateDisplayLine2("Offset: " + String(pressCal, 2));                       // get current pressure sensor calibration offset value
  
  while (1)                                                                   // SET PRESSURE SENSOR CALIBRATION OFFSET LOOP
  {                                                                           // use UP / DOWN to adjust local pressure reading so it matches pressure reported by NOAA or weather.com for your city / zip code
    if (!digitalRead(UP_BUTTON))                                              // press UP to increment calibration offset by 0.01inHg 
    {
      pressCal = pressCal + 0.01;
      updateDisplayLine2("Offset: " + String(pressCal, 2));                   // display calibration offset value
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to decrement calibration offset bu 0.01inHg
    {
      pressCal = pressCal - 0.01;
      updateDisplayLine2("Offset: " + String(pressCal, 2));                   // display calibration offset value
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doStationNames(void) /* edit station names */
{
  int x = 0;                                                                  // temporary int for station number
  int y;                                                                      // temporary int for character number in station name, maximum number of characters in station name = 9
  char z = 0x20;                                                              // station character, hex ascii character

  updateDisplayLine12("Station " + String(x) 
    + " Name:", station[x].stnName);                                          // display current name of first station (station 0)

  while (1)                                                                   // STATION SELECTION LOOP
  {
    if (!digitalRead(UP_BUTTON))
    {
      if (++x == activeStations) x = 0;                                       // press UP to cycle up through station names, and display each name
      updateDisplayLine12("Station " + String(x) 
        + " Name:", station[x].stnName);
      delay(DEBOUNCE);
    }

    if (!digitalRead(DOWN_BUTTON))                                            // press DOWN to cycle down through station names, and display each name
    {
      if (--x <  0) x = activeStations - 1;
      updateDisplayLine12("Station " 
        + String(x) + " Name:", station[x].stnName);
      delay(DEBOUNCE);
    }

    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to edit displayed station name
    {
      lcd.blink();                                                            // set LCD cursor to blink to identify character being edited
      y = 0;
      lcd.setCursor(y, 1);                                                    // move cursor to the first character in the station name
      delay(DEBOUNCE);
      
      while (1)                                                               // EDIT STATION NAME LOOP
      {
        if (!digitalRead(UP_BUTTON))                                          // press UP to cycle up through displayable ASCII character set
        {
          if (++z == 0x7B) z = 0x20;
          lcd.print(String(z));                                               // display the next ASCII character
          lcd.setCursor(y, 1);                                                // as cursor auto increments, we need to keep moving it back to the character being edited until we are done
          station[x].stnName[y] = z;                                          // save the character
          delay(DEBOUNCE);
        }

        if (!digitalRead(DOWN_BUTTON))                                        // press DOWN to cycle down through displayable ASCII character set
        {
          if (--z < 0x20) z = 0x7A;                                           
          lcd.print(String(z));                                               // display the next ASCII character
          lcd.setCursor(y, 1);                                                // as cursor auto increments, we need to keep moving it back to the character being edited until we are done                     
          station[x].stnName[y] = z;                                          // save the character
          delay(DEBOUNCE);
        }

        if (!digitalRead(ENTR_BUTTON))                                        // press ENTER to move on to the next character
        {
          if (++y > 8) y = 0;
          lcd.setCursor(y, 1);
          delay(DEBOUNCE);
        }

        if (!digitalRead(FUNC_BUTTON))                                        // press FUNC to stop editing the selected station name and return to the STATION SELECTION LOOP
        {
          lcd.noBlink();
          delay(DEBOUNCE);
          break;
        }
      }
    }
    if (!digitalRead(FUNC_BUTTON))                                            // press FUNC to exit STATION SELECTION LOOP and return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doSetBackLight(void) /* turn off LCD back light */
{
  if (setLCDBackLight) updateDisplayLine2("ON");                              // display current status of LCD back light setting
  else updateDisplayLine2("OFF");

  while (1)                                                                   // SET LCD BACK LIGHT LOOP
  {
    if (!digitalRead(UP_BUTTON) | !digitalRead(DOWN_BUTTON))                  // if UP or DOWN pressed, change state of back light setting from ON to OFF, or OFF to ON
    {
      if (setLCDBackLight == 1)
      {
        setLCDBackLight = 0;
        updateDisplayLine2(F("OFF"));
      }
      else
      {
        setLCDBackLight = 1;
        updateDisplayLine2(F("ON"));
      }
      delay(DEBOUNCE);
    }
    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


void doDegreesF_C(void) /* set temperature display to F or C */
{
  if (fahrenheit) updateDisplayLine2("Fahrenheit");                           // current temperature display settings
  else updateDisplayLine2("Centigrade");

  while (1)                                                                   // SET TEMPERATURE UNITS LOOP
  {
    if (!digitalRead(UP_BUTTON) | !digitalRead(DOWN_BUTTON))                  // if UP or DOWN pressed, change temperature units from F to C, or C to F
    {
      if (fahrenheit)
      {
        fahrenheit = 0;
        updateDisplayLine2(F("Centigrade"));
      }
      else
      {
        fahrenheit = 1;
        updateDisplayLine2(F("Fahrenheit"));
      }
      delay(DEBOUNCE);
    }
    if (!digitalRead(ENTR_BUTTON))                                            // press ENTER to return to LCD MAIN MENU LOOP
    {
      delay(DEBOUNCE);
      break;
    }
  }
}


/******************** SD Card File Routines *********************************************************************************/

void readGatewayData(void) /* reads authentication and other static data from SD Card for Temboo email and Google Sheets */
{
  int n = 0;
  char character;

  updateDisplayLine12("SD Card", "Reading Data...");
  myFile = SD.open(F("GW_Data.txt"));                                         // look for the GW_Data.txt file
  if (!myFile) updateDisplayLine12("SD File Error", "GW_Data.txt");           // if not found, display an error

  while (myFile.available())                                                  // keep reading the file as long there are bytes available in the file
  {
    while (myFile.read() != '[') {}                                           // find a new line delimited by "["
    character = myFile.read();
    while (character != ']')                                                  // read and save characters to a data set in gatewayData array until "]" character is encountered
    {
      gatewayData[n] += character;
      character = myFile.read();
    }
    n++;                                                                      // get the next data set
  }
  myFile.close();                                                             // close the file when all data sets read
  delay(2000);
}


/******************** LCD Routines ******************************************************************************************/

void updateStationDisplay(void) /* updates the station display (LCD Line 2) for FIXED and CYCLE station display modes */
{
  
  if ((stationDisplayMode == FIXED_STATION_TEMP)                                        // test if display mode is fixed or cycle temperature
      || (stationDisplayMode == CYCLE_STATION_TEMP) 
          || (stationDisplayMode == LAST_STATION_TEMP))                                 
  {
    if (station[stationDisplayNum].lastContactTimer > CONTACT_TIMEOUT)                  // test if contact lost with the sstation
    {
      lastStationDisplay = String(station[stationDisplayNum].stnName) + " " + "******"; // if contact lost, blank out data display with "******"
    }
    else lastStationDisplay = String(station[stationDisplayNum].stnName)  + " " +       // otherwise, display the temperature data
                              String(station[stationDisplayNum].data[AIR_TEMP], 1) +
                              temperatureUnits[fahrenheit];                             // get the temperature units F or C
  }
  
  else if ((stationDisplayMode == FIXED_STATION_HUMI)                                   // test if display mode is fixed or cycle hunidity
           || (stationDisplayMode == CYCLE_STATION_HUMI) 
               || (stationDisplayMode == LAST_STATION_HUMI))                       
  {
    if (station[stationDisplayNum].lastContactTimer > CONTACT_TIMEOUT)                  // test if contact lost with the station
    {
      lastStationDisplay = String(station[stationDisplayNum].stnName) + " " + "******"; // if contact lost, blank out humidity display with "******"
    }
    else lastStationDisplay = String(station[stationDisplayNum].stnName) + " " +        // otherwise, display humidty data
                              String(station[stationDisplayNum].data[HUMI], 1) + " %";                               
  }
  updateDisplayLine2(lastStationDisplay);                                               // update LCD Line 2
}


/* LCD Utilities to simplify the code */

void updateDisplayLine12( String Line_1, String Line_2)                       // updates both Line 1 & 2 of the LCD
{
  lcd.clear();
  lcd.print(Line_1);
  lcd.setCursor(0, 1);
  lcd.print(Line_2);
}


void updateDisplayLine2(String Line_2)                                        // updates Line 2 while leaving Line 1 unchaged
{
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  lcd.print(Line_2);
}


void updateDisplayLine1(String Line_1)                                        // updates Line 1 while leaving Line 2 unchanged
{
  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 0);
  lcd.print(Line_1);
}


void clearDisplayLine1(String Line_1)                                         // clear LCD and display new Line 1
{
  lcd.clear();
  lcd.print(Line_1);
}


void clearDisplayLine2(String Line_2)                                         // clear LCD and display new Line 2
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(Line_2);
}
/******************** END OF PROGRAM ****************************************************************************************/
