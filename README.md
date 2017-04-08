# LoRa-IOT-Home-Environment-Monitoring-System

The LoRa IOT Home Environmental Monitoring System consists of an Arduino Mega based IOT-to-Internet gateway and Arduino Feather based remote stations with environmental sensors. The remote stations communicate wirelessly with the gateway using LoRa radios.

The system enables a homeowner to monitor the home environment via an internet accessible dashboard, receive periodic SMS environmental notifications, receive realtime SMS alerts when monitored environmental parameters exceed preset thresholds, and log environmental data to the cloud.

As a homeowner, other than fire, the greatest fear is water, whether it's a sudden and catastrophic pipe or fixture failure, or a slow leak that goes undetected over time. If you are away from home when a sudden and catastrophic event occurs there may not be much that can be done, however, returning home sooner rather later or calling a neighbor due to humidity readings suddenly spiking could mitigate the damage. A slow leak can be more insidious, potentially resulting in structural damage and black mold. The LoRa IOT Home Environment Monitoring System monitors household humidity levels over time and provides insight to how normal humidity levels fluctuate throughout the year with changes in outdoor humidity, and daily with normal household activities such as clothes drying, showers & etc. Armed with this knowledge, high humidity thresholds can be intelligently set for the various remote stations enabling the LoRa IOT Home Environment Monitoring System to send an alert when abnormal humidity readings are detected. SMS alerts and hourly updates appear on a smart phone Lock Screen so are immediately available without requiring an application to be launched or browsing to a dashboard.

The LoRa IOT Home Environment Monitoring System is built using readily available hardware modules and a few individual components such as switches and connectors. The parts for the system can be obtained from Adafruit, Digikey and Sparkfun; in many cases, Adafruit and Sparkfun parts are also available from Digikey. Competent soldering skills are needed to assemble the hardware, in particular to solder SMT uFL connectors to the LoRa radio breakout and Feather boards. The Arduino code is written in a procedural style and liberally commented to allow easy extension of functionality.

The objectives for this project included the following:

1) Create a home IOT gateway Minimum Viable Product (MVP)
2) Create a User Interface (UI) framework for Arduino projects
3) Assess suitability of LoRa technology for the home environment

The MVP establishes the minimum hardware configuration and minimum set of software features needed to produce a really useful IOT based home monitoring system.

The intent of the UI framework objective is to make all user based configuration parameters accessible without requiring code changes and recompilation. User configuration changes are made via a simple UI consisting of four push button switches and a 2 x 16 LCD. The LCD is also used to display environmental variables locally, six display modes are available. Third party service authentication information (Temboo and Google Drive) is stored in a simple text file on an SD memory card. The current versions of remote station and gateway software mostly achieve the goal of user configuration without the need for code compilation, exceptions are noted.

LoRa is a long range, low power wireless technology developed by Semtech for the Internet of Things. Semtech builds LoRa technology into its chipsets, and these chipsets available on the breakout and Feather boards from Adafruit used in the LoRa IOT Home Environment Monitoring System. LoRa enables the realization of simple, reliable, and energy efficient wireless telemetry links in the home between remote stations and a hub/gateway. Why not use WiFi? WiFi is overkill for environmental monitoring around the home in terms of data rates, it's a power hog, and, in my experience, WiFi links based on simplified clients are not always reliable. LoRa operates at low data rates in the 915-MHz ISM band. The lower data rates combined with the propagation characteristics at 915-MHz (versus 2.4- and 5-GHz for WiFi) enable very reliable radio links to be established around the largest home environments.

The LoRa IOT Home Environment Monitoring System consists of the following subsystems:

1) LoRa IOT Gateway - wireless sensor hub and gateway to the Internet
2) Indoor Station - wireless temperature and humidity sensor
3) Outdoor Station - wireless temperature, humidity and pressure sensor

The LoRa IOT Gateway receives periodic readings from the remote station and implements the following functions:

1) Local time keeping based on NTP
2) UI Functions and local display via 2 x 16 LCD
3) Alerts for over / under threshold readings via SMS
4) Hourly temperature and humidity updates via SMS
5) Real-time dashboard (ten minute updates) via Adafruit IO
6) Long term cloud storage on Google Drive in Google Sheets

Remote stations sleep most of the time, waking up periodically to take environmental readings and send them to the gateway.

Communications in the LoRa IOT Home Environment Monitoring System is currently one-way from the remote station to the gateway; that is, the gateway is always receiving and does not transmit, and the remote stations periodically transmit but receivers are never enabled. As the LoRa wireless links have proven to be very reliable in a home environment the complications that come with implementing a two-way protocol in the system did not seem warranted. The remote station send updates asynchronously at intervals of approximately ten minutes. The transmissions are very short, so the probability of transmissions from two stations colliding is very small. An SMS alert is sent if the gateway does not receive transmissions from a given remote within a configurable number of 10 minute intervals. Battery levels are monitored for battery powered station (currently only the Outdoor Station), and an alert sent for a low battery level. A Receive Signal Strength Indicator (RSSI) for each remote is accessible via the LCD functions to check wireless link quality.
