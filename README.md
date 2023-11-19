# Temp-Humidity-Pressure-Dewpoint-Server
This is a program for reporting temperature, humidity, pressure, and dewpoint
sensor measurements.  It is designed to run on an Adafruit ESP32 Feather V2
connected to a Sparkfun BME280 Atmospheric Sensor Breakout Board (SEN-15440)
via a Qwiic interface cable.  It is intended to be built using the Arduino IDE.

With the associated web page, it can plot temperature, relative humidity, 
barometric pressure, and dewpoint for the past five days using Highcharts.  It
can also provide real-time temperature, humidity, pressure, and dewpoint data
in response to an html get request.  In addition, it can send an email message
reporting a relative humidity over a specified threshold.  The threshold can
be set via an html get request.  It can also report the Wi-Fi RSSI level, 
current time of day, and free heap size, and respond to a restart request.  On
startup, and every three days (approx.), it sets its real-time clock via an NTP
request.

The index.html file that charts the data via Highcharts must be stored in a
data subdirectory inside the Arduino sketch folder.  The HTML file is
stored on the ESP32 SPIFFS (Serial Peripheral Interface Flash File System).
Note that I used the LittleFS Arduino library instead of the SPIFFS library. See
https://randomnerdtutorials.com/esp32-web-server-spiffs-spi-flash-file-system/
if you're not familiar with how to implement SPIFFS.

For other useful tutorials that may help understand and modify this software, see
https://randomnerdtutorials.com/projects-esp32/ .
