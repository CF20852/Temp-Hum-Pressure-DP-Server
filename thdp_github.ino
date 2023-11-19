/*******************************************************************************
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

Copyright (c) 2023 Robert F. Fleming, III


Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*******************************************************************************/

#include <WiFi.h>
#include <SparkFunBME280.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "time.h"
#include <ESP32Time.h>
#include <ESP_Mail_Client.h>

BME280 mySensorA;  //Uses default I2C address 0x77

// Replace with your network details
const char *ssid = "yourssid";
const char *password = "yourwi-fipassword";


// IP addresses for Wi-Fi initialization--replace with your IP configuration
IPAddress local_IP(192, 168, 1, 150);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
int portNumber = 8080;

// Cloudflare DNS
IPAddress primarydns(1, 1, 1, 1);
IPAddress secondarydns(1, 0, 0, 1);

// Google DNS
// IPAddress primarydns(8, 8, 8, 8);
// IPAddress secondarydns(8, 8, 4, 4);

/* The smtp host name e.g. smtp.gmail.com for GMail or smtp.office365.com for
   Outlook or smtp.mail.yahoo.com */
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials--replace with your credentials */
#define AUTHOR_EMAIL "yourgmailid"
#define AUTHOR_PASSWORD "yourgmailpassword"

/* Recipient's email--replace with your recipient address */
#define RECIPIENT_EMAIL "recipient-email"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Declare the message class */
SMTP_Message message;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

// Create AsyncWebServer object
AsyncWebServer server(portNumber);

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

// Set up the Wi-Fi link
void setupWiFi(void);

// Set up things to use the hardware timer alarm feature to signal time to take a sample
// We’ll have a timer increment every microsecond, so 300*10^6 increments = 300 seconds = 5 minutes
hw_timer_t * timer0 = NULL;
const uint64_t alarmTimeout = 300UL * 1000000UL;
 
volatile boolean recordSample = true;

void ARDUINO_ISR_ATTR Timer0_ISR() {
  recordSample = true;
}

// Declare function that sets up a timer to trigger an alarm that will signal
// that it's time to take a new atmospheric parameter sample
void setupTimer();

// Declare function that initialize LittleFS, which will store this server's web page
void initLittleFS();

// Declare function that initializes the BME280 sensor
void initBME();

// Use the ESP32Time library for the real time clock
ESP32Time rtc(0);  // offset in seconds (not needed if using gmtOffset_sec)
const int gmtOffset_sec = -7 * 3600;  // Phoenix, Arizona, USA
const int daylightOffset_sec = 0;     // Arizona doesn't use DST except on the
                                      // Navajo and maybe Hopi Reservations
const char *ntpServer = "pool.ntp.org";
const unsigned long oneHour = 59 * 60;  // actually 59 minutes
const unsigned long threeDays = 3 * 86400;

float rhThresh=50.0;

// Declare function that sets the time using ntp
void setTheClock(void);

// Variable to save current epoch time
unsigned long epochTime;

// Variable to store the next epoch time at which to set the clock
unsigned long clockResetTime;

// Variable to store the next epoch time at which to send another humidity alert
unsigned long sendAnotherAlertTime = 0;

// Function to get a hardware random number generator number between 0 and 2^14-1
unsigned long myRandomNo() {
  return esp_random() >> 18;
}

// Declare function that sets up the SMTP email client
void setupEmail(void);

// Arrays to store the samples
#define NSAMPLES 1440
uint64_t timeStamps[NSAMPLES];
double dataPoints[NSAMPLES][4];

/*  sampleHeadPtr is initially zero.  It increments after data are written at the
	buffer location it points to.
	Thus it points to the next location to be written.  If the sampleHeadPtr has
	not rolled over, i.e., gone through all the numbers and returned to zero,
	the oldest sample is at 0.  If the sampleHeadPtr has rolled
	over, the oldest sample is at (sampleHeadPtr + 1) % NSAMPLES. 

	sampleTailPtr is initially zero if sampleHeadPtr has not rolled over from

	NSAMPLES-1 to zero.  If
	sampleHeadPtr has rolled over, sampleTailPtr is initially set to sampleHeadPtr.
	It increments 	after data are read at the buffer location it points to.
	When pulling data from the buffer, 	reading should stop when sampleTailPtr
	is equal to sampleHeadPtr, because the data at sampleHeadPtr
	has already been read out and new data has not been written there yet.*/

unsigned int sampleHeadPtr = 0;
unsigned int sampleTailPtr = 0;
boolean rollOver = false;

// Variables for sending chunked chart data
const char *jsonHeader = "{\"enviro_records\":[";
const char *jsonFooter = "]}\r\n";
StaticJsonDocument<256> readings;
char jsonString[256];

void setup() {
  // Start the serial communication
  Serial.begin(115200);

  setupWiFi();

  setupTimer();

  // Initialize LittleFS
  initLittleFS();

  // Set the RTC using NTP
  setTheClock();
  clockResetTime = rtc.getEpoch() + threeDays + (unsigned long)(esp_random() >> 18);
  Serial.print("Clock will be set again at epoch time = ");
  Serial.println(clockResetTime);

  Serial.println(rtc.getEpoch());         //  (long)    1609459200 epoch without offset
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));   // (String) returns time with specified format 

  Serial.println("Initializing BME280...");
  initBME();
	
	Serial.println("Setting up email...");
	setupEmail();

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/", LittleFS, "/");

  server.on("/current", HTTP_GET, sendCurrentReadings);

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "Minimum free heap recorded = " + String(esp_get_minimum_free_heap_size()) + "<br>" + 
                                    "Current free heap= " + String(esp_get_free_heap_size()) + "<br>");
    // Serial.println("Minimum free heap observed = " + String(esp_get_minimum_free_heap_size()));
    // Serial.println("Current free heap observed = " + String(esp_get_free_heap_size()));
  });

  server.on("/rssi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "Wi-Fi RSSI = " + String(WiFi.RSSI()));
    // Serial.println("Wi-Fi RSSI = " + String(WiFi.RSSI()));
  });

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "The current ESP32 RTC time is " + rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  });
  
  
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", "HTTP GET request to restart this device has been accepted.<br>");
    delay(2000);
    esp_restart();
  });

  // Request for chart of readings
  server.on("/chart.json", HTTP_GET, sendSensorLog);

  // Handle a relative humidity threshold change request
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputVal;

    if (request->hasParam("rh_thresh")) {
      inputVal = request->getParam("rh_thresh")->value();
			rhThresh = inputVal.toFloat();

      request->send(200, "text/html", "HTTP GET request to change relative humidity threshold with value: " + inputVal +
                                      "% accepted.<br>");
    }

  });

  // Start server
  server.begin();

  Serial.println("Made it through setup()...");
}

void loop() {

  if (recordSample) {

    epochTime = rtc.getEpoch();

    timeStamps[sampleHeadPtr] = epochTime;
    //Serial.println(timeStamps[sampleHeadPtr]);

    dataPoints[sampleHeadPtr][0] = mySensorA.readTempF();;
    dataPoints[sampleHeadPtr][1] = mySensorA.readFloatHumidity();
    dataPoints[sampleHeadPtr][2] = mySensorA.dewPointF();
    dataPoints[sampleHeadPtr][3] = mySensorA.readFloatPressure();

    //Serial.println(sampleHeadPtr);

    sampleHeadPtr = (sampleHeadPtr + 1) % NSAMPLES;

    // if sampleHeadPtr equals 0 after incrementing then
    // we have come back to the beginning of the circular buffer storage array
    if (sampleHeadPtr == 0) {
      rollOver = true;
      Serial.println("Rollover occurred");
    }
    recordSample = false;

    // Reconnect to Wi-Fi if necessary
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Attempting to reconnect to Wi-Fi");
      while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, password);
        delay(3000);
      }
      Serial.println("\nRe-connected.");
    }
		
		float relHum = mySensorA.readFloatHumidity();
		if ((epochTime > sendAnotherAlertTime) && (relHum > rhThresh)) {
			sendAnotherAlertTime = epochTime + oneHour;
			sendHumidityAlert(relHum);
		}

    // Set the clock using NTP about once every 74.25 hours (3 days + randomly up to 4.55 hours)
    if (epochTime > clockResetTime) {
      setTheClock();
      clockResetTime = epochTime + threeDays + (unsigned long)(esp_random() >> 18);
    }
  }
}

// Set up the Wi-Fi connection
void setupWiFi(void) {
  Serial.println("Initializing Wi-Fi mode...");
  WiFi.mode(WIFI_STA);

  Serial.println("Initializing Wi-Fi IP config...");
  if (!WiFi.config(local_IP, gateway, subnet, primarydns, secondarydns)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected.");

  // Print the IP address
  Serial.println("");
  Serial.print("Connected to WiFi! IP address: ");
  Serial.println(WiFi.localIP());
}

// Set up a timer to trigger an alarm that will signal that it's time to take a new atmospheric parameter sample
void setupTimer() {
    timer0 = timerBegin(0, 80, true);  // 80 MHz clock / 80 = 1 timer increment/microsecond
    timerAttachInterrupt(timer0, &Timer0_ISR, false);  // timer 0 will cause an interrupt served by Timer0_ISR
    timerAlarmWrite(timer0, alarmTimeout, true);  // timer 0 will auto-reload
    timerAlarmEnable(timer0);
}

//Initialize LittleFS, which will store this server's web page
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}

// Initialize the BME280 sensor
void initBME() {
  Wire.begin();

  mySensorA.setI2CAddress(0x77);  //The default for the SparkFun Environmental Combo board is 0x77 (jumper open).
  //If you close the jumper it is 0x76
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  if (mySensorA.beginI2C() == false) {
    Serial.println("Sensor A connect failed");
  } else {
    Serial.println("Sensor A initialization appears to have succeeded.");
  }
}

// Function that sets the time using ntp
void setTheClock(void) {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    rtc.setTimeStruct(timeinfo); 
  }
}

// Function to set up the SMTP email client
void setupEmail(void) {
	  /*  Set the network reconnection option */
  MailClient.networkReconnect(true);

  /** Enable the debug via Serial port
   * 0 for no debugging
   * 1 for basic level debugging
   *
   * Debug port can be changed via ESP_MAIL_DEFAULT_DEBUG_PORT in ESP_Mail_FS.h
   */
  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);
}

void sendCurrentReadings(AsyncWebServerRequest *request) {
  double tempF, relHum, dewPt, press;

  tempF = round(mySensorA.readTempF() * 10.0) / 10.0;
  relHum = round(mySensorA.readFloatHumidity());
  dewPt = round(mySensorA.dewPointF() * 10.0) / 10.0;
  press = mySensorA.readFloatPressure() / 100.0;

  // Create a plain text response with current readings
  String response = ("Current Atmospheric Conditions<br>");
  response += "Temperature: " + String(tempF) + " &#8457<br>";
  response += "Humidity: " + String(relHum) + " %<br>";
  response += "Dew Point: " + String(dewPt) + " &#8457<br>";
  response += "Pressure: " + String(press) + " hPa<br>";

  // Send the plain text response
  request->send(200, "text/html", response);
}

void sendSensorLog(AsyncWebServerRequest *request) {

  Serial.print("Received request from client with IP: ");
  Serial.println(request->client()->remoteIP());

  AsyncWebServerResponse *response = request->beginChunkedResponse("application/json",
  // The following lambda function will be called each time the chunk transmitter is ready to
  // send a new chunk of the response.  This function will put data in 'buffer'.  The
  // 'index' variable tells us how many characters have been sent in the current response,
  // and starts at 0 when no characters have been sent yet.  Experiments have shown that
  // 'maxlen' is typically more than 5000 characters, but we'll keep it below 25% full.
  [](uint8_t *buffer, size_t maxlen, size_t index) -> size_t {
    static unsigned int sampleTailPtr;
    static boolean sentAllSamples;

    // Initialize the buffer as a null-terminated string
    buffer[0] = '\0';

    // If the caller tells us the index = 0, we're at the beginning of the chunked transmission
    if (index == 0) {

      // Flag to tell us when we're reading to send the JSON closing brackets
      sentAllSamples = false;

      // Start sending at the oldest sample
      if (!rollOver) {
        sampleTailPtr = 0;  // reset the log line index when we get a new request
      } else {
        sampleTailPtr = sampleHeadPtr;
      }

      // Copy the JSON document header to the buffer
      strcpy((char *)buffer, jsonHeader);
      // Now fall through and start sending samples, if there are any
    }

    // If we haven't sent all the samples yet...
    if (!sentAllSamples) {

      // The following line handles the case where the header is in the buffer
      size_t bufferLength = strlen((char *)buffer);

      // Fill the buffer a little over 1/4 full
      while (bufferLength < (maxlen / 4)) {

        // If there is a sample, build a JSON object containing an array row for the sample
        if (timeStamps[sampleTailPtr] > 0) {
          readings["unix_time"] = timeStamps[sampleTailPtr] * 1000UL;  // multiply by 1000 for Javascript time
          readings["tempF"] = round(dataPoints[sampleTailPtr][0] * 10.0) / 10.0;
          readings["relHum"] = round(dataPoints[sampleTailPtr][1] * 10.0) / 10.0;
          readings["dewPt"] = round(dataPoints[sampleTailPtr][2] * 10.0) / 10.0;
          readings["press"] = round(dataPoints[sampleTailPtr][3] * 10.0) / 1000.0;  // convert Pa to hPa
          serializeJson(readings, jsonString);

          // If there is at least one more sample to send, add a comma to separate JSON array rows
          if (((sampleTailPtr + 1) % NSAMPLES != sampleHeadPtr) && (timeStamps[sampleTailPtr] > 0)) {
            strcat(jsonString, ",");
          }

          // Write the JSON string to the chunk buffer and account for the increased buffer length
          snprintf((char *)buffer + bufferLength, maxlen, jsonString);
          bufferLength = strlen((char *)buffer);

          // Update the circular buffer tail pointer
          sampleTailPtr = (sampleTailPtr + 1) % NSAMPLES;
        }

        // Keep adding samples until the buffer level hits threshhold or we
        // run out of samples
        if ((sampleTailPtr == sampleHeadPtr) || (timeStamps[sampleTailPtr] == 0)) {
          break;
        }
      }

      // If we've sent all the samples, append the JSON closing brackets
      if ((sampleTailPtr == sampleHeadPtr) || (timeStamps[sampleTailPtr] == 0)) {
        strncat((char *)buffer + bufferLength, jsonFooter, maxlen);
        sentAllSamples = true;
      }

      // Let the caller know how many characters are in the buffer
      return strlen((char *)buffer);
    }

    //All the samples and the footer have been sent, so return 0 from the lambda function
    return 0;
  });

  request->send(response);
}

void sendHumidityAlert(float RH) {

  /* Declare the Session_Config for user defined session credentials */
  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";

  /*
  Set the NTP config time
  For times east of the Prime Meridian use 0-12
  For times west of the Prime Meridian add 12 to the offset.
  Ex. American/Denver GMT would be -6. 6 + 12 = 18
  See https://en.wikipedia.org/wiki/Time_zone for a list of the GMT/UTC timezone offsets
  */
  config.time.gmt_offset = 19;
  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.day_light_offset = 0;

  /* Clear the message */
  message.clear();

  /* Set the message headers */
  message.sender.name = F("ESP32 Environmental Sensor");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = F("ESP32 Humidity Alert");
  message.addRecipient(F("Chip"), RECIPIENT_EMAIL);
    
  /*Send HTML message*/
  /*String htmlMsg = "<div style=\"color:#2f4468;\"><h1>Hello World!</h1><p>- Sent from ESP board</p></div>";
  message.html.content = htmlMsg.c_str();
  message.html.content = htmlMsg.c_str();
  message.text.charSet = "us-ascii";
  message.html.transfer_encoding = Content_Transfer_Encoding::enc_7bit;*/

   
  //Send raw text message
  String textMsg = "Alert from " + local_IP.toString() + ":" + String(portNumber) + ":  Relative Humidity exceeds " + String(rhThresh) + "%.  Current RH = " + String(RH) + "%";
  message.text.content = textMsg.c_str();
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;


  /* Connect to the server */
  if (!smtp.connect(&config)){
    ESP_MAIL_PRINTF("Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!smtp.isLoggedIn()){
    Serial.println("\nNot yet logged in.");
  }
  else{
    if (smtp.isAuthenticated())
      Serial.println("\nSuccessfully logged in.");
    else
      Serial.println("\nConnected with no Auth.");
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    ESP_MAIL_PRINTF("Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status){
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success()){
    // ESP_MAIL_PRINTF used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. AVR, SAMD, ESP32 and ESP8266.
    // In ESP8266 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      
      ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}