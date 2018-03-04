/*
 *  This sketch sends ads1115 current sensor data via HTTP POST request to thingspeak server.
 *  It needs the following libraries to work (besides the esp8266 standard libraries supplied with the IDE):
 *
 *  - https://github.com/adafruit/Adafruit_ADS1X15
 *
 *  designed to run directly on esp8266-01 module, to where it can be uploaded using this marvelous piece of software:
 *
 *  https://github.com/esp8266/Arduino
 *
 *  2015 Tisham Dhar
 *  licensed under GNU GPL
 *
 *  https://www.hackster.io/whatnick/esp8266-iot-energy-monitor-b199ed
 *
 */

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include <Adafruit_ADS1015.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

//#include "version.h"

/*
0	D3
1	TX	SERIAL
2	D4	(TXD1)      / LED?
3	RX	SERIAL
4	D2	I2C	SDA
5	D1	I2C SCL
12  D6	(MISO)
13	D7	SPI_MOSI
14	D5	SPI_CLK
15  D8	TFT_CS
16	D0	(USER/WAKE) / LED
*/

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     15 //NodeMCU D8
#define TFT_RST    -1 // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     2  //NodeMCU D4
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);


// replace with your channel's thingspeak API key,
String apiKey = "OLRDWXWO6QS3ISBX";

const char* host = "ESP-EnergyMonitor";

//WIFI credentials go here
//const char* ssid     = "XXXXXXXXXXX";
//const char* password = "XXXXXXXXXXXXXXXXXXXXX";
#include "../../../private_data.h"
const char* ssid     = WiFi_SSID;
const char* password = WiFi_Password;


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//5 NodeMCU D1
//4 NodeMCU D2

//Maximum value of ADS
#define ADC_COUNTS 32768
#define PHASECAL 1.7
//#define VCAL 0.6
//#define VCAL 1.49
//#define VCAL 1.7
//#define VCAL 2  //117.85
//#define VCAL 1.8  //124.79
//#define VCAL 1.84 //  122.12
//#define VCAL 1.62 // 138.71
#define VCAL 1.8
#define ICAL 0.003

const char* server = "api.thingspeak.com";
WiFiClient client;

double filteredI;
double lastFilteredV,filteredV; //Filtered_ is the raw analog value minus the DC offset
int sampleV;                 //sample_ holds the raw analog read value
int sampleI;

double offsetV;                          //Low-pass filter output
double offsetI;                          //Low-pass filter output

double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
double phaseShiftedV; //Holds the calibrated phase shifted voltage.
int startV; //Instantaneous voltage at start of sample window.
double sqV,sumV,sqI,sumI,instP,sumP; //sq = squared, sum = Sum, inst = instantaneous
boolean lastVCross, checkVCross; //Used to measure number of times threshold is crossed.

long previousMillis = 0;
long interval = 20000;

double squareRoot(double fg)
{
  double n = fg / 2.0;
  double lstX = 0.0;
  while (n != lstX)
  {
    lstX = n;
    n = (n + fg / n) / 2.0;
  }
  return n;
}

void calcVI(unsigned int crossings, unsigned int timeout)
{

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(st==false)                                   //the while loop...
  {
     startV = ads.readADC_Differential_2_3();                    //using the voltage waveform
     if ((abs(startV) < (ADC_COUNTS*0.55)) && (abs(startV) > (ADC_COUNTS*0.45))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = ads.readADC_Differential_2_3();                 //Read in raw voltage signal
    sampleI = ads.readADC_Differential_0_1();                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);
  filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
  filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.
  float multiplier = 0.125F; /* ADS1115 @ +/- 4.096V gain (16-bit results) */
  //float multiplier = 0.1875F; /* ADS1115 @ +/- 6.144V gain (16-bit results) */
  double V_RATIO = VCAL * multiplier;
  Vrms = V_RATIO * squareRoot(sumV / numberOfSamples);

  double I_RATIO = ICAL * multiplier;
  Irms = I_RATIO * squareRoot(sumI / numberOfSamples);

  //Calculation power values
  //realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  realPower = sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------
}

double calcIrms(unsigned int Number_of_Samples)
{
  /* Be sure to update this value based on the IC and the gain settings! */
  float multiplier = 0.125F;    /* ADS1115 @ +/- 4.096V gain (16-bit results) */
  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = ads.readADC_Differential_0_1();

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
  //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;
    //filteredI = sampleI * multiplier;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  Irms = squareRoot(sumI / Number_of_Samples)*multiplier;

  //Reset accumulators
  sumI = 0;
//--------------------------------------------------------------------------------------

  return Irms;
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextWrap(false); // Allow text to run off right edge
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3);


  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  tft.print("Connecting to ");
  Serial.println(ssid);
  tft.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tft.print(".");
  }

  Serial.println("");
  tft.println("");
  Serial.println("WiFi connected");
  tft.println("WiFi connected");
  Serial.println("IP address: ");
  tft.println("IP address: ");
  Serial.println(WiFi.localIP());
  tft.println(WiFi.localIP());

  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //ads.setGain(GAIN_TWOTHIRDS);        // 2/3x gain   +/- 6.144V  1 bit = 2mV      0.1875mV
  ads.begin();


  ArduinoOTA.setHostname(host);
  ArduinoOTA.onStart([]() {
                          Serial.println("Start");
                    	  tft.setTextSize(2);
                          tft.fillScreen(ST7735_BLACK);
                          tft.setTextColor(ST7735_WHITE);
                          int width = tft.width();
                          int height = tft.height(); //font size 8 *2
                          String text = "OTA";
                          tft.setCursor(width / 2 -  (6 * 2 * text.length() / 2 )  , ( height - (8 *2) ) / 3 * 1 - 8 * 2  /2 );
                          tft.println(text);
                          //getTextBounds
                          text = "Programming!";
                          tft.setCursor(width / 2 -  (6 * 2 * text.length()  / 2 )  ,( height - (8 *2) ) / 3  * 2 - 8 * 2  /2 );
                          tft.println(text);

                    });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int width = tft.width();
    int height = tft.height(); //font size 8 *2
                      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));

                      tft.setCursor(width / 2 -  (6 * 2 * 4 /*length*/ / 2 )  , ( height - (8 *2) ) / 3 * 3 - 8 * 2  /2 );
                      tft.fillRect(tft.getCursorX(),tft.getCursorY(), 6*2 *4 ,16 , ST7735_BLACK);
                      tft.printf("%u%%", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
                      Serial.printf("Error[%u]: ", error);
                      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                      else if (error == OTA_END_ERROR) Serial.println("End Failed");
                    });
  ArduinoOTA.onEnd([]() { // do a fancy thing with our board led at end
                          Serial.println("\nEnd");
                          tft.fillScreen(ST7735_BLACK);
                          tft.setTextColor(ST7735_WHITE);
                          int width = tft.width();
                          int height = tft.height(); //font size 8 *2
                          String text = "OTA";
                          tft.setCursor(width / 2 -  (6 * 2 * text.length()  / 2 )  ,height / 2 - 8 * 2  );
                          tft.println(text);
                          //getTextBounds
                          text = "Completed!";
                          tft.setCursor(width / 2 -  (6 * 2 * text.length()  / 2 )  ,height / 2 + 8 * 2  );
                          tft.println(text);
                        });

   ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });

   /* setup the OTA server */
   ArduinoOTA.begin();

}

void loop() {

  unsigned long currentMillis = millis();

  ArduinoOTA.handle();

  if(currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;

	  Serial.println("reading.... ADC " + String(millis()));
	  calcVI(20,2000);
	  Serial.println("done!" + String(millis()));

	  tft.fillScreen(ST7735_BLACK);
	  tft.setTextColor(ST7735_WHITE);
	  tft.setCursor(0, 0);
	  tft.setTextSize(1);
	  tft.println("realPower");
	  tft.setTextSize(3);
	  tft.println(realPower);

	  tft.setTextSize(1);
	  tft.println("Vrms");
	  tft.setTextSize(3);
	  tft.println(Vrms);

	  tft.setTextSize(1);
	  tft.println("Irms");
	  tft.setTextSize(3);
	  tft.println(Irms);

	  tft.setTextSize(1);
	  tft.println("powerFactor");
	  tft.setTextSize(3);
	  tft.println(powerFactor);

	  if (client.connect(server,80)) {  //   "184.106.153.149" or api.thingspeak.com
   	    Serial.println("http-post");
   	    String postStr = apiKey;
   	           postStr +="&field1=";
   	           postStr += String(realPower);
   	           postStr +="&field2=";
   	           postStr += String(Vrms);
   	           postStr +="&field3=";
   	           postStr += String(Irms);
   	           postStr +="&field4=";
   	           postStr += String(powerFactor);
   	           postStr += "\r\n\r\n";

		 client.print("POST /update HTTP/1.1\n");
		 client.print("Host: api.thingspeak.com\n");
		 client.print("Connection: close\n");
		 client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
		 client.print("Content-Type: application/x-www-form-urlencoded\n");
		 client.print("Content-Length: ");
		 client.print(postStr.length());
		 client.print("\n\n");
		 client.print(postStr);
		 tft.setTextSize(1);
		 tft.println("thingspeak - ok!");
	  }
	  client.stop();

	  //Serial.println("Waiting...");
	  // thingspeak needs minimum 15 sec delay between updates
	  //delay(20000);
  }
  delay(50);
}
