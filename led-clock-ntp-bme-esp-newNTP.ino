#ifndef STASSID
#define STASSID "asushotyn"                            // set your SSID
#define STAPSK  "hotyn777"                        // set your wifi password
#endif

/* Configuration of NTP */
#define MY_NTP_SERVER "ua.pool.ntp.org"           
#define MY_TZ "EET-2EEST,M3.5.0/3,M10.5.0/4"   

bool ntpGood = true;
int updateFails = 0;
int updPeriod = 10;

/* Necessary Includes */
#include <ESP8266WiFi.h>            // we need wifi to get internet access
#include <time.h>                   // for time() ctime()
#include <coredecls.h> // optional settimeofday_cb() callback to check on server

#include <Ticker.h>

Ticker flipper;
Ticker DisplayOrderer;

//#include <Adafruit_Sensor.h>
//#include <Adafruit_BME280.h>


#include <EnvironmentCalculations.h>  
#include <BME280I2C.h>
#include <Wire.h>

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool bmeReady;
unsigned long bmeReinit, bmeReinitDelay = 1000;


#define SEALEVELPRESSURE_HPA (1013.25)
#define ALTITUDESEALEVEL_M (186)

//Adafruit_BME280 bme; // I2C
  

float Humidity = 0;
float Temperature = 0;
float Pressure = 0;

int TempCorr = -1;

unsigned long dhtDisplay = 0;
bool readDHT = false;


const char O_RTC = 0;
const char O_DHT = 1;
const char O_PRESS = 2; 

enum Order_t {o_rtc,o_dhc} ; 
uint8_t printOrder = O_DHT;
uint8_t displayOrder = O_DHT;



#include <TM1637Display.h>

const char* host = "esp8266-03";

/* Globals */
time_t now;                         // this are the seconds since Epoch (1970) - UTC
tm tm;                              // the structure tm holds time information in a more convenient way

int LightSensorPin = A0;    // select the input pin for the potentiometer
int LightSensorValue = 0;  // variable to store the value coming from the sensor
int Brightness = 0;

// Module connection pins (Digital Pins)
#define CLK1 D6 
#define DIO1 D5  

// Module connection pins (Digital Pins)
#define CLK2 D0  
#define DIO2 D7  


TM1637Display display(CLK1, DIO1);
TM1637Display display2(CLK2, DIO2);

unsigned long hours = 0;
unsigned long minutes = 0;
unsigned long seconds = 0;

bool dots = false;


void changeDisplayToDHT(){
    displayOrder = O_DHT;
    DisplayOrderer.detach();
    DisplayOrderer.attach(20,changeDisplayToPRESS);  
    Serial.println("displayOrder = O_DHT");
  }

void changeDisplayToPRESS(){
    displayOrder = O_PRESS;
    DisplayOrderer.detach();
    DisplayOrderer.attach(20,changeDisplayToDHT);  
    Serial.println("displayOrder = O_PRESS");
  }

void updateDHT(){
  //DHTUpdater.detach();
  //DHTUpdater.attach(20,updateDHT);
  //float h = 0;
  //float t = 0;
  //float p = 0;
  
  float t(NAN), h(NAN), p(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);

	bme.read(p, t, h, tempUnit, presUnit);
	
	EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
    EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
   
	p =  EnvironmentCalculations::EquivalentSeaLevelPressure(ALTITUDESEALEVEL_M, t, p, envAltUnit, envTempUnit);
	

  //t = bme.readTemperature();
  //p = bme.readPressure() / 100.0F;
  //h = bme.readHumidity();
  
    Humidity = h;
    Temperature = t + TempCorr; 
    Pressure = p;
    
    
    char botString[200] = "";
    sprintf(botString, "{\"humid\":%.1f, \"temper\":%.1f, \"press\":%.1f, \"TempCorr\":%d}",  Humidity, Temperature,Pressure,TempCorr);
    Serial.println(botString);
   /*
   String tgMess = "{\"humid\":%.1f, \"temper\":%.1f, \"press\":%.1f, \"TempCorr\":%d}";
   bool sent = false;
        
        sent = bot.sendMessage(tgchat_id, tgMess, "Markdown");

        if (sent) {
          Serial.println("MESS was successfully sent");
        } else {
          Serial.println("MESS was not sent");
        }
     */   
}

void flip() {
	//countTime();
	updateDHT();
	showTime();
	dots = !dots;
	displayOut();
	display2Out();
	setBrightness();
}

void displayOut(){
	//
	//      A
	//     ---
	//  F |   | B
	//     -G-
	//  E |   | C
	//     ---
	//      D
	
	const uint8_t digitToSegment[] = {
	 // XGFEDCBA
	  0b00111111,    // 0
	  0b00000110,    // 1x
	  0b01011011,    // 2
	  0b01001111,    // 3x
	  0b01100110,    // 4x
	  0b01101101,    // 5
	  0b01111101,    // 6x
	  0b00000111,    // 7x
	  0b01111111,    // 8
	  0b01101111,    // 9x
	  0b01110111,    // A
	  0b01111100,    // b
	  0b00111001,    // C
	  0b01011110,    // d
	  0b01111001,    // E
	  0b01110001     // F
	  };


	const uint8_t digitToSegmentUW[] = {
	 // XGFEDCBA
	  0b00111111,    // 0
	  0b00110000,    // 1x
	  0b01011011,    // 2
	  0b01111001,    // 3x
	  0b01110100,    // 4x 0b01100110
	  0b01101101,    // 5
	  0b01101111,    // 6x
	 // XGFEDCBA
	  0b00111000,    // 7x
	  0b01111111,    // 8
	  0b01111101,    // 9x
	  
	  0b01110111,    // A
	  0b01111100,    // b
	  0b00111001,    // C
	  0b01011110,    // d
	  0b01111001,    // E
	  0b01110001     // F
	  };


	unsigned long timeDisplay = hours * 100 + minutes;	
	uint8_t dotPos = 0b00000000;
	if  (dots) {
		dotPos = 0b01100000;
		} 
	if (!ntpGood) {
		dotPos += 0b10000000;
		}
    
   // display.showNumberDecEx(timeDisplay, dotPos, true);
    // Вивести перевернуту цифру в позиції ..:х. 
	if (hours>0){
		const uint8_t segments[4] = {
			digitToSegment[hours/10] + (ntpGood?0b00000000:0b10000000),
			digitToSegment[hours - (hours/10)*10] + (dots?0b10000000:0b00000000),
			digitToSegmentUW[minutes/10] + (dots?0b10000000:0b00000000), 
			digitToSegment[minutes - (minutes/10)*10]
		};
		display.setSegments(segments, 4, 0);
		
	} else {
		const uint8_t segments[4] = {
			digitToSegment[minutes/10] + (ntpGood?0b00000000:0b10000000), 
			digitToSegment[minutes - (minutes/10)*10] + (dots?0b10000000:0b00000000),
			digitToSegmentUW[seconds/10] + (dots?0b10000000:0b00000000), 
			digitToSegment[seconds - (seconds/10)*10], 
		};
		display.setSegments(segments, 4, 0);
	}

    
    
    
}

void display2Out(){
	if (displayOrder == O_DHT) {
		dhtDisplay = int(Humidity) * 100 + int(round(Temperature));	
 		display2.showNumberDecEx(dhtDisplay, (0b01010000), true);
     return;
    }
    
    String PressStr;
  
  int PressInt;
  int8_t dotP = 0b00000000;
  if (Pressure < 1000) {
      PressInt = int(Pressure*10);
      String PressStr = String(int(Pressure*10)) ;
      dotP = 0b00100000;
    }else {
      PressInt = int(Pressure);
      String PressStr = String(int(Pressure));
      dotP = 0b00000000;
    }

  //Serial.printf("P= %.1f", Pressure);
  display2.showNumberDecEx(PressInt, dotP, true);
}



void setBrightness(){
	
	LightSensorValue = analogRead(LightSensorPin);
	float k = float(LightSensorValue)/1024 * 7;
	
	Brightness = 7 - k;
	
	//Serial.printf("Light Val = %d, k = %0.2f, Brigh = %d", LightSensorValue,k, Brightness);
	//Serial.println();
	
	display.setBrightness(Brightness);
    display2.setBrightness(Brightness/2);
	
}

void showTime() {
  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time
  //Serial.print("year:");
  //Serial.print(tm.tm_year + 1900);  // years since 1900
  //Serial.print("\tmonth:");
  //Serial.print(tm.tm_mon + 1);      // January = 0 (!)
  //Serial.print("\tday:");
  //Serial.print(tm.tm_mday);         // day of month
  Serial.print("\thour:");
  Serial.print(tm.tm_hour);         // hours since midnight  0-23
  Serial.print("\tmin:");
  Serial.print(tm.tm_min);          // minutes after the hour  0-59
  Serial.print("\tsec:");
  Serial.print(tm.tm_sec);          // seconds after the minute  0-61*
  //Serial.print("\twday");
  //Serial.print(tm.tm_wday);         // days since Sunday 0-6
  if (tm.tm_isdst == 1)             // Daylight Saving Time flag
    Serial.print("\tDST");
  else
    Serial.print("\tstandard");
  Serial.println();

  
  hours = tm.tm_hour; //(UTC.hour() + TZ) > 23 ? UTC.hour() + TZ - 24 : UTC.hour() + TZ;
  //hours = hours < 0 ? hours + 24: hours;
  minutes = tm.tm_min; //UTC.minute();
  seconds = tm.tm_sec; // UTC.second();
  
  
}

uint32_t sntp_startup_delay_MS_rfc_not_less_than_60000 () {
  randomSeed(A0);
  return random(5000);
}

uint32_t sntp_update_delay_MS_rfc_not_less_than_15000 () {
  return 1 * 60 * 60 * 1000UL; // 12 hours
}

void time_is_set(bool from_sntp /* <= this optional parameter can be used with ESP8266 Core 3.0.0*/) {
  Serial.print(F("time was sent! from_sntp=")); Serial.println(from_sntp);
}


void setup() {
  Serial.begin(57600);
  Serial.println("\nNTP TZ DST - bare minimum");

  configTime(MY_TZ, MY_NTP_SERVER); // --> Here is the IMPORTANT ONE LINER needed in your sketch!
	settimeofday_cb(time_is_set); // optional: callback if time was sent

  // start network
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print ( "." );
  }
  Serial.println("\nWiFi connected");
  // by default, the NTP will be started after 60 secs
	
  Wire.begin();

	 // BME280
 
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    bmeReady = bme.begin();  
    if (!bmeReady) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        bmeReinit = millis();
    }
   
    flipper.attach(1, flip);
    DisplayOrderer.attach(20,changeDisplayToPRESS);
 
}

void loop() {
  if (!bmeReady && ((millis()-bmeReinit)>bmeReinitDelay) ) {
	  Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
	   bmeReady = bme.begin();
	   bmeReinit = millis();
	  }
  //updateDHT();
  //showTime();
  //displayOut();
  //display2Out();
  
  //delay(1000); // dirty delay
 // setBrightness();
}
