/***************************************************************
 * MySensors Meteo Gateway-Node
 * 
 * Created by: Adam Stolarczyk
 * 2020-06-04
 * License: GPL-3.0
 * 
 * Code used:
 * Henrik Ekblad <henrik.ekblad@mysensors.org> / Copyright (C) 2013-2019 Sensnology AB / MQTTGateway Example
 * Nathan Seidle / SparkFun Electronics / Weather Shield Example Code
 * BH1750 library example
 * 
 * REVISION HISTORY
 * Version 1.0 - Adam Stolarczyk
 * Version 1.1 - Adam Stolarczyk
 *****************************************************************/

#define MY_DEBUG                                                // Enable debug prints to serial monitor
#define DEBUG                                                   // Enable debug prints to serial monitor
//#define MY_DISABLED_SERIAL                                    // Enable this in sketch if you want to use TX(1), RX(0) as normal I/O pin
#define MY_SPLASH_SCREEN_DISABLED                               // disable splash screen - This saves 120 bytes of flash.
#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE

#define MY_GATEWAY_MQTT_CLIENT                                  // Enable gateway ethernet module type with MQTT

// MQTT settings
#define MY_MQTT_PUBLISH_TOPIC_PREFIX   "mqtt-gw-meteo-out"      // Set this node's publish topic prefix
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "mqtt-gw-meteo-in"       // Set this node's subscribe topic prefix
#define MY_MQTT_CLIENT_ID              "mqtt-gw-meteo"          // Set MQTT client id
#define MY_MQTT_USER                   "********"               // Enable these if your MQTT broker requires username/password
#define MY_MQTT_PASSWORD               "********"               // Enable these if your MQTT broker requires username/password

// Network settings
#define MY_IP_ADDRESS             192,168,0,204                        // Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_GATEWAY_ADDRESS     192,168,0,1                          // If using static ip you can define Gateway and Subnet address as well
#define MY_IP_SUBNET_ADDRESS      255,255,255,0                        // If using static ip you can define Gateway and Subnet address as well
#define MY_MAC_ADDRESS            0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xCC   // The MAC address can be anything you want but should be unique on your network, 0xCC = 204
#define MY_CONTROLLER_IP_ADDRESS  192,168,0,7                          // MQTT broker ip address or url. Define one or the other.
#define MY_PORT                   1883                                 // The MQTT broker port to to open

//#define MY_W5100_SPI_EN 4  // W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)


#define MY_DEFAULT_LED_BLINK_PERIOD 300      // Set blinking period
#define DWELL_TIME 50                       

#define SKETCH_NAME "Meteo GW"
#define SKETCH_VERSION "1.1"

// Hardware pin definitions
#define WSPEED_PIN  3
#define RAIN_PIN    2
#define WDIR_PIN    A0

// Weather hardware constants
#define RAIN_BUCKET_SIZE  0.2794  // Bucket size for tipping bucket rain sensor = 0.2794mm
#define WIND_SPEED_CLICK  2.4     // 1 click per second = 2.4 km/h

// MySensors child
#define CHILD_ID_RAIN         1
#define CHILD_ID_RAINRATE     2
#define CHILD_ID_LIGHT        3
#define CHILD_ID_WIND_SPEED   4
#define CHILD_ID_WIND_GUST    5
#define CHILD_ID_WIND_DIR     6
#define CHILD_ID_WIND_DIR_TXT 7

#include <BH1750.h>
#include <Wire.h>
#include "RTClib.h"
#include <Ethernet.h>
#include <MySensors.h>

RTC_DS1307 rtc;
BH1750     lightSensor;

// Global Variables
#define WIND_SPD_SAMPLES  30          // 1 sample @ 10 seconds interval * 5 minutes = 5*60/10 = 30
#define WIND_DIR_SAMPLES  30          // 1 sample @ 10 seconds interval * 5 minutes = 5*60/10 = 30

byte currentSecond                    = 0;
byte secondsCounter                   = 0;
uint16_t rainClicksTotal              = 0;
byte windGustAvg_30m[30]              = {}; // 30 byte to keep track of 30 minute average 
byte windSpdAvg[WIND_SPD_SAMPLES]     = {}; // 30 bytes to keep track of 5 minute average @ 10 second interval
uint16_t windDirAvg[WIND_DIR_SAMPLES] = {}; // 30 bytes to keep track of 5 minute average @ 10 second interval
unsigned long lastRainCheck           = millis();
unsigned long lastWindCheck           = millis();
byte windSpdSampleIdx                 = 0;
byte windDirSampleIdx                 = 0;

// MySensors generic message
MyMessage msg((uint8_t)0, (mysensors_data_t)0);

// Volatile Variables - volatiles are subject to modification by IRQs
volatile unsigned long lastWindIRQ = 0;
volatile unsigned int  windClicks  = 0;
volatile unsigned long lastRainIRQ = 0;
volatile unsigned int  rainClicks  = 0;

/*****************************************************************************************
 * Function prototypes
 *****************************************************************************************/
void     rainIRQ();
void     windSpeedIRQ();
void     presentation();
void     setup();
void     loop();
int      getWindSpeed();
int      getWindDir();
float    getLightLevel();
byte     getWindSpeedAvg();
uint16_t getWindDirAvg();
byte     getWindGust();
float    getRainRate();
float    getRainTotal();
void     storeRain();
String   deg2compass(int dir);

/*****************************************************************************************
 * rainIRQ()
 * 
 * Rain IRQ routine
 *****************************************************************************************/
void rainIRQ() {
    if (millis() - lastRainIRQ > 10) { // Ignore switch-bounce glitches less than 10ms
        lastRainIRQ = millis();
        rainClicks++;                  // 1 click = = 0.2794mm of rain
    }
}

/*****************************************************************************************
 * windSpeedIRQ
 * 
 * Wind IRQ routine
 *****************************************************************************************/
void windSpeedIRQ() {
    if (millis() - lastWindIRQ > 10) { // Ignore switch-bounce glitches less than 10ms (142MPH max reading)
        lastWindIRQ = millis();
        windClicks++;                  // 1 click per 1 second = 2.4km/h wind speed
    }
}

/*****************************************************************************************
 * presentation()
 * 
 * Presentation
 *****************************************************************************************/
void presentation() {
    sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
    wait(DWELL_TIME);
    present(CHILD_ID_RAIN, S_RAIN);
    wait(DWELL_TIME);
    present(CHILD_ID_RAINRATE, S_RAIN);
    wait(DWELL_TIME);
    present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
    wait(DWELL_TIME);
    present(CHILD_ID_WIND_SPEED, S_WIND);
    wait(DWELL_TIME);
    present(CHILD_ID_WIND_GUST, S_WIND);
    wait(DWELL_TIME);
    present(CHILD_ID_WIND_DIR, S_WIND);
    wait(DWELL_TIME);
    present(CHILD_ID_WIND_DIR_TXT, S_INFO);
    wait(DWELL_TIME);
}

/*****************************************************************************************
 * setup()
 * 
 * Setup
 *****************************************************************************************/
void setup() {
    #ifndef MY_DEBUG
        Serial.begin(115200);
    #endif
    #ifdef DEBUG
        Serial.print(F(SKETCH_NAME));
        Serial.print(F(" "));
        Serial.println(F(SKETCH_VERSION));
    #endif

    Wire.begin();
    Wire.setClock(10000);     // Slow down I2C bus to 10kHz for long cable support

    // DS1307 RTC setup
    #ifdef DEBUG
        Serial.println(F("Starting RTC DS1307..."));
    #endif
    if (rtc.begin()) {
        if (!rtc.isrunning()) {
            #ifdef DEBUG
                Serial.println(F("RTC is NOT running!"));
            #endif
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));    // Set RTC to the date & time this sketch was compiled
        }    
    } else {
        #ifdef DEBUG
            Serial.println(F("RTC DS1307 not found!"));
            Serial.println(F("Guru meditation #00000001.00000001 and Kernel panic"));
        #endif
        while(1);
    }
    
    // Pin definition
    pinMode(WSPEED_PIN, INPUT);
    pinMode(WDIR_PIN,   INPUT);
    pinMode(RAIN_PIN,   INPUT);

    // Light sensor setup
    #ifdef DEBUG
        Serial.println(F("Starting light sensor BH1750..."));
    #endif
    lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
    lightSensor.setMTreg(69);

    // Restore saved rain data if any
    #ifdef DEBUG
        Serial.print(F("Loading stored data..."));
    #endif
    if (rtc.readnvram(0) == 0xEE) {                                       // Check control byte
        #ifdef DEBUG
            Serial.print(F("data found "));
        #endif
        if (rtc.readnvram(1) == rtc.now().day()) {                        // Check day
            rainClicksTotal = rtc.readnvram(2) * 0xFF + rtc.readnvram(3); // Read data
            #ifdef DEBUG
                Serial.print(F("and loaded. Rain="));
                Serial.print(rainClicksTotal * RAIN_BUCKET_SIZE);
                Serial.print(F("mm."));
            #endif
        } else {
            #ifdef DEBUG
                Serial.print(F("but too old. Skipping."));
            #endif
        }
    } else {
        #ifdef DEBUG
            Serial.print(F("not found."));
        #endif
    }
    #ifdef DEBUG
        Serial.println(F(" Done."));
    #endif

    // Attach external interrupt pins to IRQ functions
    attachInterrupt(0, rainIRQ,      FALLING);
    attachInterrupt(1, windSpeedIRQ, FALLING);
        
    // Turn on interrupts
    interrupts();

    // Initialize variables
    lastRainCheck = millis();
    byte currentWindSpeed = getWindSpeed();
    int  currentWindDir   = getWindDir();
    for (windSpdSampleIdx=0; windSpdSampleIdx<WIND_SPD_SAMPLES; windSpdSampleIdx++) {
        windSpdAvg[windSpdSampleIdx] = currentWindSpeed;
    }
    for (windDirSampleIdx=0; windDirSampleIdx<WIND_DIR_SAMPLES; windDirSampleIdx++) {
        windDirAvg[windDirSampleIdx] = currentWindDir;
    }
    windSpdSampleIdx = 0;
    windDirSampleIdx = 0;
        
    #ifdef DEBUG
        Serial.print(F(SKETCH_NAME));
        Serial.println(F(" started."));
    #endif
}

/*****************************************************************************************
 * loop()
 * 
 * Main loop
 *****************************************************************************************/
void loop () {
    DateTime now = rtc.now();
    byte currentWindSpeed;
    int  currentWindDir;

    // Update indexes
    windSpdSampleIdx++;
    if (windSpdSampleIdx == WIND_SPD_SAMPLES) {
        windSpdSampleIdx = 0;
    }
    windDirSampleIdx++;
    if (windDirSampleIdx == WIND_DIR_SAMPLES) {
        windDirSampleIdx = 0;
    }

    // Clear rain total at midnight
    if (now.hour() == 0 and now.minute() == 0 and now.second() == 0) {
        rainClicksTotal = 0;
        #ifdef DEBUG
          Serial.println(F("Midnight. Clear daily rain data"));
        #endif
    }

    // Clear wind gust for minute
    if (now.second() == 0) {
        windGustAvg_30m[now.minute() % 30];
    }

    // Store some data every 10 seconds
    if (now.second() % 10 == 0) {
        // Calculate wind speed
        currentWindSpeed = getWindSpeed();
        currentWindDir   = getWindDir();
        windSpdAvg[windSpdSampleIdx] = currentWindSpeed;
        windDirAvg[windDirSampleIdx] = currentWindDir;

        // Check if this is a gust for the minute
        if(windSpdAvg[windSpdSampleIdx] > windGustAvg_30m[now.minute() % 30]) {
            windGustAvg_30m[now.minute() % 30] = windSpdAvg[windSpdSampleIdx];
        }

        #ifdef DEBUG
          Serial.print(now.hour());
          Serial.print(":");
          Serial.print(now.minute());
          Serial.print(":");
          Serial.print(now.second());
          Serial.print(" ");
          Serial.print(F("Wind: "));
          Serial.print(currentWindSpeed);
          Serial.print(F("km/h "));
          Serial.print(deg2compass(currentWindDir));
          Serial.println();
        #endif
    }

    // Send data every 5 minutes
    //if (now.second() == 0) {
    //if (now.minute() % 2 == 0 & now.second() == 0) {
    if (now.minute() % 5 == 0 & now.second() == 0) {
        // Grab current data
        float    lightLevel = getLightLevel();
        byte     windSpeed  = getWindSpeedAvg();
        uint16_t windDir    = getWindDirAvg();
        String   windDirTxt = deg2compass(windDir);
        byte     windGust   = getWindGust();
        float    rainRate   = getRainRate();
        float    rainTotal  = getRainTotal();

        // Display data
        #ifdef DEBUG
            Serial.print(F("Weather: light="));
            Serial.print(lightLevel);
            Serial.print(F("lx, "));
            Serial.print(F("windSpeed="));
            Serial.print(windSpeed, 1);
            Serial.print(F("km/h, "));
            Serial.print(F("windDir="));
            Serial.print(windDir);
            Serial.print(F(" "));
            Serial.print(windDirTxt);
            Serial.print(F(", windGust="));
            Serial.print(windGust, 1);
            Serial.print(F("km/h, "));
            Serial.print(F("rainRate="));
            Serial.print(rainRate, 2);
            Serial.print(F("mm/h, "));
            Serial.print(rainTotal, 2);
            Serial.println(F("mm"));
        #endif        

        // Send data to MQTT 
        send(msg.setSensor(CHILD_ID_RAIN).setType(V_RAIN).set(rainTotal,1));
        send(msg.setSensor(CHILD_ID_RAINRATE).setType(V_RAINRATE).set(rainRate,1));
        send(msg.setSensor(CHILD_ID_LIGHT).setType(V_LEVEL).set(lightLevel,0));
        send(msg.setSensor(CHILD_ID_WIND_SPEED).setType(V_WIND).set(windSpeed));
        send(msg.setSensor(CHILD_ID_WIND_GUST).setType(V_GUST).set(windGust));
        send(msg.setSensor(CHILD_ID_WIND_DIR).setType(V_DIRECTION).set(windDir));
        send(msg.setSensor(CHILD_ID_WIND_DIR_TXT).setType(V_TEXT).set(windDirTxt.c_str()));
    }

    // Delay to next second
    while(currentSecond == rtc.now().second()) {
        delay(100);
    }
    currentSecond = rtc.now().second();
}

/*****************************************************************************************
 * getWindSpeed()
 * 
 * Measure wind speed
 *****************************************************************************************/
int getWindSpeed() {
    float deltaTime = millis() - lastWindCheck; //750ms
    
    float tmpWindSpeed = (float)windClicks / (deltaTime / 1000); // 3 / (750 / 1000) = 3 / 0.750 = 4
    
    windClicks    = 0;
    lastWindCheck = millis();
    
    tmpWindSpeed *= WIND_SPEED_CLICK; //4 * 2.4 = 9.6km/h
    
    return(tmpWindSpeed);
}

/*****************************************************************************************
 * getWindDir()
 * 
 * Read the wind direction sensor, return heading in degrees
 *****************************************************************************************/
int getWindDir() {
    
    uint16_t adc = analogRead(WDIR_PIN);
        
    // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
    // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
    // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
        
    if (adc < 380) return (113); // ESE
    if (adc < 394) return (68);  // ENE
    if (adc < 414) return (90);  // E
    if (adc < 456) return (158); // SSE
    if (adc < 509) return (135); // SE
    if (adc < 552) return (203); // SSW
    if (adc < 616) return (180); // S
    if (adc < 681) return (23);  // NNE
    if (adc < 746) return (45);  // NE
    if (adc < 802) return (248); // WSW
    if (adc < 833) return (225); // SW
    if (adc < 878) return (338); // NNW
    if (adc < 914) return (0);   // N
    if (adc < 941) return (293); // WNW
    if (adc < 971) return (315); // NW
    if (adc < 999) return (270); // W
    return (-1); // error, disconnected?
}

/*****************************************************************************************
 * getLightLevel()
 * 
 * Measure light level with autoadjustment
 *****************************************************************************************/
float getLightLevel() {
    float tmpLux = lightSensor.readLightLevel(true);
    if (tmpLux < 0) {
        Serial.println(F("Error condition detected"));
        return (0);
    } else if (tmpLux > 40000.0) {
        lightSensor.setMTreg(32);
        Serial.println(F("Setting MTReg to low value for high light environment"));
    } else if (tmpLux > 10) {
        lightSensor.setMTreg(69);
        Serial.println(F("Setting MTReg to default value for normal light environment"));
    } else if (tmpLux < 10) {
        lightSensor.setMTreg(138);
        Serial.println(F("Setting MTReg to high value for low light environment"));
    }

    return (lightSensor.readLightLevel(true));
}

/*****************************************************************************************
 * getwindSpeedAvg()
 * 
 * Calculate wind speed 2 minutes average
 *****************************************************************************************/
byte getWindSpeedAvg() {
    int tmpWindSpeedAvg = 0;
    for(int i = 0 ; i < WIND_SPD_SAMPLES ; i++) {
        tmpWindSpeedAvg += windSpdAvg[i];
    }
    return(tmpWindSpeedAvg / WIND_SPD_SAMPLES);
}

/*****************************************************************************************
 * getWindDirAvg()
 * 
 * Calc winddir_avg2m, Wind Direction using statistical function "mode"
 * Note: Mitsuta method not working well, so statistical mode is more better.
 *****************************************************************************************/
uint16_t getWindDirAvg() {
    byte currentcount ;
    byte mostcount    = 1;
    uint16_t current  = 0;
    uint16_t most     = windDirAvg[0];
    
    for(byte pos = 0; pos < WIND_DIR_SAMPLES; pos++) {
        current = windDirAvg[pos];
        currentcount = 1;
        for (byte inner = pos + 1; inner < WIND_DIR_SAMPLES; inner++) {
            if(windDirAvg[inner] == current) {
                currentcount++;
            }
        }
        if (currentcount > mostcount) {
            most = current;
            mostcount = currentcount;
        }
    }
    return most;
}

/*****************************************************************************************
 * getWindGust()
 * 
 * Measure light level with autoadjustment
 *****************************************************************************************/
// Calc windgustkmh_30m
// Find the largest windgust in the last 30 minutes
byte getWindGust() {
    byte tmpWindGust = 0;
    for (byte i = 0; i < 30 ; i++) {
        if(windGustAvg_30m[i] > tmpWindGust) {
            tmpWindGust = windGustAvg_30m[i];
        }
    }
    
    return(tmpWindGust);
}

/*****************************************************************************************
 * getRainRate()
 * 
 * Calculate rain rate
 *****************************************************************************************/
float getRainRate() {
    unsigned long deltaTime = millis() - lastRainCheck;
    unsigned int  tmpRainRate;

    rainClicksTotal += rainClicks;

    tmpRainRate = rainClicks * 3600000L / deltaTime;
    tmpRainRate *= RAIN_BUCKET_SIZE;

    lastRainCheck = millis();
    rainClicks    = 0;

    // Store rain data
    storeRain();

    return (tmpRainRate);
}

/*****************************************************************************************
 * getRainTotal()
 * 
 * Calculate rain total
 *****************************************************************************************/
float getRainTotal() {
    return (rainClicksTotal * RAIN_BUCKET_SIZE);  
}

/*****************************************************************************************
 * getRainTotal()
 * 
 * Store rain data to DS1307 NVRAM
 *****************************************************************************************/
void storeRain() {
    rtc.writenvram(0, 0xEE); // control byte
    rtc.writenvram(1, rtc.now().day());  
    rtc.writenvram(2, highByte(rainClicksTotal));  
    rtc.writenvram(3, lowByte(rainClicksTotal));  
}

/*****************************************************************************************
 * deg2compas()
 * 
 * Convert degrees to compas dir
 *****************************************************************************************/
  String deg2compass(int dir) {
    const static char* const compassArr[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };

    if (dir < 0) return("ERR"); // Error
    int dirIdx( static_cast<int>(((dir / 360.0) * 16) + 0.5)  % 16);
    return(compassArr[dirIdx]);    
}
