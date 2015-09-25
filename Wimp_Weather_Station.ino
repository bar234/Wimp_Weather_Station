/*
 Weather Station using the Electric Imp
 By: Nathan Seidle
 SparkFun Electronics
 Date: October 4th, 2013
 License: This code is public domain but you buy me a beer if you use this and
 we meet someday (Beerware license).

 Much of this is based on Mike Grusin's USB Weather Board code.

 This code reads all the various sensors (wind speed, direction, rain gauge,
 humidty, pressure, light, batt_lvl) and sends it to the imp, which then
 forwards that data to an Imp Agent on the cloud that does some processing then
 bounces the weather data to Wunderground.

 The Imp Shield has Card Detect tied to pin A0. We use A0 for wind direction.
 You will need to cut the trace on the Imp shield.

 Current:
 130 for 2 seconds while transmitting
 ~30mA during sleep

 Todo:
 Reset after 45 days to avoid millis roll over problems

 What was the wind direction and speed gust for the last 10 minutes?
 Is the 3.3V pin tied on the weather shield or elsewhere?
 */

//#define ENABLE_LIGHTNING
#include <Wire.h> // I2C needed for sensors
#include "SparkFunMPL3115A2.h" // Pressure sensor
#include "SparkFunHTU21D.h" // Humidity sensor

#ifdef ENABLE_LIGHTNING
#include "AS3935.h" //Lighting dtector
#include <SPI.h> //Needed for lighting sensor

// Not really an interrupt pin, we will catch it in software
const byte LIGHTNING_IRQ = 4;   

// SS for AS3935
const byte slaveSelectPin = 10; 

byte SPItransfer(byte sendByte);
AS3935 AS3935(SPItransfer, slaveSelectPin, LIGHTNING_IRQ);
#endif

// SoftwareSerial imp(8, 9); // RX, TX into Imp pin 7

MPL3115A2 myPressure; // Create an instance of the pressure sensor
HTU21D myHumidity; // Create an instance of the humidity sensor

// Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;

// analog I/O pins
const byte WDIR = A0;
const byte LIGHT = A1;
const byte REFERENCE_3V3 = A3;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// The millis counter to see when a second rolls by
long lastSecond;

// Used to reset variables after 24 hours.  Imp should tell us when it's
// midnight, this is backup.
unsigned int minutesSinceLastReset;

// When it hits 60, increase the current minute
byte seconds;

// Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte seconds_2m;

// Keeps track of where we are in various arrays of data
byte minutes;

// Keeps track of where we are in wind gust/dir over last 10 minutes array of
// data
byte minutes_10m;

// Time the wind was last checked
long lastWindCheck = 0;

// Time the wind speed IRQ was last triggered
volatile long lastWindIRQ = 0;

// Number of wind speed clicks since last report
volatile byte windClicks = 0;

// Distance of lightning strike
#ifdef ENABLE_LIGHTNING
byte lightning_distance = 0;
#endif

// We need to keep track of the following variables:
// Wind speed/dir each update (no storage)
// Wind gust/dir over the day (no storage)
// Wind speed/dir, avg over 2 minutes (store 1 per second)
// Wind gust/dir over last 10 minutes (store 1 per minute)
// Rain over the past hour (store 1 per minute)
// Total rain over date (store one per day)

// 120 bytes to keep track of 2 minute average
byte windspdavg[120];

// Number of seconds of wind direction recordings to keep track of. 120 samples
// = 2 minutes of wind direction recording at 1 sample/sec
#define WIND_DIR_AVG_SIZE 120
int winddiravg[WIND_DIR_AVG_SIZE];

// 10 floats to keep track of largest gust in the last 10 minutes
float windgust_10m[10];

// 10 ints to keep track of largest gust direction in the last 10 minutes
int windgustdirection_10m[10];

// 60 floating numbers to keep track of 60 minutes of rain
volatile float rainHour[60];

// These are all the weather values that wunderground expects:
int winddir;                // [0-360 instantaneous wind direction]
float windspeedmph;         // [mph instantaneous wind speed]
float windgustmph;          // [mph current wind gust, using software specific
                            // time period]
int windgustdir;            // [0-360 using software specific time period]
float windspdmph_avg2m;     // [mph 2 minute average wind speed mph]
int winddir_avg2m;          // [0-360 2 minute average wind direction]
float windgustmph_10m;      // [mph past 10 minutes wind gust mph ]
int windgustdir_10m;        // [0-360 past 10 minutes wind gust direction]
float humidity;             // [%]
float tempf;                // [temperature F]
float rainin;               // [rain inches over the past hour)] -- the
                            // accumulated rainfall in the past 60 min
volatile float dailyrainin; // [rain inches so far today in local time]
float pressure;             // It's hard to calculate baromin locally, do this
                            // in the agent

// These are not wunderground values, they are just for us
float batt_lvl = 0.0;
float light_lvl = 0.0;

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Interrupt routines (these are called by the hardware interrupts, not by the
// main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur Activated by the magnet and reed
// switch in the rain gauge, attached to input D2
{
    // grab current time
    raintime = millis();

    // calculate interval between this and last event
    raininterval = raintime - rainlast;

    // ignore switch-bounce glitches less than 10mS after initial edge
    if (raininterval > 10)
    {
        dailyrainin += 0.011;       // Each dump is 0.011" of water
        rainHour[minutes] += 0.011; // Increase this minute's amount of rain

        rainlast = raintime;        // set up for next event
    }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached
// to input D3
{
    // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after
    // the read switch closes
    if (millis() - lastWindIRQ > 10)
    {
        lastWindIRQ = millis(); // Grab the current time
        windClicks++;           // There is 1.492MPH for each click per second.
    }
}

// When the imp tells us it's midnight, reset the total amount of rain and
// gusts
void midnightReset()
{
    // Reset daily amount of rain
    dailyrainin = 0;

    // Zero out the windgust for the day
    windgustmph = 0;

    // Zero out the gust direction for the day
    windgustdir = 0;

    // Reset minute tracker
    minutes = 0;
    seconds = 0;

    // Reset variable used to track minutes
    lastSecond = millis();

    // Zero out the backup midnight reset variable
    minutesSinceLastReset = 0;
}

// Takes an average of readings on a given pin.  Returns the average
int averageAnalogRead(int pinToRead)
{
    byte numberOfReadings = 8;
    unsigned int runningValue = 0;

    for(int x = 0 ; x < numberOfReadings ; x++)
    {
        runningValue += analogRead(pinToRead);
    }
    runningValue /= numberOfReadings;

    return(runningValue);
}


// Returns the voltage of the light sensor based on the 3.3V rail
// This allows us to ignore what VCC might be (an Arduino plugged into USB has
// VCC of 4.5 to 5.2V)
float get_light_level()
{
    float operatingVoltage = averageAnalogRead(REFERENCE_3V3);

    float lightSensor = averageAnalogRead(LIGHT);

    operatingVoltage = 3.3 / operatingVoltage;  // The reference voltage is 3.3V

    lightSensor *= operatingVoltage;

    return(lightSensor);
}

// Return the Vcc voltage level based on the internal ADC reading of the 1.1v
// reference. See https://code.google.com/p/tinkerit/wiki/SecretVoltmeter for
// more details
float get_battery_level()
{
    long result;

    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

    delay(2); // Wait for Vref to settle

    ADCSRA |= _BV(ADSC); // Convert

    while (bit_is_set(ADCSRA,ADSC));

    result = ADCL;
    result |= ADCH<<8;
    result = 1126400L / result; // Back-calculate AVcc in mV

    return result / 1000.0;
}

// Calculates each of the variables that wunderground is expecting
void calcWeather()
{
    // current winddir, current windspeed, windgustmph, and windgustdir are
    // calculated every 100ms throughout the day

    // Calc windspdmph_avg2m, which is the average windspeed over the last 2
    // minutes
    float temp = 0;
    for(int i = 0 ; i < 120 ; i++)
    {
        temp += windspdavg[i];
    }

    temp /= 120.0;

    windspdmph_avg2m = temp;

    // Calc winddir_avg2m, which is the average wind direction over the last 2
    // minutes
    // You can't just take the average. Google "mean of circular quantities"
    // for more info. We will use the Mitsuta method because it doesn't require
    // trig functions. And because it sounds cool.
    // Ref: http://abelian.org/vlf/bearings.html
    // Ref: http://stackoverflow.com/questions/1813483/averaging-angles-again
    long sum = winddiravg[0];
    int D = winddiravg[0];
    for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
    {
        int delta = winddiravg[i] - D;

        if(delta < -180)
        {
            D += delta + 360;
        }
        else if(delta > 180)
        {
            D += delta - 360;
        }
        else
        {
            D += delta;
        }

        sum += D;
    }
    winddir_avg2m = sum / WIND_DIR_AVG_SIZE;

    if(winddir_avg2m >= 360)
    {
        winddir_avg2m -= 360;
    }

    if(winddir_avg2m < 0)
    {
        winddir_avg2m += 360;
    }

    // Find the largest windgust in the last 10 minutes
    windgustmph_10m = 0;
    windgustdir_10m = 0;

    // Step through the 10 minutes
    for(int i = 0; i < 10 ; i++)
    {
        if(windgust_10m[i] > windgustmph_10m)
        {
            windgustmph_10m = windgust_10m[i];
            windgustdir_10m = windgustdirection_10m[i];
        }
    }

    // Calc humidity
    humidity = myHumidity.readHumidity();

    // Calc tempf from pressure sensor
    tempf = myPressure.readTempF();

    // Total rainfall for the day is calculated within the interrupt
    // Calculate amount of rainfall for the last 60 minutes
    rainin = 0;
    for(int i = 0 ; i < 60 ; i++)
    {
        rainin += rainHour[i];
    }

    // Calc pressure
    pressure = myPressure.readPressure();

    // Calc light level
    light_lvl = get_light_level();

    // Calc battery level
    batt_lvl = get_battery_level();
}

// Returns the instataneous wind speed
float get_wind_speed()
{
    // Calculate time since the last wind speed check
    float deltaTime = millis() - lastWindCheck;

    // Covert to seconds
    deltaTime /= 1000.0;

    // Calculate number of wind clicks per second
    float windSpeed = (float)windClicks / deltaTime;

    // Reset and start watching for new wind clicks
    windClicks = 0;
    lastWindCheck = millis();

    // Convert wind clicks per second to miles per hour
    windSpeed *= 1.492;

    return(windSpeed);
}

// read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
    unsigned int adc;

    // get the current reading from the sensor
    adc = averageAnalogRead(WDIR);

    // The following table is ADC readings for the wind direction sensor
    // output, sorted from low to high.  Each threshold is the midpoint between
    // adjacent headings. The output is degrees for that ADC reading.  Note
    // that these are not in compass degree order! See Weather Meters datasheet
    // for more information.

    if (adc < 380) return (113);
    if (adc < 393) return (68);
    if (adc < 414) return (90);
    if (adc < 456) return (158);
    if (adc < 508) return (135);
    if (adc < 551) return (203);
    if (adc < 615) return (180);
    if (adc < 680) return (23);
    if (adc < 746) return (45);
    if (adc < 801) return (248);
    if (adc < 833) return (225);
    if (adc < 878) return (338);
    if (adc < 913) return (0);
    if (adc < 940) return (293);
    if (adc < 967) return (315);
    if (adc < 990) return (270);
    return (-1); // error, disconnected?
}

// The following is for the AS3935 lightning sensor
#ifdef ENABLE_LIGHTNING
byte readLightning(void)
{
    byte distance = 0;

    // Check to see if we have lightning!
    if(digitalRead(LIGHTNING_IRQ) == HIGH)
    {
        // first step is to find out what caused interrupt
        // as soon as we read interrupt cause register, irq pin goes low
        int irqSource = AS3935.interruptSource();

        // returned value is bitmap field, bit 0 - noise level too high,
        // bit 2 - disturber detected, and finally bit 3 - lightning!
        if (irqSource & 0b0100)
        {
            distance = 64;
        }

        if (irqSource & 0b1000)
        {
            // Function returns approximate lightning strike distance in
            // kilometers, where value 1 represents storm in detector's near
            // victinity, and 63 - very distant, out of range strike.
            distance = AS3935.lightningDistanceKm();

            // The AS3935 remembers the nearest strike distance. For example
            // 15km away then 10, then overhead. All following distances (ie.
            // 10, 20, 30) will output as overhead. Resetting the chip erases
            // this.
            lightning_init();
        }
    }

    return(distance);
}

// Initialize the lightning sensor
void startLightning(void)
{
    // set the slaveSelectPin as an output:
    pinMode(slaveSelectPin, OUTPUT);

    // Set IRQ pin as input
    pinMode(LIGHTNING_IRQ, INPUT_PULLUP);

    // Start SPI
    SPI.begin();

    // Chip uses SPI MODE1
    SPI.setDataMode(SPI_MODE1);

    // Uno 16MHz / 16 = 1MHz
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // and chip is MSB first
    SPI.setBitOrder(MSBFIRST);

    // Setup the values for the sensor
    lightning_init(); 

    Serial.println("Lightning sensor online");
}

void lightning_init()
{
    // Reset all internal register values to defaults
    AS3935.reset(); 

    // If lightning detector can not tune tank circuit to required tolerance,
    // calibration function will return false
    if(!AS3935.calibrate())
    {
        Serial.println("Tuning out of range, check your wiring and sensor");
    }

    // The weather station is outdoors
    AS3935.setOutdoors(); 

    // We want to know if a man-made event happens
    AS3935.enableDisturbers(); 

    // See table 16 of the AS3935 datasheet. 4-6 works. This was found through
    // experimentation.
    AS3935.setNoiseFloor(3); 

    // printAS3935Registers();
}

/*void printAS3935Registers()
{
  int noiseFloor = AS3935.getNoiseFloor();
  int spikeRejection = AS3935.getSpikeRejection();
  int watchdogThreshold = AS3935.getWatchdogThreshold();
  Serial.print("Noise floor is: ");
  Serial.println(noiseFloor, DEC);
  Serial.print("Spike rejection is: ");
  Serial.println(spikeRejection, DEC);
  Serial.print("Watchdog threshold is: ");
  Serial.println(watchdogThreshold, DEC);
}*/

byte SPItransfer(byte sendByte)
{
    return SPI.transfer(sendByte);
}
#endif

// Reports the weather string to the Imp
void reportWeather()
{
    calcWeather(); // Go calc all the various sensors

    Serial.print("$,winddir=");
    Serial.print(winddir);
    Serial.print(",windspeedmph=");
    Serial.print(windspeedmph, 1);
    Serial.print(",windgustmph=");
    Serial.print(windgustmph, 1);
    Serial.print(",windgustdir=");
    Serial.print(windgustdir);
    Serial.print(",windspdmph_avg2m=");
    Serial.print(windspdmph_avg2m, 1);
    Serial.print(",winddir_avg2m=");
    Serial.print(winddir_avg2m);
    Serial.print(",windgustmph_10m=");
    Serial.print(windgustmph_10m, 1);
    Serial.print(",windgustdir_10m=");
    Serial.print(windgustdir_10m);
    Serial.print(",humidity=");
    Serial.print(humidity, 1);
    Serial.print(",tempf=");
    Serial.print(tempf, 1);
    Serial.print(",rainin=");
    Serial.print(rainin, 2);
    Serial.print(",dailyrainin=");
    Serial.print(dailyrainin, 2);
    Serial.print(",");  // Don't print pressure= because the agent will be doing
                        // calcs on the number
    Serial.print(pressure, 2);
    Serial.print(",batt_lvl=");
    Serial.print(batt_lvl, 2);
    Serial.print(",light_lvl=");
    Serial.print(light_lvl, 2);

#ifdef LIGHTNING_ENABLED
    Serial.print(",lightning_distance=");
    Serial.print(lightning_distance);
#endif

    Serial.print(",");
    Serial.println("#,");
}

void setup()
{
    Serial.begin(9600);

    pinMode(WSPEED, INPUT_PULLUP);  // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP);    // input from wind meters rain gauge sensor

    pinMode(WDIR, INPUT);
    pinMode(LIGHT, INPUT);
    pinMode(REFERENCE_3V3, INPUT);

    pinMode(STAT1, OUTPUT);

    // Reset rain totals
    midnightReset();

#ifdef ENABLE_LIGHTNING
    // Init the lighting sensor
    startLightning();
#endif

    // Configure the pressure sensor

    // Get sensor online
    myPressure.begin();

    // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setModeBarometer();

    // Set Oversample to the recommended 128
    myPressure.setOversampleRate(128);

    // Enable all three pressure and temp event flags
    myPressure.enableEventFlags();

    // Go to active mode and start measuring!
    myPressure.setModeActive();

    // Configure the humidity sensor
    myHumidity.begin();

    // Initialize second counters
    seconds = 0;
    lastSecond = millis();

    // attach external interrupt pins to IRQ functions
    attachInterrupt(0, rainIRQ, FALLING);
    attachInterrupt(1, wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();

    Serial.println("Wimp Weather Station online!");
    reportWeather();
}

void loop()
{
    // Keep track of which minute it is
    if(millis() - lastSecond >= 1000)
    {
        lastSecond += 1000;

        // Take a speed and direction reading every second for 2 minute average
        if(++seconds_2m > 119)
        {
            seconds_2m = 0;
        }

        // Calc the wind speed and direction every second for 120 second to get
        // 2 minute average
        windspeedmph = get_wind_speed();
        winddir = get_wind_direction();
        windspdavg[seconds_2m] = (int)windspeedmph;
        winddiravg[seconds_2m] = winddir;

        // Check to see if this is a gust for the minute
        if(windspeedmph > windgust_10m[minutes_10m])
        {
            windgust_10m[minutes_10m] = windspeedmph;
            windgustdirection_10m[minutes_10m] = winddir;
        }

        // Check to see if this is a gust for the day. Resets at midnight each
        // night
        if(windspeedmph > windgustmph)
        {
            windgustmph = windspeedmph;
            windgustdir = winddir;
        }

        // Blink stat LED briefly to show we are alive
        digitalWrite(STAT1, HIGH);
        delay(25);
        digitalWrite(STAT1, LOW);

        // If we roll over 60 seconds then update the arrays for rain and
        // windgust
        if(++seconds > 59)
        {
            seconds = 0;

            if(++minutes > 59)
            {
                minutes = 0;
            }

            if(++minutes_10m > 9)
            {
                minutes_10m = 0;
            }

            // Zero out this minute's rainfall amount
            rainHour[minutes] = 0;

            // Zero out this minute's gust
            windgust_10m[minutes_10m] = 0;

            // It's been another minute since last night's midnight reset
            minutesSinceLastReset++;
        }
    }

    //Check to see if there's been lighting
#ifdef ENABLE_LIGHTNING
    if(digitalRead(LIGHTNING_IRQ) == HIGH)
    {
        // We've got something!
        lightning_distance = readLightning();
    }
#endif

    // Wait for the imp to ping us with the ! character
    if(Serial.available())
    {
        byte incoming = Serial.read();
        if(incoming == '!')
        {
            // Send all the current readings out the imp and to its agent for
            // posting to wunderground.
            reportWeather();

#ifdef ENABLE_LIGHTNING
            // Give imp time to transmit then read any erroneous lightning strike
            delay(1000);

            // Clear any readings
            readLightning();
#endif
        }

        // Special character from Imp indicating midnight local time
        else if(incoming == '@')
        {
            // Reset a bunch of variables like rain and daily total rain
            midnightReset();
        }
    }

    // If we go for more than 24 hours without a midnight reset then force a
    // reset 24 hours * 60 mins/hr = 1,440 minutes + 10 extra minutes. We hope
    // that Imp is doing it.
    if(minutesSinceLastReset > (1440 + 10))
    {
        // Reset a bunch of variables like rain and daily total rain
        midnightReset();
    }

    delay(100); // Update every 100ms. No need to go any faster.
}


