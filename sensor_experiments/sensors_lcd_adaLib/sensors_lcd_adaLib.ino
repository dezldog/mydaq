//dezldog 31JAN16
// Getting data from DHT humidity/temperature, a potentiometer, a CDS cell, and a GPS
// output them to serial and LCD
// code borrowed from many.

#include "DHT.h"
#include "SoftwareSerial.h"
#include "Adafruit_GPS.h"
#include "Adafruit_LiquidCrystal.h"

#define DHTPIN0 2     // what pins DHTs are connected to
#define DHTPIN1 4
#define DHTTYPE DHT11   // We are using DHT11s
#define PROBE_0 "Outside"  // Let's name the DHT probes
#define PROBE_1 "Inside"

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false


//Sample time in seconds
const int sampleSeconds = 30;
const int sampleMultiplier = 1000; //because milliseconds
int sampleDelay = (sampleSeconds * sampleMultiplier);

// Instansiate LCD object
Adafruit_LiquidCrystal lcd(0);

// Instansiate DHT sensors
DHT dht0(DHTPIN0, DHTTYPE);
DHT dht1(DHTPIN1, DHTTYPE);

//define photocell input/variable
int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider

//define potentiometer input/variable
int potPin = 1;     // the potentiometer is connected to a1
float potReading;     // the analog reading from the potentiometer

//calibrate a/d for potentiometer
float yIntercept = 646.00;
float xIntercept = 3.3;
float slope = ((0 - xIntercept) / (yIntercept - 0));

//Set up softwareserial
int rxPin = 8;
int txPin = 7;
int GPSBaud = 9600;
SoftwareSerial gpsSerial(rxPin, txPin); // create gps sensor connection

//Instansiate GPS
Adafruit_GPS GPS(&gpsSerial); // create gps object
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
uint32_t timer = millis();


void setup()
{
  //start gps
  Serial.begin(115200);

  gpsSerial.begin (GPSBaud);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true); GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  //start dht
  dht0.begin();
  dht1.begin();
  lcd.begin(16, 4);
  // Print a message to the LCD
  lcd.print("LCD Setup Complete");
  Serial.println("Serial Setup Complete");
  delay(3000);
  lcd.clear();
}

void loop()
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.println();
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 6);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      displayInfo();
      displayLcd();
    }
  }
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void displayInfo()
{
  //Process DHT data
  float h0 = dht0.readHumidity();
  float t0 = dht0.readTemperature();
  float f0 = dht0.readTemperature(true);

  float h1 = dht1.readHumidity();
  float t1 = dht1.readTemperature();
  float f1 = dht1.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h0) || isnan(t0) || isnan(f0)) {
    Serial.println("Failed to read from DHT0 sensor!");
    return;
  }
  if (isnan(h1) || isnan(t1) || isnan(f1)) {
    Serial.println("Failed to read from DHT1 sensor!");
    return;
  }

  float hif0 = dht0.computeHeatIndex(f0, h0);    // Compute heat index in Fahrenheit (the default)
  float hic0 = dht0.computeHeatIndex(t0, h0, false);  // Compute heat index in Celsius (isFahreheit = false)

  float hif1 = dht1.computeHeatIndex(f1, h1);
  float hic1 = dht1.computeHeatIndex(t1, h1, false);

  //Process CDS cell reading
  photocellReading = analogRead(photocellPin);
  potReading = analogRead(potPin);

  //Calculate Volts from potentiometer a/d value
  float volts = 3.3 + (potReading * slope);

  //Print the other data
  Serial.print("CDS cell value = "); Serial.println(photocellReading);
  Serial.print("Potentiometer reading = "); Serial.print( volts); Serial.println(" V");
  Serial.print(PROBE_0" Humidity: ");
  Serial.print(h0);
  Serial.print(" %");
  Serial.println();
  Serial.print(PROBE_0" Temp: ");
  Serial.print(t0);
  Serial.print(" C");
  Serial.println();
  Serial.print(PROBE_0" Temp: ");
  Serial.print(f0);
  Serial.print(" F");
  Serial.println();
  Serial.print(PROBE_0" Heat Index: ");
  Serial.print(hic0);
  Serial.print(" C");
  Serial.println();
  Serial.print(PROBE_0" Heat Index: ");
  Serial.print(hif0);
  Serial.print(" F");
  Serial.println();
  Serial.print(PROBE_1" Humidity: ");
  Serial.print(h1);
  Serial.print(" %");
  Serial.println();
  Serial.print(PROBE_1" Temp: ");
  Serial.print(t1);
  Serial.print(" C");
  Serial.println();
  Serial.print(PROBE_1" Temp: ");
  Serial.print(f1);
  Serial.print(" F");
  Serial.println();
  Serial.print(PROBE_1" Heat Index: ");
  Serial.print(hic1);
  Serial.print(" C");
  Serial.println();
  Serial.print(PROBE_1" Heat Index: ");
  Serial.print(hif1);
  Serial.print(" F");
  Serial.println();
  Serial.println();
}

void displayLcd()
{
  int minutes = 0;
  int seconds = 0;
  int hours = 0;

  // Send data to the LCD too
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  hours = GPS.hour;
  if (hours < 10)
  {
    lcd.print("0");
    lcd.print(hours);
  }
  else
  {
    lcd.print(hours);
  }
  lcd.print(':');
  minutes = GPS.minute;
  if (minutes < 10)
  {
    lcd.print("0");
    lcd.print(minutes);
  }
  else
  {
    lcd.print(GPS.minute);
  }
  lcd.print(':');
  seconds = GPS.seconds;
  if (seconds < 10)
  {
    lcd.print("0");
    lcd.print(seconds);
  }
  else
  {
    lcd.print(seconds);
  }
  lcd.print(" GMT");
  lcd.setCursor(0, 1);
  lcd.print("Date: ");
  lcd.print(GPS.month);
  lcd.print('/');
  lcd.print(GPS.day);
  lcd.print("/20");
  lcd.print(GPS.year);
  lcd.setCursor(0, 2);
  lcd.print("Lat:  "); lcd.print(GPS.latitudeDegrees, 6);
  lcd.setCursor(0, 3);
  lcd.print("Lon: "); lcd.print(GPS.longitudeDegrees, 6);

}

