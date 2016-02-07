//dezldog 31JAN16
// Getting data from DHT humidity/temperature, a potentiometer, a CDS cell, and a GPS
// output them to serial and LCD
// code borrowed from many.

#include "DHT.h"
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

#define DHTPIN0 2     // what pins DHTs are connected to
#define DHTPIN1 4
#define DHTTYPE DHT11   // We are using DHT11s
#define PROBE_0 "Outside"  // Let's name the DHT probes
#define PROBE_1 "Inside"

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
TinyGPSPlus gps; // create gps object

void setup()
{
  //start gps
  Serial.begin(115200);
  gpsSerial.begin (GPSBaud);
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
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
    {
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        return;
      }
      displayInfo();
      displayLcd();
      delay(sampleDelay); //Wait between samples
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

  //Get and process GPS data
  gps.encode(gpsSerial.read());
  Serial.print(F("Date: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
  Serial.print(F("Time: "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.println();
  }
  else
  {
    Serial.print(F("INVALID"));
    Serial.println();
  }
  Serial.print("CDS cell value = "); Serial.println(photocellReading);
  Serial.print("Potentiometer reading = "); Serial.print( volts); Serial.println(" V");
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
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
  // Send data to the LCD too
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.setCursor(0, 1);
  lcd.print("Date: ");
  lcd.setCursor(0, 2);
  lcd.print("Lat: ");
  lcd.setCursor(0, 3);
  lcd.print("Lon: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
    {
      lcd.setCursor(6, 0);
      lcd.print("0");
      lcd.setCursor(7, 0);
      lcd.print(gps.time.hour());
    }
    else
    {
      lcd.setCursor(6, 0);
      lcd.print(gps.time.hour());
    }
    lcd.print(":");
    if (gps.time.minute() < 10)
    { lcd.setCursor(9, 0);
      lcd.print("0");
      lcd.setCursor(10, 0);
      lcd.print(gps.time.minute());
    }
    else
    {
      lcd.setCursor(9, 0);
      lcd.print(gps.time.minute());
    }
    lcd.setCursor(11, 0);
    lcd.print(" GMT");
    lcd.setCursor(6, 1);
  }
  else
  {
    lcd.setCursor(6, 0);
    lcd.print("INVALID");
  }
  if (gps.date.isValid())
  {
    lcd.print(gps.date.month());
    lcd.print("/");
    lcd.print(gps.date.day());
    lcd.print("/");
    lcd.print(gps.date.year());
  }
  else
  {
    lcd.setCursor(6, 1);
    lcd.print("INVALID");
  }
  lcd.setCursor(5, 2);
  lcd.print(gps.location.lat(), 6);
  lcd.setCursor(5, 3);
  lcd.print(gps.location.lng(), 6);
}
