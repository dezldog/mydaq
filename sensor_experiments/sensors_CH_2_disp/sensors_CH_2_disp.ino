//dezldog 31JAN16
// Getting data from DHT humidity/temperature, a potentiometer, and a CDS cell
// code borrowed from many.
#include "Wire.h"
#include "DHT.h"
#include "Adafruit_LEDBackpack.h"

#define DHTPIN0 2     // what pins DHTs are connected to
#define DHTPIN1 4
#define DHTTYPE DHT11   // We are using DHT11s
#define PROBE_0 "Outside"  // Let's name the DHT probes
#define PROBE_1 "Inside"

//Sample time in seconds
const int sampleSeconds = 30;
const int sampleMultiplier = 100;
int sampleDelay = (sampleSeconds * sampleMultiplier);

// Initialize DHT sensors
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

//Set up 7-seg displays
Adafruit_7segment matrix0 = Adafruit_7segment();
Adafruit_7segment matrix1 = Adafruit_7segment();


void setup()
{
  //start gps
  Serial.begin(115200);
  dht0.begin();
  dht1.begin();
  matrix0.begin(0x70);
  matrix1.begin(0x71);
  Serial.println("Setup Complete");
}

void loop()
{

  displayInfo();

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

 
  //Prtint out all the things
  //  Serial.print(gps.date.day);Serial.print(gps.date.month);Serial.print(gps.date.year);
  // Serial.println(gps.time);
  Serial.print("CDS cell value = "); Serial.println(photocellReading);
  Serial.print("Potentiometer reading = "); Serial.print( volts); Serial.println(" V");
  Serial.print(PROBE_0",humid,");
  Serial.print(h0);
  Serial.print(",");
  Serial.print("temp,");
  Serial.print(t0);
  Serial.print(",");
  Serial.print(f0);
  Serial.print(",");
  Serial.print("h_index,");
  Serial.print(hic0);
  Serial.print(",");
  Serial.print(hif0);
  Serial.println();
  Serial.print(PROBE_1",humid,");
  Serial.print(h1);
  Serial.print(",");
  Serial.print("temp,");
  Serial.print(t1);
  Serial.print(",");
  Serial.print(f1);
  Serial.print(",");
  Serial.print("h_index: ");
  Serial.print(hic1);
  Serial.print(",");
  Serial.print(hif1);
  Serial.println();
  Serial.println();

  matrix0.print(f0);
  matrix0.writeDisplay();
  matrix1.print(volts);
  matrix1.writeDisplay();

  delay(sampleDelay); //Wait between samples
}
