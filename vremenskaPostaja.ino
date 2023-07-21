
/*
Vremenska postaja
Zavod Kersnikova

Senzorji(+i2c naslov):

AHT-20 (0x38)
Gravity iic oxygen (0x73)
CJMCU-811 (0x5a)
GY-302 (0x23)

Senzorje se priključi na Arduino Uno preko ethernet shield-a.
Modul deluje kot strežnik. Na IP naslovu 192.168.1.177 preberemo informacije o temperaturi, vlagi, svetlobi, kisiku in CO2 v Json formatu.
 */
#include "Adafruit_CCS811.h"
#include "DFRobot_OxygenSensor.h"
#include <Wire.h>  //BH1750 IIC Mode
#include <math.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <AHT20.h>
#include "SPI.h"
#include "Ethernet.h"
#include <ArduinoJson.h>


float measuredTemperature;
float measuredHumidity;
float measuredLuminosity;
float measuredOxygen;
float measuredDioxyde;

byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0xAA, 0xE7 };
IPAddress ip(192, 168, 1, 177);
EthernetServer server(80);

#define JSON_SIZE 200
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER 10  // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;
int BH1750address = 0x23;
byte buff[2];
Adafruit_CCS811 ccs;

//AHT 20
AHT20 aht20;
unsigned long previousMillis = 0;
const long AHTinterval = 1000;

void setup(void) {
  Wire.begin();
  Serial.begin(9600);
  while (!oxygen.begin(Oxygen_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  Serial.println("CCS811 test");

  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1)
      ;
  }

  AHTsetup();

  // Wait for the sensor to be ready
  while (!ccs.available())
    ;


  Ethernet.begin(mac, ip);  // Initialize l'arduino comme un élément du réseau local
  server.begin();           // Se met à l'écoute des communications client (browser web)
  Serial.print("Server is at ");
  Serial.println(Ethernet.localIP());
}

void loop(void) {
  measuredOxygen = oxygenloop();
  measuredLuminosity = (float)lightloop();
  dioxydeloop();

  unsigned long currentMillis = millis();

  EthernetClient client = server.available();
  if (currentMillis - previousMillis >= AHTinterval) {
    AHTloop();
  }
  if (client) {
    // Print a message to the serial monitor
    Serial.println("New client");

    // Create a JSON document object
    StaticJsonDocument<JSON_SIZE> doc;

    float temp = 0;   // test
    float humid = 0;  //test

    float lumin = measuredLuminosity;
    float co2 = ccs.geteCO2();
    float o2 = measuredOxygen;

    doc["temperature"] = temp;
    doc["humidity"] = humid;
    doc["luminosity"] = lumin;
    doc["co2"] = co2;
    doc["o2"] = o2;


    // Wait for a request from the client
    while (client.connected()) {
      if (client.available()) {
        // Read the first line of the request
        String request = client.readStringUntil('\r');

        // Print the request to the serial monitor
        Serial.print("Request: ");
        Serial.println(request);

        // Ignore the rest of the request
        client.flush();

        // Send a response header to the client
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();

        // Serialize and send the JSON document to the client
        serializeJson(doc, client);

        // Print a message to the serial monitor
        Serial.println("Response sent");

        // Break out of the loop
        break;
      }
    }

    // Close the connection with the client
    client.stop();

    // Print a message to the serial monitor
    Serial.println("Client disconnected");

    // Wait for a second before next loop iteration
    delay(1000);
  }


  //delay(1000);
}
void dioxydeloop() {
  if (ccs.available()) {
    if (!ccs.readData()) {
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
    } else {
      Serial.println("ERROR!");
      while (1)
        ;
    }
  }
}

float oxygenloop() {
  float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.print(" oxygen concentration is ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
  return oxygenData;
}
int BH1750_Read(int address)  //
{
  int i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available())  //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();
  return i;
}

int lightloop() {
  int i;
  int luminosityValue;
  uint16_t val = 0;
  BH1750_Init(BH1750address);
  delay(200);

  if (2 == BH1750_Read(BH1750address)) {
    val = ((buff[0] << 8) | buff[1]) / 1.2;
    Serial.print(val, DEC);
    Serial.println("[lx]");
    luminosityValue = val;
  }
  return luminosityValue;
}

void BH1750_Init(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x10);  //1lx reolution 120ms
  Wire.endTransmission();
}

void AHTloop() {
  if (aht20.available() == true) {
    //Get the new temperature and humidity value
    float temperature = aht20.getTemperature();
    float humidity = aht20.getHumidity();

    //Print the results
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print(" C\t");
    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.print("% RH");

    Serial.println();
    measuredHumidity = humidity;
    measuredTemperature = temperature;
  }
}
void AHTsetup() {

  if (aht20.begin() == false) {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1)
      ;
  }
  Serial.println("AHT20 acknowledged.");
}
