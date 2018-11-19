#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
float lat, lon, alt;
uint8_t num_sat;

HardwareSerial Serial1(2);

// Pin definetion of WIFI LoRa 32
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)
#define BAND    433E6  // 915E6
#define PABOOST true
int counter = 0;
long frequency = 433E6;
int sf = 9; // from 6 to 12
long bw = 62; // the closest number to the BW
int CodingRate = 6; // from 5 to 8

#define SENDBYTE 13
byte sendVal[SENDBYTE];

// Set LED GPIO
const int ledPin = 25;
// Stores LED state
String ledState;

void setup() {
  // Serial port for debugging purposes
  Serial.begin(9600);
  delay(1000);
  Serial1.begin(9600);
  pinMode(ledPin, OUTPUT);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  lat = 32.123456;
  lon = 35.456789;
  alt = 123;
  num_sat = 12;
}

void loop() {
  DataSend();
  Send();

  unsigned long tss = millis();

  while ((millis() - tss) < 1000)
  {
    getGPSData();
  }
}


void getGPSData() {
  while (Serial1.available()) {
    char x = Serial1.read();
    //    Serial.print(x);
    gps.encode(x);
  }

  if (gps.location.isValid())
  {
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
    num_sat = gps.satellites.value();
    //    Serial.print(alt);
    //    Serial.print(",");
    //    Serial.print(lat, 6);
    //    Serial.print(",");
    //    Serial.println(lon, 6);
  }
}

void Send() {

  // send packet
  LoRa.beginPacket();
  for (int i = 0 ; i < SENDBYTE; i++ ) {
    LoRa.write(sendVal[i]);
  }
  //  LoRa.write(lat);
  //  LoRa.write(lon);
  //  LoRa.write(alt);
  //  LoRa.write(num_sat);
  LoRa.endPacket();

  Serial.print(lat, 6);
  Serial.print(",");
  Serial.print(lon, 6);
  Serial.print(",");
  Serial.println(alt);
}

void DataSend() {
  memcpy(sendVal, &lat, 4);
  memcpy(sendVal + 4, &lon, 4);
  memcpy(sendVal + 8, &alt, 4);
  memcpy(sendVal + 12, &num_sat, 1);
}
