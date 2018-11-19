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
#define BAND    425E6  // 915E6
#define PABOOST true
int counter = 0;
long frequency = 425E6;
int sf = 9; // from 6 to 12
long bw = 62; // the closest number to the BW
int CodingRate = 6; // from 5 to 8
#define RECBYTE 13
byte recieveVal[RECBYTE];

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

}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    for (int i = 0 ; i < packetSize; i++ ) {
      recieveVal[i] = LoRa.read();
    }
    float lat = *(float*)(recieveVal);
    float lon = *(float*)(recieveVal + 4);
    float alt = *(float*)(recieveVal + 8);
    uint8_t num_sat = *(int*)(recieveVal + 12);

    Serial.print("gps,");
    Serial.print(lat, 7);
    Serial.print(",");
    Serial.print(lon, 7);
    Serial.print(",");
    Serial.print(alt);
    Serial.print(",");
    Serial.print(num_sat);
    Serial.println();
    // received a packet
    //    Serial.print("Received packet '");
    // read packet
    //    while (LoRa.available()) {
    //      Serial.print((char)LoRa.read());
    //    }
    // print RSSI of packet
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
  }
}


