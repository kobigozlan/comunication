#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;

float myLat = 32.1039;
float myLon = 35.20935;

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
//#define RF95_FREQ 915.0
//#define RF95_FREQ 434.0
//#define RF95_FREQ 440.0
#define RF95_FREQ 446.5
byte recieveVal[25];
uint8_t data[8] = {48, 48, 48, 48, 48, 48, 48,48};;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now
    //    uint8_t buf[23] = {0};
    //    uint8_t len = sizeof(buf);
    uint8_t len = sizeof(recieveVal);

    if (rf95.recv(recieveVal, &len))
    {
      digitalWrite(LED, HIGH);
      //            RH_RF95::printBuffer("Received: ", buf, len);
      float lat = *(float*)(recieveVal);
      float lon = *(float*)(recieveVal + 4);
      float alt = *(float*)(recieveVal + 8);
      uint16_t count = *(int*)(recieveVal + 12);
      uint16_t recive = *(int*)(recieveVal + 14);
      int rssi = *(int*)(recieveVal + 16);
      int snr = *(int*)(recieveVal + 18);
      int bat = *(int*)(recieveVal + 20);
      uint8_t num_sat = *(int*)(recieveVal + 22);
      uint16_t out_count = *(uint16_t*)(recieveVal + 23);

      //      Serial.println((char*)buf);
      Serial.print("massege count :  ");
      Serial.println(count);
      Serial.print("gps lat: ");
      Serial.print(lat, 6);
      Serial.print(" lon: ");
      Serial.print(lon, 6);
      Serial.print(" alt: ");
      Serial.print(alt);
      Serial.print(" sat: ");
      Serial.print(num_sat);
      Serial.print(" dist: ");
      Serial.println(gps.distanceBetween(myLat, myLon, lat, lon));
      Serial.print("param rssi: ");
      Serial.print(rssi);
      Serial.print(" SNR: ");
      Serial.println(snr);
      if (recive) {
        Serial.println("send massege success :-)");
      } else {
        Serial.println("send massege fail :-(");
      }
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.print("SNR: ");
      Serial.println(rf95.lastSNR(), DEC);
      Serial.print("spray out: ");
      Serial.println(out_count);
      Serial.print("BATTARY: ");
      Serial.println(bat);

      int buttonState = digitalRead(7);
//      Serial.println(buttonState);
      if (buttonState) {
        data[1] = 50;
      } else {
        data[1] = 52;
      }
//      Serial.println((char*)data);
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      ////      Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
    Serial.println();
  }
  delay(1);
}
