#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

#define LED 13
unsigned long tled = 0;

//for lOrA32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define SENDBYTE 25
#define RF95_FREQ 915.0

#define SENDBYTE 25
byte sendVal[SENDBYTE];

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

TinyGPSPlus gps;
float lat, lon, alt;
uint8_t num_sat;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  delay(1000);
  digitalWrite(LED, LOW);
  Serial1.begin(9600);
  delay(1000);
  digitalWrite(LED, HIGH);
  Serial.println("start transmit gps");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("radio init failed");
    while (1);
  }
  Serial.println("radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  tled = millis();
}

void loop() {

  Send();
  // Now wait for a reply12
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  unsigned long tss = millis();

  while ((millis() - tss) < 1000)
  {
    getGPSData();
    if (rf95.available())
    {
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        //        Serial.print("Got reply: ");
        //        Serial.println((char*)buf);
        //        Serial.println(buf[1]);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        Serial.print("snr: ");
        Serial.println(rf95.lastSNR(), DEC);
      }
      else
      {
        Serial.println("available but not recive");
      }
    }
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
  DataSend();
  delay(10);
  rf95.send((uint8_t *)sendVal, SENDBYTE);
  delay(10);
  rf95.waitPacketSent();
  Serial.print(alt);
  Serial.print(",");
  Serial.print(lat, 6);
  Serial.print(",");
  Serial.println(lon, 6);
}

void DataSend() {
  memcpy(sendVal, &lat, 4);
  memcpy(sendVal + 4, &lon, 4);
  memcpy(sendVal + 8, &alt, 4);
  memcpy(sendVal + 12, &num_sat, 1);
}

