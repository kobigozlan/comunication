#include <SPI.h>
#include <RH_RF95.h>

//for lOrA32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
byte recieveVal[25];
uint8_t len = sizeof(recieveVal);

#define LED 13

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //  while (!Serial);
  Serial.begin(9600);
  delay(1000);
  

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
  
  Serial.println("start recive gps");
}

void loop()
{
  if (rf95.available())
  {
    if (rf95.recv(recieveVal, &len))
    {
      //      RH_RF95::printBuffer("Received: ", buf, len);
      float lat = *(float*)(recieveVal);
      float lon = *(float*)(recieveVal + 4);
      float alt = *(float*)(recieveVal + 8);
      uint8_t num_sat = *(int*)(recieveVal + 12);

//      Serial.print("gps,");
      Serial.print(lat, 7);
      Serial.print(",");
      Serial.print(lon, 7);
      Serial.print(",");
      Serial.print(alt);
      Serial.print(",");
      Serial.print(num_sat);
      Serial.print(",");
      Serial.print(rf95.lastRssi(), DEC);
      Serial.print(",");
      Serial.print(rf95.lastSNR(), DEC);
      Serial.println();

    }
    else
    {
      Serial1.println("Receive failed");
    }
  }
}


