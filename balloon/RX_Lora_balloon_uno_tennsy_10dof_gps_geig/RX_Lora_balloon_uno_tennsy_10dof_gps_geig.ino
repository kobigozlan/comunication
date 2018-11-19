#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;

float myLat = 32.1039;
float myLon = 35.20935;

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 434.0
byte recieveVal[47];

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
  Serial.begin(115200);
  delay(1000);

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
//    uint8_t buf[71]={0};
//    uint8_t len = sizeof(buf);
    uint8_t len = sizeof(recieveVal);
    
    if (rf95.recv(recieveVal, &len))
    {
      digitalWrite(LED, HIGH);
//      RH_RF95::printBuffer("Received: ", recieveVal, len);
      int16_t model = *(int16_t*)(recieveVal);
      float lat = *(float*)(recieveVal+2);  
      float lon = *(float*)(recieveVal + 6);
      uint16_t alt = *(uint16_t*)(recieveVal + 10);
      float pascals = *(float*)(recieveVal + 12);
      float altm = *(float*)(recieveVal + 16);
      float tempC = *(float*)(recieveVal + 20);
      int16_t count = *(int16_t*)(recieveVal + 24);
      int16_t recive = *(int16_t*)(recieveVal + 26);
      int16_t rssi = *(int16_t*)(recieveVal + 28);
      int16_t snr = *(int16_t*)(recieveVal + 30);
      float cpm =*(float*)(recieveVal + 32); 
      uint8_t hr = *(uint8_t*)(recieveVal + 36);
      uint8_t mn = *(uint8_t*)(recieveVal + 37);
      uint8_t se = *(uint8_t*)(recieveVal + 38);
      float mag =*(float*)(recieveVal + 39);
      float uv =*(float*)(recieveVal + 43);
      
//      Serial.println((char*)buf);
      Serial.print("massege count :  ");      
      Serial.println(count);
      Serial.print("gps lat: ");
      Serial.print(lat,6);
      Serial.print(" lon: ");
      Serial.print(lon,6);
      Serial.print(" alt: ");
      Serial.print(alt);
      Serial.print(" dist: ");
      Serial.println(gps.distanceBetween(myLat,myLon,lat,lon)); 
      Serial.print("gps time: ");
      Serial.print(hr+3); 
      Serial.print(":");
      Serial.print(mn); 
      Serial.print(":"); 
      Serial.println(se); 
      Serial.print("baro pascal :");
      Serial.print(pascals);
      Serial.print(" alt: ");
      Serial.print(altm);
      Serial.print(" temp: ");
      Serial.println(tempC);
      Serial.print("Geiger cpm: ");
      Serial.println(cpm);
      Serial.print("Mag uT: ");
      Serial.println(mag);
      Serial.print("uv :");
      Serial.println(uv);
      Serial.print("param rssi: ");
      Serial.print(rssi);
      Serial.print(" SNR: ");
      Serial.println(snr);
      if(recive){        
      Serial.println("send massege success :-)");
      }else{        
      Serial.println("send massege fail :-(");
      }
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.print("SNR: ");
      Serial.println(rf95.lastSNR(), DEC);  
      
//      // Send a reply
      uint8_t data[] = "0123456789";
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
