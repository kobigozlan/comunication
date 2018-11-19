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
#define RF95_FREQ 434.0
//#define RF95_FREQ 446.5
byte recieveVal[51];

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

//   The default transmitter power is 13dBm, using PA_BOOST.
//   If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
//   you can set transmitter powers from 5 to 23 dBm:
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
//      RH_RF95::printBuffer("Received: ", buf, len);
      float lat = *(float*)(recieveVal);  
      float lon = *(float*)(recieveVal + 4);
      float alt = *(uint16_t*)(recieveVal + 8);
      float pascals = *(float*)(recieveVal + 12);
      float altm = *(float*)(recieveVal + 16);
      float tempC = *(float*)(recieveVal + 20);
      int count = *(int*)(recieveVal + 24);
      int recive = *(int*)(recieveVal + 26);
      int rssi = *(int*)(recieveVal + 28);
      int snr = *(int*)(recieveVal + 30);
      float cpm =*(float*)(recieveVal + 32);  
      float Humidity = *(float*)(recieveVal + 36);  
      float Celsius = *(float*)(recieveVal + 40);  
      float DewPoint = *(float*)(recieveVal + 44);  
      int bat = *(int*)(recieveVal + 48);  
      byte sat = *(byte*)(recieveVal + 50);  
      
//      Serial.println((char*)buf);
      Serial.print("massege: count: ");      
      Serial.println(count);
      Serial.print("gps lat: ");
      Serial.print(lat,6);
      Serial.print(" lon: ");
      Serial.print(lon,6);
      Serial.print(" alt: ");
      Serial.print(alt);
      Serial.print(" sat: ");
      Serial.print(sat);
      Serial.print(" dist: ");
      Serial.println(gps.distanceBetween(myLat,myLon,lat,lon));
      Serial.print("baro: pascal: ");
      Serial.print(pascals);
      Serial.print(" alt: ");
      Serial.print(altm);
      Serial.print(" temp: ");
      Serial.println(tempC);     
      Serial.print("Geiger: cpm: ");
      Serial.println(cpm);
      Serial.print("HTD: Humidity: ");
      Serial.print(Humidity);
      Serial.print(" Temp: ");
      Serial.print(Celsius);
      Serial.print(" DewPoint: ");
      Serial.println(DewPoint);
      Serial.print("param: rssi: ");
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
      Serial.print("BATTARY: ");
      Serial.println((bat/205.4)); 

       Serial.println("finish Receive starting send");  
//       delay(500);    
      // Send a reply
      uint8_t data[] = "0123456789";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent finish");
      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
    Serial.println();
  }
//  delay(500);
}
