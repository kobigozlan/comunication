#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// Singleton instance of the radio driver
#define RFM95_CS 48
#define RFM95_RST 49
#define RFM95_INT 2
#define RF95_FREQ 434.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

TinyGPSPlus gps;
float lat, lon, alt;
uint32_t ilat, ilon, ialt;
int count = 0;
unsigned long t1, t2;
int x = 1234;
bool recive = 0;
int AR;

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  lat = 32.504030;
  lon = 35.708090;
  while (!Serial);
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

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

  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    while (1);
  }
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  char radiopacket[70] = {0} ;

  ilat = ((lat - (int)lat) * 1000000);
  ilon = ((lon - (int)lon) * 1000000);
  count++;

  float pascals = baro.getPressure();
  Serial.print(pascals / 3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
  
  AR = analogRead(A8);
  Serial.println(AR);
  AR = map(AR, 0, 1023, 0, 500);

  sprintf(radiopacket, "%d,%d,%d,%d.%lu,%d.%lu,%d,%d,%d,%d,%d,%d",(int)recive, count,AR, (int)(lat), (uint32_t) ((lat - (int)lat) * 1000000), (int)lon, (uint32_t) ((lon - (int)lon) * 1000000), (int)alt, rf95.lastRssi(), rf95.lastSNR(), (int)pascals, (int)altm, (int)(tempC*10));

  Serial.println(radiopacket);
  //  Serial.print("Sending "); Serial.println(radiopacket);
  //  radiopacket[19] = 0;

  t1 = millis();
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 70);

  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  t2 = millis();
  Serial.println(t2 - t1);

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(3000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.print("snr: ");
      Serial.println(rf95.lastSNR(), DEC);
      recive = 1;
    }
    else
    {
      //      Serial.println("Receive failed");
      recive = 0;
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
    recive = 0;
  }

  //  unsigned long ts = micros();
  //  while (micros() - ts < 1000000) {
  //    getGPSData();
  //  }
}

void getGPSData() {
  if (Serial1.available() > 0) {
    //    Serial.println("gps available");
    gps.encode(Serial1.read());
    if (gps.location.isValid())
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();
      //    Serial.print(lat,6);
      //    Serial.print(",");
      //    Serial.println(lon,6);
    }
  }
}
