#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

#define MotorPin 6
bool motor = 0;
uint16_t out_count = 0;
// Singleton instance of the radio driver
#define RFM95_CS 48
#define RFM95_RST 49
#define RFM95_INT 2
//#define RF95_FREQ 915.0
#define RF95_FREQ 446.5
//#define RF95_FREQ 430.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
#define SENDBYTE 25
byte sendVal[SENDBYTE];

TinyGPSPlus gps;
float lat, lon, alt;
float prev_alt = 0;
float ref_alt = -1;
uint8_t num_sat;
uint16_t count = 0;
unsigned long t1, t2, t3;
uint16_t recive = 0;

int bat;
int rssi;
int snr;


void setup()
{
  pinMode(MotorPin, OUTPUT);
  digitalWrite(MotorPin, LOW);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  lat = 32.504030;
  lon = 35.708090;
  //  while (!Serial);
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(100);
  byte settingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01}; //
  configureUblox(settingsArray);

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

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  unsigned long ts = millis();
  count++;

  bat = analogRead(A0);
  rssi = rf95.lastRssi();
  snr = rf95.lastSNR();

  DataSend();

  t1 = millis();
  //  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)sendVal, SENDBYTE);

  //  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  t2 = millis();
  //  Serial.println(t2 - t1);

  // Now wait for a reply12
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //  Serial.println("Waiting for reply...");
  delay(10);

  t3 = millis();

  while ((millis() - t3) < 3000)
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
        recive = 1;
        if (buf[1] == 50 && motor == 0) {
          drop();
          motor = 1;
          Serial.print("spray out: ");
          Serial.println(out_count);
        }
        if (buf[1] == 52 && motor == 1) {
          motor = 0;
        }
        break;
      }
      else
      {
        //      Serial.println("Receive failed");
        recive = 0;
        Serial.println("available but not recive");
      }

    }

  }
  toRelease(alt);
  while (millis() - ts < 5000) {
    getGPSData();
  }
  //      Serial.println(gps.satellites.value());
}

//Vlad Landa
void toRelease(float alt) {
  //if alt under 5km and was before and in desent open for 3sec, wait to second call if still on decent
  if (alt <= 6000 && alt > 800) {
    if (alt - prev_alt < 0) {
      if (ref_alt == -1) {
        ref_alt = prev_alt;
      } else if (ref_alt - alt > 100) {
        drop();
        ref_alt = -1;
      }
    } else {
      ref_alt = -1;
    }
  }
  prev_alt = alt;
  digitalWrite(MotorPin, LOW);
}

void drop() {
  digitalWrite(MotorPin, HIGH);
  delay(3000);
  digitalWrite(MotorPin, LOW);
  out_count++;
}

void getGPSData() {
  while (Serial1.available()) {
    char x = Serial1.read();
    //        Serial.print(x);
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
    //    Serial.print(lat,6);
    //    Serial.print(",");
    //    Serial.println(lon,6);
  }
}

void DataSend() {
  memcpy(sendVal, &lat, 4);
  memcpy(sendVal + 4, &lon, 4);
  memcpy(sendVal + 8, &alt, 4);
  memcpy(sendVal + 12, &count, 2);
  memcpy(sendVal + 14, &recive, 2);
  memcpy(sendVal + 16, &rssi, 2);
  memcpy(sendVal + 18, &snr, 2);
  memcpy(sendVal + 20, &bat, 2);
  memcpy(sendVal + 22, &num_sat, 1);
  memcpy(sendVal + 23, &out_count, 2);
}


