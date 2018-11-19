#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <idDHT11.h>

int idDHT11pin = 3; //Digital pin for comunications
int idDHT11intNumber = 1; //interrupt number (must be the one that use the previus defined pin (see table above)
void dht11_wrapper(); // must be declared before the lib initialization

idDHT11 DHT11(idDHT11pin, idDHT11intNumber, dht11_wrapper);

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// Singleton instance of the radio driver
#define RFM95_CS 48
#define RFM95_RST 49
#define RFM95_INT 2
//#define RF95_FREQ 915.0
#define RF95_FREQ 440.0
//#define RF95_FREQ 430.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
byte sendVal[50];

TinyGPSPlus gps;
float lat, lon, alt;
uint32_t ilat, ilon, ialt;
int count = 0;
unsigned long t1, t2;
int x = 1234;
int recive = 0;
int AR;

float cpm;

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

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    while (1);
  }

  Wire.begin();
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  unsigned long ts = micros();
  count++;
  idDHT11_loop();

  float Humidity = DHT11.getHumidity();
  float Celsius = DHT11.getCelsius();
  float DewPoint = DHT11.getDewPoint();


  bool x = baro.begin();
  float pascals = baro.getPressure();
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();

  int bat = analogRead(A0);

  memcpy(sendVal + 12, &pascals, 4);
  memcpy(sendVal + 16, &altm, 4);
  memcpy(sendVal + 20, &tempC, 4);
  memcpy(sendVal + 24, &count, 2);
  memcpy(sendVal + 26, &recive, 2);
  int rssi = rf95.lastRssi();
  memcpy(sendVal + 28, &rssi, 2);
  int snr = rf95.lastSNR();
  memcpy(sendVal + 30, &snr, 2);
  memcpy(sendVal + 32, &cpm, 4);
  memcpy(sendVal + 36, &Humidity, 4);
  memcpy(sendVal + 40, &Celsius, 4);
  memcpy(sendVal + 44, &DewPoint, 4);
  memcpy(sendVal + 48, &bat, 2);


  //  sprintf(radiopacket, "%d,%d,%d,%d.%lu,%d.%lu,%d,%d,%d,%d,%d,%d", (int)recive, count, AR, (int)(lat), (uint32_t) ((lat - (int)lat) * 1000000), (int)lon, (uint32_t) ((lon - (int)lon) * 1000000), (int)alt, rf95.lastRssi(), rf95.lastSNR(), (int)pascals, (int)altm, (int)(tempC * 10));
  //
  //  Serial.println(radiopacket);
  //  Serial.print("Sending "); Serial.println(radiopacket);
  //  radiopacket[19] = 0;

  t1 = millis();
  //  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)sendVal, 50);

  //  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  t2 = millis();
  Serial.println(t2 - t1);

  // Now wait for a reply12
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(4000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      //      Serial.print("Got reply: ");
      //      Serial.println((char*)buf);
      //      Serial.print("RSSI: ");
      //      Serial.println(rf95.lastRssi(), DEC);
      //      Serial.print("snr: ");
      //      Serial.println(rf95.lastSNR(), DEC);
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
    //    Serial.println("No reply, is there a listener around?");
    recive = 0;
  }

  I2C();
  while (micros() - ts < 6000000) {
    getGPSData();
  }

  //      Serial.println(gps.satellites.value());
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
    //          Serial.print(alt);
    //          Serial.print(",");
    //          Serial.print(lat,6);
    //          Serial.print(",");
    //          Serial.println(lon,6);
    memcpy(sendVal, &lat, 4);
    memcpy(sendVal + 4, &lon, 4);
    memcpy(sendVal + 8, &alt, 4);
  }
}

void I2C() {
  Wire.requestFrom(0xAA, 4);
  delay(50);
  while (Wire.available()) { // slave may send less than requested
    byte recieveVal[4];
    recieveVal[0] = Wire.read();
    recieveVal[1] = Wire.read();
    recieveVal[2] = Wire.read();
    recieveVal[3] = Wire.read();
    cpm = *(float*)(recieveVal);
    Serial.println(cpm);
    Serial.println();
  }
}

