#include <TinyGPS++.h>
#include "wiring_private.h"
#include <IridiumSBD.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

static const int ledPin = 13;

Adafruit_BMP280 bmp; // I2C
float pascals, altm, tempC;

//for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 434.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
static const int send_size = 42;
byte sendVal[send_size];
int16_t recive = 0;
int16_t count = 0;
unsigned long t1, t2;
int16_t rssi, snr;

int16_t bat;
float cpm;

IridiumSBD isbd(Serial1, 6);
int signalQuality = -1;
int8_t sig_send = -1;
int err;
unsigned long ti1, ti2;
unsigned long tcb1, tcb2;
bool cb_start = 0;

// The TinyGPS++ object
TinyGPSPlus gps;
float lat, lon, alt;
uint8_t Shour, Sminute, Ssecond;

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);//pin 10 TX, pin 11  RX
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);

  //gps
  Serial2.begin(9600);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  //irdium
  Serial1.begin(19200);
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.begin();
  isbd.setPowerProfile(1);
  isbd.adjustSendReceiveTimeout(30);
  pinMode(ledPin, OUTPUT);

  //lora
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  Serial.println("Feather LoRa TX Test!");
  digitalWrite(RFM95_RST, LOW); // manual reset
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
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  rf95.setTxPower(23, false);

  bmp.begin();

  ti1 = millis();
}

void loop()
{
  unsigned long ts = micros();
  count++;

  bat = analogRead(A7) * 0.63;

  //  t1 = millis();
  err = isbd.getSignalQuality(signalQuality);
  //  t2 = millis();
  //  Serial.println(t2 - t1);
  sig_send = signalQuality;
  Serial.print(err);
  Serial.print("  Signal quality is ");
  Serial.println(signalQuality);

  getGPSData();
  Baro_read();
  I2C();
  Save_data();
  Print_data();
  Send_data();
  Recive();

  ti2 = millis();

  if (ti2 - ti1 > 1000 * 60 * 10) {
    ti1 = millis();
    ird_send();
  }

  while (micros() - ts < 10000000) {
    getGPSData();
  }

}

bool ISBDCallback()
{
  //  Serial.println("callback");
  getGPSData();
  digitalWrite(ledPin, (millis() / 100) % 2 == 1 ? HIGH : LOW);
  return true;
}

void getGPSData() {
  if (Serial2.available() > 0) {
    char x = Serial2.read();
    //    Serial.print(x);
    gps.encode(x);
    if (gps.location.isValid())
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();
      Shour = gps.time.hour();
      Sminute = gps.time.minute();
      Ssecond = gps.time.second();
      //      Serial.print("h:");
      //      Serial.println(gps.time.hour());
      //      Serial.print("m:");
      //      Serial.println(gps.time.minute());
      //      Serial.print("s:");
      //      Serial.println(gps.time.second());
      //      Serial.print(lat, 6);
      //      Serial.print(",");
      //      Serial.println(lon, 6);
    }
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

void Save_data() {
  memcpy(sendVal, &lat, 4);
  memcpy(sendVal + 4, &lon, 4);
  memcpy(sendVal + 8, &alt, 4);
  memcpy(sendVal + 12, &pascals, 4);
  memcpy(sendVal + 16, &altm, 4);
  memcpy(sendVal + 20, &tempC, 4);
  memcpy(sendVal + 24, &count, 2);
  memcpy(sendVal + 26, &recive, 2);
  rssi = rf95.lastRssi();
  memcpy(sendVal + 28, &rssi, 2);
  snr = rf95.lastSNR();
  memcpy(sendVal + 30, &snr, 2);
  memcpy(sendVal + 32, &cpm, 4);
  memcpy(sendVal + 36, &bat, 2);
  memcpy(sendVal + 38, &sig_send, 1);
  memcpy(sendVal + 39, &Shour, 1);
  memcpy(sendVal + 40, &Sminute, 1);
  memcpy(sendVal + 41, &Ssecond, 1);
}

void Send_data() {
  //  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)sendVal, send_size);
  //  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
}

void Recive() {
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
}

void Baro_read() {
  //  bool x = baro.begin();
  pascals = bmp.readPressure();
  altm = bmp.readAltitude(1013.25);
  tempC = bmp.readTemperature();
}

void Print_data () {
  Serial.print(F("Temperature = "));
  Serial.print(tempC);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pascals);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(altm); // this should be adjusted to your local forcase
  Serial.println(" m");

  Serial.print(F("lat:  "));
  Serial.print(lat);
  Serial.println(" ");

  Serial.print(F("lon:  "));
  Serial.print(lon);
  Serial.println(" ");

  Serial.print(F("alt gps: "));
  Serial.print(alt); // this should be adjusted to your local forcase
  Serial.println(" m");
}

void ird_send() {
  char radiopacket[19] = {0};
  sprintf(radiopacket, "%d,%d.%lu,%d.%lu,%d,%d", count, (int)(lat), (uint32_t) ((lat - (int)lat) * 1000000), (int)lon, (uint32_t) ((lon - (int)lon) * 1000000), (int)alt, (int)(tempC * 10));
  Serial.println(radiopacket);
  err = isbd.sendSBDText(radiopacket);
}


