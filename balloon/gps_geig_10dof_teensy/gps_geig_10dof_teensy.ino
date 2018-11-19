#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float mag = 0.0f;

Adafruit_BMP280 bmp; // I2C
float pascals, altm, tempC;
int8_t temp;
uint16_t presure;

#define RFM95_RST     3   // "A"
#define RFM95_CS      6   // "B"
#define RFM95_INT     2    // "C"
#define RF95_FREQ 434.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
static const int send_size = 47;
byte sendVal[send_size];
static const int16_t model = 1;

int16_t recive = 0;
int16_t count = 0;
int16_t rssi, snr;

uint8_t bat;
float cpm;
unsigned long ts_loop;

// The TinyGPS++ object
TinyGPSPlus gps;
float lat, lon;
uint8_t Shour, Sminute, Ssecond;
uint16_t alt;

const int UVOUT = A9; //Output from the sensor
float uvIntensity;

void setup()
{
  Serial.begin(115200); //serial
  Serial1.begin(9600);  //gps
  Wire.begin();
  delay(1000);

  pinMode(UVOUT, INPUT);

  delay(100);
  byte settingsArray[] = {0x06, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01}; //
  configureUblox(settingsArray);

  Serial.println("kobi Start");

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  delay(300);
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }


  pinMode(RFM95_INT, INPUT_PULLUP);
  //lora
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  Serial.println("LoRa TX Test!");
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

}

void loop()
{
  ts_loop = millis();
  count++;
  I2C_geiger();
  delay(100);
  baroloop();
  delay(100);
  magloop();
  uv_loop();
  Save_data();
  //  Print_data();
  Send_data();
  Recive();
  while (millis() - ts_loop < 6000) {
    getGPSData();
  }
}

void getGPSData() {
  while (Serial1.available() > 0) {
    char x = Serial1.read();
        Serial.print(x);
    gps.encode(x);
  }
  //  Serial.println();
  //  Serial.print("sat: ");
  //  Serial.println(gps.satellites.value());
  //  Serial.print("valid: ");
  //  Serial.println(gps.location.isValid());
  if (gps.location.isValid())
  {
    lat = gps.location.lat();
    lon = gps.location.lng();
    alt = gps.altitude.meters();
    Shour = gps.time.hour();
    Sminute = gps.time.minute();
    Ssecond = gps.time.second();
        Serial.print("h:");
        Serial.println(gps.time.hour());
        Serial.print("m:");
        Serial.println(gps.time.minute());
        Serial.print("s:");
        Serial.println(gps.time.second());
        Serial.print(lat, 6);
        Serial.print(",");
        Serial.println(lon, 6);
        Serial.print("alt: ");
        Serial.println(alt);
  }
}

float I2C_geiger() {
  Wire.requestFrom(0xAA, 4);
  delay(50);
  //  Serial.println("Start i2c ");
  //  Serial.println(Wire.available());
  while (Wire.available()) { // slave may send less than requested
    byte recieveVal[4];
    recieveVal[0] = Wire.read();
    recieveVal[1] = Wire.read();
    recieveVal[2] = Wire.read();
    recieveVal[3] = Wire.read();
    cpm = *(float*)(recieveVal);
    //    Serial.println(cpm);
    //    Serial.println();
  }
}

void baroloop() {
  pascals = bmp.readPressure();
  altm = bmp.readAltitude(1013.25);
  tempC = bmp.readTemperature();
  temp = tempC;
  presure = pascals;

//    Serial.print(F("Temperature = "));
//    Serial.print(temp);
//    Serial.println(" *C");
//    Serial.print(F("Pressure = "));
//    Serial.print(presure);
//    Serial.println(" Pa");
//    Serial.print(F("Approx altitude = "));
//    Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
//    Serial.println(" m");
//    Serial.println();
}

void magloop() {
  IMU.readSensor();
  float mx = IMU.getMagX_uT();
  float my = IMU.getMagY_uT();
  float mz = IMU.getMagZ_uT();
  mag = sqrt((mx * mx) + (my * my) + (mz * mz));
//    Serial.print(mag);
//    Serial.println(" uT");
}

void uv_loop() {
  byte numberOfReadings = 8;
  uint16_t runningValue = 0;
  for (int x = 0 ; x < numberOfReadings ; x++) {
    runningValue += analogRead(UVOUT);
  }
  runningValue /= numberOfReadings;

  float outputVoltage = 3.3  * runningValue;
  uvIntensity = (outputVoltage - 0.99) * (15) / (2.8 - 0.99);

  //  Serial.print("output: ");
  //  Serial.print(runningValue);
  //  Serial.print(" / UV Intensity (mW/cm^2): ");
  //  Serial.print(uvIntensity);
  //  Serial.println();
  delay(100);
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

void Save_data() {
  memcpy(sendVal, &model, 2);
  memcpy(sendVal + 2, &lat, 4);
  memcpy(sendVal + 6, &lon, 4);
  memcpy(sendVal + 10, &alt, 2);
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
  memcpy(sendVal + 36, &Shour, 1);
  memcpy(sendVal + 37, &Sminute, 1);
  memcpy(sendVal + 38, &Ssecond, 1);
  memcpy(sendVal + 39, &mag, 4);
  memcpy(sendVal + 43, &uvIntensity, 4);
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
  //  Serial.println("reciveing...");
  // Now wait for a reply12
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  //  Serial.println("Waiting for reply...");
  delay(10);
  if (rf95.waitAvailableTimeout(3000))
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

