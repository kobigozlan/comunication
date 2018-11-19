#include <IridiumSBD.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>

Servo pump_servo;
int val = 1000;
Servo hit_servo;

Adafruit_BMP280 bmp; // I2C
int8_t temp;
uint16_t presure;

uint8_t bat;

static const int ledPin = 13;
float cpm;
uint16_t cpmsend[10];
uint16_t altsend[10];

unsigned long ts_loop;

IridiumSBD isbd(Serial2, 12);
int signalQuality = -1;
int8_t sig_send = -1;
int err;
byte sendVal[50];

// The TinyGPS++ object
TinyGPSPlus gps;
float lat, lon;
uint8_t Shour, Sminute, Ssecond;
uint32_t ilat, ilon;
uint16_t alt, prev_alt;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);

  Serial.begin(115200); //serial
  Serial2.begin(19200); //iridium
  Serial1.begin(9600);  //gps
  Wire.begin();

  delay(2000);
  Serial.println("Start");

  digitalWrite(13, 0);
  hit_servo.attach(23);
  delay(500);
  hit_servo.writeMicroseconds(2000);
  delay(1000);
  hit_servo.writeMicroseconds(1000);
  delay(1000);
  hit_servo.writeMicroseconds(1420);
  
  pump_servo.attach(6);
  delay(500);
  pump_servo.writeMicroseconds(2000);
  delay(1000);
  pump_servo.writeMicroseconds(1000);
  Serial.println("pump 1000");
  delay(1000);
  pump_servo.writeMicroseconds(2000);
  delay(1000);
  pump_servo.writeMicroseconds(1000);

  if (!bmp.begin()) {  
//    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  digitalWrite(13, 1);

  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.begin();
  isbd.setPowerProfile(1);

  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0)
  {
//    Serial.print("SignalQuality failed: error ");
//    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);

}

void loop()
{
  ts_loop = millis();
  //  exit(0);
  for (int i = 0; i < 10; i++) {
    while (millis() - ts_loop < 1000 * 60 * (i + 1)) {
      getGPSData();
    }
    float g = I2C();
    cpmsend[i] = g * 100;
    altsend[i] = alt;
    //    printdata(i);
    //    Serial.println(i);
    toRelease(alt);
  }
  bat = analogRead(A1)/4;  //need to divide by 0.019 4.19 is 221
//  Serial.println(bat*0.019);
  baroloop();
  data_to_send();
  ird_sr();
}

//Vlad Landa
void toRelease(uint16_t alt) {
  //if alt under 5km and was before and in desent open for 3sec, wait to second call if still on decent
  if (alt <= 5000 && alt > 800) {
    if (alt - prev_alt < 0) {
      pump_servo.writeMicroseconds(2000);
      delay(3000);
    }
  }
  pump_servo.writeMicroseconds(1000);
  prev_alt = alt;

}

bool ISBDCallback()
{
  //  Serial.println("callback");
  getGPSData();
  digitalWrite(ledPin, (millis() / 100) % 2 == 1 ? HIGH : LOW);
  return true;
}

void ird_sr()
{
  uint8_t buffer[200] = { 0 };
  size_t bufferSize = sizeof(buffer);
  err = isbd.sendReceiveSBDBinary(sendVal, 50, buffer, bufferSize);
  if (err != 0)
  {
    Serial.print("sendReceiveSBDText failed: error ");
    Serial.println(err);
    return;
  }

  if (bufferSize > 5 && bufferSize <= 10) {
    pump_servo.writeMicroseconds(2000);
    delay(3000);
  }
  if (bufferSize > 10 && bufferSize <= 20) {
    pump_servo.writeMicroseconds(2000);
    delay(6000);
  }
  if (bufferSize > 20) {
    pump_servo.writeMicroseconds(2000);
    delay(10000);
  }
  pump_servo.writeMicroseconds(1000);

//  Serial.print("Inbound buffer size is ");
//  Serial.println(bufferSize);
//  for (int i = 0; i < bufferSize; ++i)
//  {
//    Serial.write(buffer[i]);
//    Serial.print("(");
//    Serial.print(buffer[i], HEX);
//    Serial.print(") ");
//  }
//  Serial.print("Messages left: ");
//  Serial.println(isbd.getWaitingMessageCount());


  int msg_in = isbd.getWaitingMessageCount();
  if (msg_in > 0) {
    err = isbd.sendReceiveSBDBinary(sendVal, 50, buffer, bufferSize);
  }
}

void getGPSData() {
  if (Serial1.available() > 0) {
    char x = Serial1.read();
//        Serial.print(x);
    gps.encode(x);
    if (gps.location.isValid())
    {
      lat = gps.location.lat();
      lon = gps.location.lng();
      alt = gps.altitude.meters();
      ilat = (lat * 100000);
      ilon = (lon * 100000);
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

float I2C() {
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
    Serial.println(cpm);
    Serial.println();
  }
  return cpm;
}

void baroloop() {
  float temp1 = bmp.readTemperature();
  float presure1 = bmp.readPressure();
  temp = temp1;
  presure = presure1;

//  Serial.print(F("Temperature = "));
//  Serial.print(temp);
//  Serial.println(" *C");
//  Serial.print(F("Pressure = "));
//  Serial.print(presure);
//  Serial.println(" Pa");
//  Serial.print(F("Approx altitude = "));
//  Serial.print(bmp.readAltitude(1013.25)); // this should be adjusted to your local forcase
//  Serial.println(" m");
//  Serial.println();
}

void data_to_send() {
  memcpy(sendVal, &ilat, 4);
  memcpy(sendVal + 3, &ilon, 4);
  memcpy(sendVal + 6, &presure, 2);
  memcpy(sendVal + 8, &temp, 1);
  memcpy(sendVal + 9, &bat, 1);
  memcpy(sendVal + 10, &cpmsend[0], 2);
  memcpy(sendVal + 12, &altsend[0], 2);
  memcpy(sendVal + 14, &cpmsend[1], 2);
  memcpy(sendVal + 16, &altsend[1], 2);
  memcpy(sendVal + 18, &cpmsend[2], 2);
  memcpy(sendVal + 20, &altsend[2], 2);
  memcpy(sendVal + 22, &cpmsend[3], 2);
  memcpy(sendVal + 24, &altsend[3], 2);
  memcpy(sendVal + 26, &cpmsend[4], 2);
  memcpy(sendVal + 28, &altsend[4], 2);
  memcpy(sendVal + 30, &cpmsend[5], 2);
  memcpy(sendVal + 32, &altsend[5], 2);
  memcpy(sendVal + 34, &cpmsend[6], 2);
  memcpy(sendVal + 36, &altsend[6], 2);
  memcpy(sendVal + 38, &cpmsend[7], 2);
  memcpy(sendVal + 40, &altsend[7], 2);
  memcpy(sendVal + 42, &cpmsend[8], 2);
  memcpy(sendVal + 44, &altsend[8], 2);
  memcpy(sendVal + 46, &cpmsend[9], 2);
  memcpy(sendVal + 48, &altsend[9], 2);
}

void printdata(int x ) {
  Serial.print(alt);
  Serial.print(",");
  Serial.print(lat, 6);
  Serial.print(",");
  Serial.println(lon, 6);
  Serial.print(altsend[x]);
  Serial.print(",");
  Serial.print(lat);
  Serial.print(",");
  Serial.println(lon);
  Serial.print("cpm: ");
  Serial.println(cpmsend[x]);
  //  RH_RF95::printBuffer("Received: ", sendVal, 8);
}

