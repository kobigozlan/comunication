#include <IridiumSBD.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Servo.h>

Servo myservo;
int val = 1000;

static const int ledPin = 13;
float cpm;
uint16_t cpmsend[5];
uint16_t altsend[5];
uint32_t latsend [5];
uint32_t lonsend [5];

unsigned long ti1, ti2;
unsigned long tcb1, tcb2;

//SoftwareSerial nss(18, 19);
IridiumSBD isbd(Serial3, 6);
int signalQuality = -1;
int8_t sig_send = -1;
int err;
byte sendVal[50];

// The TinyGPS++ object
TinyGPSPlus gps;
float lat, lon;
uint8_t Shour, Sminute, Ssecond;
uint32_t ilat, ilon;
uint16_t alt,prev_alt;

void setup()
{
  pinMode(13,OUTPUT);
  digitalWrite(13,1);
    
  Serial.begin(115200); //serial
  Serial3.begin(19200); //iridium
  Serial1.begin(9600);  //gps
  Wire.begin();

  delay(2000);
  Serial.println("Start");
  
  digitalWrite(13,0);
  myservo.attach(3);
  delay(500);
  myservo.writeMicroseconds(2000);
  delay(1000);
  myservo.writeMicroseconds(1000);
  Serial.println("servo 1000");  
  delay(1000);
  myservo.writeMicroseconds(1500);
  delay(1000);
  myservo.writeMicroseconds(1000);

  digitalWrite(13,1);
  
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.begin();
  isbd.setPowerProfile(1);

  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
    return;
  }

  Serial.print("Signal quality is ");
  Serial.println(signalQuality);
  
}

void loop()
{
  unsigned long ts = millis();

  Save_data();
  ird_sr();
  //  printdata();
  //  exit(0);
  Serial.println("start for");
  for (int i = 0; i < 5; i++) {
    while (millis() - ts < 1000 * 60 * (i*2 + 2)) {
      getGPSData();
    }
    float g = I2C();
    cpmsend[i] = g * 100;
    latsend[i] = ilat;
    lonsend[i] = ilon;
    altsend[i] = alt;
    //    printdata(i);
    //    Serial.println(i);
    toRelease(alt);
  }

}

//Vlad Landa
void toRelease(uint16_t alt){
  //if alt under 5km and was before and in desent open for 3sec, wait to second call if still on decent 
  if(alt <= 5000 && alt > 800){
    if(alt - prev_alt < 0){
      myservo.writeMicroseconds(2000);
      delay(3000);
    }
  }
  myservo.writeMicroseconds(1000);
  prev_alt = alt;
  
}

bool ISBDCallback()
{
  //  Serial.println("callback");
  getGPSData();
  digitalWrite(ledPin, (millis() / 100) % 2 == 1 ? HIGH : LOW);
  return true;
}

void auto_level(){
  
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

  if (bufferSize <= 10) {
    myservo.writeMicroseconds(2000);
    delay(3000);
  }
  if (bufferSize > 10 && bufferSize <= 20) {
    myservo.writeMicroseconds(2000);
    delay(6000);
  }
  if (bufferSize > 20) {
    myservo.writeMicroseconds(2000);
    delay(10000);
  }
  myservo.writeMicroseconds(1000);

  Serial.print("Inbound buffer size is ");
  Serial.println(bufferSize);
  for (int i = 0; i < bufferSize; ++i)
  {
    Serial.write(buffer[i]);
    Serial.print("(");
    Serial.print(buffer[i], HEX);
    Serial.print(") ");
  }
  Serial.print("Messages left: ");
  Serial.println(isbd.getWaitingMessageCount());

  
  int msg_in = isbd.getWaitingMessageCount();
  if (msg_in > 0){
    err = isbd.sendReceiveSBDBinary(sendVal, 50, buffer, bufferSize);
  }
}

void getGPSData() {
  if (Serial1.available() > 0) {
    char x = Serial1.read();
    //    Serial.print(x);
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
  Serial.println("Start i2c ");
  Serial.println(Wire.available());
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

void Save_data() {
  memcpy(sendVal, &latsend[0], 4);
  memcpy(sendVal + 3, &lonsend[0], 4);
  memcpy(sendVal + 6, &altsend[0], 2);
  memcpy(sendVal + 8, &cpmsend[0], 2);
  memcpy(sendVal + 10, &latsend[1], 4);
  memcpy(sendVal + 13, &lonsend[1], 4);
  memcpy(sendVal + 16, &altsend[1], 2);
  memcpy(sendVal + 18, &cpmsend[1], 2);
  memcpy(sendVal + 20, &latsend[2], 4);
  memcpy(sendVal + 23, &lonsend[2], 4);
  memcpy(sendVal + 26, &altsend[2], 2);
  memcpy(sendVal + 28, &cpmsend[2], 2);
  memcpy(sendVal + 30, &latsend[3], 4);
  memcpy(sendVal + 33, &lonsend[3], 4);
  memcpy(sendVal + 36, &altsend[3], 2);
  memcpy(sendVal + 38, &cpmsend[3], 2);
  memcpy(sendVal + 40, &latsend[4], 4);
  memcpy(sendVal + 43, &lonsend[4], 4);
  memcpy(sendVal + 46, &altsend[4], 2);
  memcpy(sendVal + 48, &cpmsend[4], 2);
}

void printdata(int x ) {
  Serial.print(alt);
  Serial.print(",");
  Serial.print(lat, 6);
  Serial.print(",");
  Serial.println(lon, 6);
  Serial.print(altsend[x]);
  Serial.print(",");
  Serial.print(latsend[x]);
  Serial.print(",");
  Serial.println(lonsend[x]);
  Serial.print("cpm: ");
  Serial.println(cpmsend[x]);
  //  RH_RF95::printBuffer("Received: ", sendVal, 8);
}

