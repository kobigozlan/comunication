#include <IridiumSBD.h>
#include <TinyGPS++.h>
#include <Servo.h>
#define minutes 60000000
//*///////////////////////////*
#define maxAlt 10000
#define maxLon 35.35
#define maxTimeFailSafe 5*minutes
//*//////////////////////////*
#define RSSIInputPin A0
#define interruptPin  2
#define dropValidationPin  6
#define dropValidationGndPin  7
#define servoPin  9


int failSfaeFlag = 0;
bool failSafeCounterFlag = false;
static const int ledPin = 13;

unsigned long inteTimeStart, failsSafeTimer;
bool droneDropped = false;

int err;
int signalQuality = -1;
char sendString [60];

float lat, lon, alt;

// The TinyGPS++ object
TinyGPSPlus gps;
Servo myServo;  // create a servo object
unsigned long CH7PWM;

void setup()
{
  myServo.attach(servoPin); // attaches the servo on pin 9 to the servo object

  pinMode(ledPin, OUTPUT);
  pinMode(dropValidationGndPin, OUTPUT);
  digitalWrite(dropValidationGndPin, LOW);
  pinMode(RSSIInputPin, INPUT);
  pinMode(dropValidationPin, INPUT_PULLUP);
  pinMode(interruptPin, INPUT);
  attachInterrupt(0, interruptCheck, CHANGE);

  Serial.begin(9600);
  delay(1000);
  //  ss.begin(9600);

  //  myServo.write(1600);
  //  Serial.println("start");
  myServo.write(29);

}
uint32_t ts;
void loop() {
  //  Serial.println("loop");
  ts = micros();
  //  myServo.write(2100);
  //  digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);

  while (Serial.available() > 0) {
    char c=Serial.read();
    Serial.print(c);
    gps.encode(c);
  }

  if (gps.location.isValid())
  {
    lat = gps.location.lat();
    lon = gps.location.lng();
    Serial.println(lon);
    if ( lon > maxLon) {
      dropDrone();
    }
  }
  if (gps.altitude.isValid()) {
    alt = gps.altitude.meters();
    if (alt > maxAlt ) {
      dropDrone();
    }
  }
  if ( droneDropped) {
    dropDrone();
  }

  if (CH7PWM > 1650) {
    failSfaeFlag = 0;
    failSafeCounterFlag = false;
  } else if (CH7PWM > 1200) {
    dropDrone();
    failSfaeFlag = 0;
    failSafeCounterFlag = false;
  } else {
    //       dropDrone();
    if (!failSafeCounterFlag) {
      failsSafeTimer = micros();
      failSafeCounterFlag = true;
    }
    failSfaeFlag = 1;
  }
  if ((micros() - failsSafeTimer > maxTimeFailSafe) && failSafeCounterFlag) {
    dropDrone();
  }
  //    delay(100);
}


void interruptCheck() {
  //  Serial.println(CH7PWM);
  if (digitalRead(interruptPin)) {
    inteTimeStart = micros();
  } else {
    //    Serial.println(micros() - inteTimeStart);
    CH7PWM = micros() - inteTimeStart;
    //    Serial.println(CH7PWM);
  }
}


void dropDrone() {
  droneDropped = true;
  myServo.write(100);
  //  Serial.println("dropDrone");
  delay(100);
}

