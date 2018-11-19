#include <SPI.h>
#include <RH_RF95.h>
//#include <RH_RF69.h>
#include <Servo.h>
#include <Encoder.h>

#define enablePin 10
#define phasePin 11
#define pb A1
#define MaxPos 10000
#define SPEED 150
Encoder myEnc(6, 12);

Servo ServoMeter;
uint16_t MeterServoState = 1500 ;
#define servoPin 5

Servo ServoCam;
#define CamServoPin 13
#define CamServoDown 1100
#define CamServoUp 2000


#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM95_RST     4
#endif

//#define RFM69_CS      10
//#define RFM69_IRQ     8
//#define RFM95_RST     4

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM69_CS, RFM69_IRQ);

// Blinky on receipt
#define LED 13

//RH_RF69 rf95(8, 3);

void setup()
{
  pinMode(enablePin, OUTPUT);
  pinMode(phasePin, OUTPUT);
  pinMode(pb, INPUT_PULLUP);

  delay(2000);
  while (digitalRead(pb) == HIGH) {
    analogWrite(enablePin, SPEED);
    digitalWrite(phasePin, LOW);
    delay(5);
  }

  analogWrite(enablePin, 0);
  digitalWrite(phasePin, LOW);
  myEnc.write(0);

  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  ServoMeter.attach(servoPin);
  ServoMeter.writeMicroseconds(MeterServoState);
  delay(25);
  ServoMeter.detach();
  delay(25);

  ServoCam.attach(CamServoPin);
  ServoCam.writeMicroseconds(CamServoDown);
  delay(100);
  ServoCam.detach();

  //  while (!Serial);
  Serial.begin(115200);
  delay(2000);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  //  digitalWrite(RFM95_RST, LOW);
  //  delay(10);
  //  digitalWrite(RFM95_RST, HIGH);
  //  delay(10);

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
  rf95.setModemConfig(rf95.Bw125Cr45Sf128);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  if (digitalRead(pb) == LOW) {
    analogWrite(enablePin, 0);
    digitalWrite(phasePin, LOW);
    myEnc.write(0);
  }

  long newPosition = myEnc.read();
  Serial.println(newPosition);
  //  if (newPosition > MaxPos) {
  //    analogWrite(enablePin, 0);
  //    digitalWrite(phasePin, LOW);
  //    Serial.println(newPosition);
  //  }

  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[20];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      //      digitalWrite(LED, HIGH);
      //      RH_RF95::printBuffer("Received: ", buf, len);
      //      Serial.print("Got: ");
      //      Serial.print(buf[0]);
      //      Serial.print(" , ");
      //      Serial.print(buf[1]);
      //      Serial.print(" , ");
      //      Serial.print(buf[2]);
      //      Serial.print("  RSSI: ");
      //      Serial.print(rf95.lastRssi(), DEC);

      //        Serial.println(buf[0]);
      if (buf[0] > 220 && (digitalRead(pb) == HIGH))
      {
        analogWrite(enablePin, SPEED);
        digitalWrite(phasePin, LOW);
        //        Serial.println("in");
      }
      else if (buf[0] < 50 && newPosition < MaxPos)
      {
        analogWrite(enablePin, SPEED);
        digitalWrite(phasePin, HIGH);
        //        Serial.println("out");
      }
      else
      {
        analogWrite(enablePin, 0);
        digitalWrite(phasePin, LOW);
      }

      if (buf[1] > 220)
      {
        MeterServoState += 10 ;
      }
      else if (buf[1] < 50)
      {
        MeterServoState -= 10 ;
      }
      MeterServoState = constrain(MeterServoState, 1300, 1950);
      //      Serial.println(MeterServoState);

      ServoMeter.attach(servoPin);
      ServoMeter.writeMicroseconds(MeterServoState);
      delay(25);
      ServoMeter.detach();


      if (buf[3]) {
        ServoCam.attach(CamServoPin);
        ServoCam.writeMicroseconds(CamServoUp);
        delay(25);
        ServoCam.detach();
      }
      else
      {
        ServoCam.attach(CamServoPin);
        ServoCam.writeMicroseconds(CamServoDown);
        delay(25);
        ServoCam.detach();
      }

      // Send a reply
      //      uint8_t data[] = "g";
      //      rf95.send(data, sizeof(data));
      //      rf95.waitPacketSent();
      //      Serial.println("  Sent a reply");
      //      digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}


