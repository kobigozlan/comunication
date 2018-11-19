#include <SPI.h>
#include <RH_RF95.h>
#include <Servo.h>

#define relay 10

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM95_RST     4
#endif

Servo myservo;
uint16_t servo_state = 1500 ;
#define enablePin 10
#define phasePin 11

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM69_CS, RFM69_IRQ);

// Blinky on receipt
#define LED 13

void setup()
{
  pinMode(enablePin, INPUT);
  pinMode(phasePin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  myservo.attach(5);
  myservo.writeMicroseconds(servo_state);

  //  while (!Serial);
  Serial.begin(115200);
  delay(100);

  Serial.println("Feather LoRa RX Test!");

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

  pinMode(relay, OUTPUT);
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
//      RH_RF95::printBuffer("Received: ", buf, len);
      //      Serial.print("Got: ");
      //      Serial.print(buf[0]);
      //      Serial.print(" , ");
      //      Serial.print(buf[1]);
      //      Serial.print(" , ");
      //      Serial.print(buf[2]);
      //      Serial.print("  RSSI: ");
      //      Serial.print(rf95.lastRssi(), DEC);
      if (buf[3])
      {
        digitalWrite(relay, LOW);
      } else {
        digitalWrite(relay, HIGH);
      }

      //      if (buf[0] > 200)
      //      {
      //        digitalWrite(enablePin, HIGH);
      //        digitalWrite(phasePin, LOW);
      //      }
      //      else if (buf[0] < 50)
      //      {
      //        digitalWrite(enablePin, LOW);
      //        digitalWrite(phasePin, HIGH);
      //      }
      //      else
      //      {
      //        digitalWrite(enablePin, LOW);
      //        digitalWrite(phasePin, LOW);
      //      }
      //
      //      if (buf[1] > 200)
      //      {
      //        servo_state += 10 ;
      //      }
      //      else if (buf[1] < 50)
      //      {
      //        servo_state -= 10 ;
      //      }
      //
      //      constrain(servo_state, 1300, 1950);
      //      myservo.writeMicroseconds(servo_state);

      //
      //      Serial.print(" , ");
      //      Serial.println(servo_state);

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


