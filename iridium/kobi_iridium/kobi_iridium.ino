#include <IridiumSBD.h>
#include <SoftwareSerial.h>

//SoftwareSerial nss(2, 3);
IridiumSBD isbd(Serial1, 7);
static const int ledPin = 13;

int err;
int signalQuality = -1;

void setup()
{

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(19200);

  isbd.attachConsole(Serial);
  isbd.setPowerProfile(1);
  isbd.begin();

  delay(3000);
  Serial.println("start ");
  for (int i = 0; i < 2000; i++) {
    err = isbd.getSignalQuality(signalQuality);
    Serial.print(i);
    Serial.print("  ");
    Serial.print(err);
    Serial.print("  Signal quality is ");
    Serial.println(signalQuality);
    if (signalQuality > 2)break;
  }

  //  err = isbd.getSignalQuality(signalQuality);
  //  if (err != 0)
  //  {
  //    Serial.print("SignalQuality failed: error ");
  //    Serial.println(err);
  //    return;
  //  }
  //
  //  Serial.print("Signal quality is ");
  //  Serial.println(signalQuality);

  //  err = isbd.sendSBDText("H");
  //  if (err != 0)
  //  {
  //    Serial.print("sendSBDText failed: error ");
  //    Serial.println(err);
  //    return;
  //  }
  char txb[1] = { };
  uint8_t buffer[20] = { };
  size_t bufferSize = sizeof(buffer);
  err = isbd.sendReceiveSBDText( 0, buffer, bufferSize);
  
  if (err != 0)
  {
    Serial.print("sendReceiveSBDText failed: error ");
    Serial.println(err);
    return;
  }

  Serial.println("Hey, it worked!");
  Serial.print("Messages left: ");
  Serial.println(isbd.getWaitingMessageCount());

  for (int i = 0; i < 10; i++) {
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
    delay (1000);
  }
}

void loop()
{
  digitalWrite(ledPin, HIGH);
}

unsigned long te;
bool ISBDCallback()
{
  digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
  // Serial.println(micros()-te);
  // te=micros();
  return true;
}
