#include <IridiumSBD.h>
#include <TinyGPS++.h>

IridiumSBD isbd(Serial1, 6);

static const int ledPin = 13;

int err;
int signalQuality = -1;
char sendString [60];

float lat,lon,alt;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);


void setup()
{

	pinMode(ledPin, OUTPUT);

	Serial.begin(115200);
	delay(1000);
	Serial1.begin(19200);
	delay(1000);
	Serial2.begin(9600);
	delay(1000);
	
	Serial.println("start");

	isbd.attachConsole(Serial);
	isbd.setPowerProfile(1);
	isbd.begin();
	


	
}

void loop()
{
		Serial.println("sendString");
			
			err = isbd.getSignalQuality(signalQuality);
			if (err != 0)
			{
				Serial.print("SignalQuality failed: error ");
				Serial.println(err);
				return;
			}
			
			sprintf(sendString,"%d%d%d",(int)(lat*1000000),(int)(lon*1000000),(int)alt);
			Serial.println(sendString);
			
			err = isbd.sendSBDText(sendString);
			if (err != 0)
			{
				Serial.print("sendSBDText failed: error ");
				Serial.println(err);
				return;
			}
	sprintf(sendString,"");
	
	while(true){}
		
}

bool ISBDCallback()
{
	digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
	
	if (Serial2.available() > 0){
		gps.encode(Serial2.read());
	}
	
	 if (gps.location.isValid())
	 {
		 lat=gps.location.lat();
		 lon=gps.location.lng();
	 }
	 if(gps.altitude.isValid()){
	  alt=gps.altitude.meters();
	 }
	return true;
}
