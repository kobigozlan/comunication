/*
  Board	          int.0	  int.1	  int.2	  int.3	  int.4	  int.5
  Uno, Ethernet	  2	  3
  Mega2560	  2	  3	  21	  20	  19	  18
  Leonardo	  3	  2	  0	  1
  Due	          (any pin, more info http://arduino.cc/en/Reference/AttachInterrupt)
*/

// This wrapper is in charge of calling
// mus be defined like this for the lib work
void dht11_wrapper() {
  DHT11.isrCallback();
}

void idDHT11_loop()
{
  //  Serial.print("\nRetrieving information from sensor: ");
  //  Serial.print("Read sensor: ");
  //delay(100);
  DHT11.acquire();
  
  uint64_t t1 = millis();
  while (DHT11.acquiring() && (millis() - t1 < 3000))
    ;

  int result = DHT11.getStatus();
  switch (result)
  {
    case IDDHTLIB_OK:
//      Serial.println("OK");
      break;
    case IDDHTLIB_ERROR_CHECKSUM:
//      Serial.println("Error\n\r\tChecksum error");
      break;
    case IDDHTLIB_ERROR_ISR_TIMEOUT:
//      Serial.println("Error\n\r\tISR Time out error");
      break;
    case IDDHTLIB_ERROR_RESPONSE_TIMEOUT:
//      Serial.println("Error\n\r\tResponse time out error");
      break;
    case IDDHTLIB_ERROR_DATA_TIMEOUT:
//      Serial.println("Error\n\r\tData time out error");
      break;
    case IDDHTLIB_ERROR_ACQUIRING:
//      Serial.println("Error\n\r\tAcquiring");
      break;
    case IDDHTLIB_ERROR_DELTA:
//      Serial.println("Error\n\r\tDelta time to small");
      break;
    case IDDHTLIB_ERROR_NOTSTARTED:
//      Serial.println("Error\n\r\tNot started");
      break;
    default:
//      Serial.println("Unknown error");
      break;
  }
  //  Serial.print("Humidity (%): ");
  //  Serial.println(DHT11.getHumidity(), 2);
  //
  //  Serial.print("Temperature (oC): ");
  //  Serial.println(DHT11.getCelsius(), 2);
  //
  //  Serial.print("Temperature (oF): ");
  //  Serial.println(DHT11.getFahrenheit(), 2);
  //
  //  Serial.print("Temperature (K): ");
  //  Serial.println(DHT11.getKelvin(), 2);
  //
  //  Serial.print("Dew Point (oC): ");
  //  Serial.println(DHT11.getDewPoint());
  //
  //  Serial.print("Dew Point Slow (oC): ");
  //  Serial.println(DHT11.getDewPointSlow());

  //  delay(2000);
}

