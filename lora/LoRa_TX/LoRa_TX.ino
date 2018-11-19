/*
**************************************************************************************************
Arduino LoRa Transmit Programs

Copyright of the author Stuart Robinson - 25/04/2015 09:00
**************************************************************************************************
*/

#include <SPI.h> // include the SPI library:
#include "Lora_Library.h"            // include the LoRa constants, variables and functions

void setup()
{
	delay(1000);
	
	//initialise the program
	Serial.begin(9600);			// init serial port for send and receive at 9600 baud
	Serial.println("Program_Setup");
	pinMode(lora_PReset, OUTPUT);		// RFM98 reset line
	digitalWrite(lora_PReset, LOW);	// Reset RFM98
	pinMode (lora_PNSS, OUTPUT);		// set the slaveSelectPin as an output:
	digitalWrite(lora_PNSS, HIGH);
	pinMode(lora_PLED1, OUTPUT);		// for shield LED
	digitalWrite(lora_PLED1, LOW);
	
	SPI.begin();				// initialize SPI:
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	Serial.print("LoRa TX start");;
	LED1Flash(4, 50, 200);		//Flash the LED
	lora_ResetDev();			//Reset the device
	lora_Setup();				//Do the initial LoRa Setup
	lora_SetFreq(434.400);		//Set the LoRa
	
	Serial.print("Programmed Frequency ");
	Serial.println(lora_GetFreq());
	
	lora_Print();			        //Print the LoRa registers
	lora_Tone(1000, 1000, 5);             //Transmit an FM tone
	lora_SetModem(lora_BW41_7, lora_SF8, lora_CR4_5, lora_Explicit, lora_LowDoptOFF);		//Setup the LoRa modem parameters
	lora_PrintModem();                    //Print the modem parameters
	lora_FillTX();			//Fill TXbuff with test data
	lora_TXBuffPrint(0);			//Print the TX buffer as ASCII to screen
	delay(3000);
}

void loop()
{
	lora_Send(lora_TXStart, lora_TXEnd, 32, 255, 1, 10, 2);	//send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
	lora_TXPKTInfo();			//print packet information
	delay(1000);
}



