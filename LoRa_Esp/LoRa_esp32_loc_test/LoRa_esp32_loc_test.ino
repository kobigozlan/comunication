#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <SPI.h>
#include <LoRa.h>

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI0     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)
#define BAND    433E6  // 915E6
#define PABOOST true
int counter = 0;
long frequency = 433E6;
int sf = 9; // from 6 to 12
long bw = 62; // the closest number to the BW
int CodingRate = 6; // from 5 to 8
bool SetLora = 0;
char* freqstr[] = {"/f420", "/f425", "/f430", "/f433", "/f435", "/f440", "/f445", "/f450", "/f455", "/f460"};
char* bwstr[] = {"/bw7", "/bw10", "/bw15", "/bw20", "/bw31", "/bw41", "/bw62", "/bw125", "/bw250", "/bw500"};
char* sfstr[] = {"/sf6", "/sf7", "/sf8", "/sf9", "/sf10", "/sf11", "/sf12"};
char* crstr[] = {"/cr4_5", "/cr4_6", "/cr4_7", "/cr4_8"};

// Replace with your network credentials
const char *ssid = "gozlan";
const char* password = "66763616";

//const char *ssid = "LoRa_Esp_Tx";
//const char* ssid = "KCG-ADSL";
//const char* password = "91259125";
IPAddress apIP(192, 168, 4, 1);

// Set LED GPIO
const int ledPin = 25;
// Stores LED state
String ledState;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Replaces placeholder with LED state value
String processor(const String& var) {
  //  Serial.println(var);
  if (var == "STATE") {
    if (digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    Serial.println(ledState);
    return ledState;
  }

  if (var == "FREQ") {
    return String(frequency);
  }
  if (var == "BW") {
    return String(bw);
  }
  if (var == "SF") {
    return String(sf);
  }
  if (var == "CR") {
    SetLora = 1;
    return String(CodingRate);
  }

  return String();
}

void setfreqserver(char* var) {
  server.on(var, HTTP_GET, [var, frequency](AsyncWebServerRequest * request) {
    frequency = atol(var + 2) * 1000000;
    //    Serial.println(frequency);
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
}
void setbwserver(char* var) {
  server.on(var, HTTP_GET, [var, bw](AsyncWebServerRequest * request) {
    bw = atol(var + 3) * 1000;
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
}
void setsfserver(char* var) {
  server.on(var, HTTP_GET, [var, sf](AsyncWebServerRequest * request) {
    sf = atoi(var + 3);
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
}
void setcrserver(char* var) {
  server.on(var, HTTP_GET, [var, CodingRate](AsyncWebServerRequest * request) {
    CodingRate = atoi(var + 5);
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
}
void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi..");
    }
    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());

//  WiFi.disconnect();
//  WiFi.mode(WIFI_AP_STA);
//  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00
//  WiFi.softAP(ssid, password, 5, 0); //, 1); // no pass, ch=6, ssid=broadcast, max_conns=1
//  Serial.println(WiFi.softAPIP());

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest * request) {
    digitalWrite(ledPin, HIGH);
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest * request) {
    digitalWrite(ledPin, LOW);
    request->send(SPIFFS, "/esp32Lora.html", String(), false, processor);
  });

  for (int i = 0; i < 10 ; i++) {
    setfreqserver(freqstr[i]);
  }
  for (int i = 0; i < 10 ; i++) {
    setbwserver(bwstr[i]);
  }
  for (int i = 0; i < 7 ; i++) {
    setsfserver(sfstr[i]);
  }
  for (int i = 0; i < 4 ; i++) {
    setcrserver(crstr[i]);
  }

  // Start server
  server.begin();
}

void loop() {

  if (SetLora) {
    LoRa.sleep();
    LoRa.setSignalBandwidth(bw);
    LoRa.setSpreadingFactor(sf);
    LoRa.setFrequency(frequency);
    LoRa.setCodingRate4(CodingRate);
    LoRa.idle();
    Serial.println(frequency);
    Serial.println(bw);
    Serial.println(sf);
    Serial.println(CodingRate);
    SetLora = 0;
  }

  if (digitalRead(ledPin)) {
    //      ledState = "ON";
    Serial.print("Sending packet: ");
    Serial.println(counter);
    // send packet
    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter);
    LoRa.endPacket();
    counter++;
    delay(5000);
  }
  else {
    //      ledState = "OFF";
    counter = 0;
  }


}
