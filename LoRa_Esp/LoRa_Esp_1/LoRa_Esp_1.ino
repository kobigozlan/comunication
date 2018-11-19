#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <WebSockets.h>
#include <WiFi.h>
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// Pin definetion of WIFI LoRa 32
// HelTec AutoMation 2017 support@heltec.cn
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true
String incomingLoRa = "";
String sendRS = "";

WebSocketsServer webSocket = WebSocketsServer(80);

const char *ssid = "LoRa_Esp_1";
const char *password = "91259125";
unsigned char clientConnected = 0;
IPAddress apIP(192, 168, 4, 1);
bool SendLoraEsp = false;
bool SendLoraEaco = false;
bool SendLoraEspPing = false;

void connectWifi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00
  WiFi.softAP(ssid, password, 6, 0); //, 1); // no pass, ch=6, ssid=broadcast, max_conns=1
}

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t* src = (const uint8_t*)mem;
  Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for (uint32_t i = 0; i < len; i++) {
    if (i % cols == 0) {
      Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    Serial.printf("%02X ", *src);
    src++;
  }
  Serial.printf("\n");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        // send message to client
        String numS = String(num);
        String msg = String("Connected with ID:" + numS);
        webSocket.sendTXT(num, msg);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      sendMessageLora(payload, length);
      //      webSocket.sendTXT(num, "message here");  // send message to client
      //      webSocket.broadcastTXT("brodcast message here");  // send data to all connected clients
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\n", num, length);
      hexdump(payload, length);
      // webSocket.sendBIN(num, payload, length); // send message to client
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

void sendMessageLora(uint8_t * outgoing, size_t length)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(outgoing, length);  // add payload
  LoRa.endPacket();              // finish packet and send it
  LoRa.receive();
}

void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  connectWifi();
  Serial.println(WiFi.softAPIP());
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI00); // set CS, reset, IRQ pins

  if (!LoRa.begin(BAND, PABOOST))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa init succeeded.");
}

void loop() {
  webSocket.loop();
  if (SendLoraEsp) {
    SendLoraEsp = false;
    webSocket.broadcastTXT(incomingLoRa);
  }
  if (SendLoraEspPing) {
    SendLoraEspPing = false;
    webSocket.broadcastTXT(incomingLoRa);
    String incomingLoRa = "";
  }
  if (SendLoraEaco) {
    SendLoraEaco = false;
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    LoRa.beginPacket();
    LoRa.print("ping");
    LoRa.print(",");
    LoRa.print(rssi);
    LoRa.print(",");
    LoRa.print(snr);
    LoRa.endPacket();
    LoRa.receive();
  }
}

void onReceive(int packetSize)
{
  if (packetSize == 0) return;          // if there's no packet, return
  incomingLoRa = "";
  while (LoRa.available())             // can't use readString() in callback
  {
    incomingLoRa += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLoRa == "_echo_") {
    SendLoraEaco = true;
  } else if (incomingLoRa.substring(0, 4) == "ping") {
    SendLoraEspPing = true;
    } else {
    SendLoraEsp = true;
  }
  Serial.println("Message: " + incomingLoRa + " size " + incomingLoRa.length());
  Serial.print("RSSI: ");
  Serial.println(LoRa.packetRssi());
  Serial.print("Snr: ");
  Serial.println(LoRa.packetSnr());
  Serial.println();
}
