#include <ESP8266WiFi.h>
#include <SPI.h>



const char* ssid = "WLAN-237497";
const char* password =  "51806912150269801851";
const char* mqttServer = "192.168.2.124";
const int mqttPort = 5672;
const char* mqttUser = "admin";
const char* mqttPassword = "admin";


int status = WL_IDLE_STATUS;
IPAddress server(192,168,2,124);  // Google

 
WiFiClient client;

 
void setup() {
 
  Serial.begin(9600);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("Local IP: " + WiFi.localIP());

 
  if (client.connect(server, 80)) {
      Serial.println("connected");
      // Make a HTTP request:
      client.println("GET /search?q=arduino HTTP/1.0");
      client.println();
    }
 
}
 

 
void loop() {

    if (client.connect(server, 80)) {
      Serial.println("connected");
      // Make a HTTP request:
      client.println("GET /search?q=arduino HTTP/1.0");
      client.println();
    }
    else
    {
      Serial.println("Client not connected");
    }
          delay(2000);

  
}
