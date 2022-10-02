#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>



// Choose two Arduino pins to use for software serial
int RXPin = 12;
int TXPin = 14;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);




#define SEALEVELPRESSURE_HPA (1013.25)

const uint8_t sensorAddress = 0x77;
Adafruit_BME280 bme;


//WLAN
const char* ssid = "WLAN-237497";
const char* password =  "51806912150269801851";

//Broker
const char* mqttServer = "192.168.2.131";
const int mqttPort = 1883;
const char* mqttUser = "admin1";
const char* mqttPassword = "admin1";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (100)
char msg[100];
char telemetria[100];
int value = 0;
float lat = 48.126503,lon = 11.376705;



void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect("",mqttUser,mqttPassword)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("hello", "hello world");
      // ... and resubscribe
      client.subscribe("command");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  Serial.begin(9600);

  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);


  // default settings
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
  
}

void loop() {
  /*
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
*/
  
  
  unsigned long now = millis();
  if (now - lastMsg > 300000) {
    lastMsg = now;
    ++value;

    float temperature = bme.readTemperature();
    //temperature = 9.0/5.0 * temperature + 32.0;
    float pressure = bme.readPressure();
    float humidity = bme.readHumidity();
    
    
    snprintf (msg, MSG_BUFFER_SIZE, "Temperatura #%lf Humedad #%1f  Presion #%1f", temperature, humidity, pressure);
    Serial.println(msg);
    //client.publish("temp", msg,true);


   
    Serial.println(temperature);
    Serial.println(pressure / 100.0F);
    Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(humidity);

    delay(1000);

    // Initialize the client library
    //HttpClient client;
    
    // Make a HTTP request:
    sprintf (telemetria, "tem=%f&pre=%f&alt=%f&hum=%f&lat=%f&lon=%f",temperature,(pressure / 100.0F),bme.readAltitude(SEALEVELPRESSURE_HPA),humidity,lat,lon);

    HTTPClient http;  //Declare an object of class HTTPClient
    
    http.begin("http://aluna.com.mx/wsTelemetria/telemetria.php?"+String(telemetria));  //Specify request destination
    int httpCode = http.GET();                                                                  //Send the request
    Serial.println("HTTP code:"+String(httpCode));                     //Print the response payload
   
    if (httpCode != 200) { //Check the returning code
    
      String payload = http.getString();   //Get the request response payload
      Serial.println(payload);                     //Print the response payload
   
    }
    
    http.end();   //Close connection


    delay(100);
    while (gpsSerial.available() > 0)
    {
      if (gps.encode(gpsSerial.read()))
        displayInfo();
    }
    // If 5000 milliseconds pass and there are no characters coming in
    // over the software serial port, show a "No GPS detected" error
    //if (millis() > 5000 && gps.charsProcessed() < 10)
    //{
    //  Serial.println("No GPS detected");
    //  while(true);
    //}
   
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    lat = gps.location.lat();
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    lon = gps.location.lng();
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(50);

}
