/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SDA 4 // D2
#define SCL 5 // D1

#define SEALEVELPRESSURE_HPA 1013.25


void sendDiscovery();

// Update these with values suitable for your network.

const char* ssid = "oakridge";
const char* password = "donkeykong";
const char* mqtt_server = "server.home";
const char* topic_prefix = "nick";

Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	128
char msg[MSG_BUFFER_SIZE];

#define TOPIC_SIZE 64
char topic[TOPIC_SIZE];
unsigned long int delayTime;
String hostname;

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

  hostname = WiFi.hostname();
  Serial.print("Hostname: ");
  Serial.println(hostname);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
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
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      sendDiscovery();
      snprintf(topic, TOPIC_SIZE, "%s/%s/state", topic_prefix, hostname.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup_bme() {
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!!!");
    delayTime = 60000;
    return;
  }

    Serial.println("-- General Sensing Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling");
    Serial.println("= filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
                      
    delayTime = 60000;  // in milliseconds
}

void scanWire() {
  Wire.begin(SDA, SCL);
  Serial.println("Scanning bus");
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device: (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      delay(1);
    }
  }
  Serial.println("Scanned");
  delay(1000);
}

void sendDiscoveryMessage(const char* buffer, const char* to) {
  Serial.println(buffer);

  snprintf(topic, TOPIC_SIZE, "homeassistant/sensor/%s/%s/config", hostname.c_str(), to);
  boolean result = client.publish(topic, buffer, true);
  if (!result) {
    Serial.println("Failed");
  }

}

#define DISCOVERY_BUFFER_SIZE 1024
void sendDiscovery() {
  Serial.println("Sending discovery");

  
  char* buffer = (char*)malloc(DISCOVERY_BUFFER_SIZE);
  if (buffer == NULL) {
    Serial.println("Failed to allocate memory");
    return;
  }

  client.setBufferSize(DISCOVERY_BUFFER_SIZE + 64);

  const char* hostname_str = hostname.c_str();

  snprintf(buffer, DISCOVERY_BUFFER_SIZE,
    "{\"name\": \"%s_t\", \"uniq_id\": \"%s_t\", \"dev_cla\": \"temperature\", \"stat_t\": \"%s/%s/state\", \"unit_of_meas\": \"°C\", \"val_tpl\": \"{{ value_json.temp }}\", "
    "\"dev\": { \"ids\": [ \"%s\"], \"name\": \"%s\", \"mf\":\"Nick\", \"mdl\": \"ESP Sensor\"}}",
    hostname_str, hostname_str, topic_prefix, hostname_str, hostname_str, hostname_str);

  sendDiscoveryMessage(buffer, "temperature");

  snprintf(buffer, DISCOVERY_BUFFER_SIZE,
    "{\"name\": \"%s_p\", \"uniq_id\": \"%s_p\", \"dev_cla\": \"pressure\", \"stat_t\": \"%s/%s/state\", \"unit_of_meas\": \"hPa\", \"val_tpl\": \"{{ value_json.press }}\", "
    "\"dev\": { \"ids\": [ \"%s\"], \"name\": \"%s\", \"mf\":\"Nick\", \"mdl\": \"ESP Sensor\"}}",
    hostname_str, hostname_str, topic_prefix, hostname_str, hostname_str, hostname_str);

  sendDiscoveryMessage(buffer, "pressure");

  snprintf(buffer, DISCOVERY_BUFFER_SIZE,
    "{\"name\": \"%s_h\", \"uniq_id\": \"%s_h\", \"dev_cla\": \"humidity\", \"stat_t\": \"%s/%s/state\", \"unit_of_meas\": \"%%\", \"val_tpl\": \"{{ value_json.hum }}\", "
    "\"dev\": { \"ids\": [ \"%s\"], \"name\": \"%s\", \"mf\":\"Nick\", \"mdl\": \"ESP Sensor\"}}",
    hostname_str, hostname_str, topic_prefix, hostname_str, hostname_str, hostname_str);

  sendDiscoveryMessage(buffer, "humidity");

  free(buffer);

  client.setBufferSize(MQTT_MAX_PACKET_SIZE);

  Serial.println("Sent discovery");

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  delay(1000);
  scanWire();
  setup_bme();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void takeAndSendReading() {
  bme.takeForcedMeasurement();
  float temp = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0f;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float humidity = bme.readHumidity();

  snprintf(msg, MSG_BUFFER_SIZE, "{\"host\": \"%s\", \"temp\":%f, \"press\": %f, \"alt\": %f, \"hum\":%f}", hostname.c_str(), temp, pressure, altitude, humidity );
  Serial.print("Send to ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);
  client.publish(topic, msg);
}

void loop() {

  if (!client.connected()) {
    reconnect();
    lastMsg = 0;
  }
  client.loop();

  unsigned long now = millis();

  if ((lastMsg == 0) || (now - lastMsg > delayTime)) {
    lastMsg = now;
    takeAndSendReading();
    // ++value;

    // snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    // Serial.print("Publish message: ");
    // Serial.println(msg);
    // client.publish("test/outTopic", msg);
  }
}