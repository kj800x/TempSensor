// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
#include "DHT.h"

#include <FS.h>                   // this needs to be first, or it all crashes and burns...
#include "SPIFFS.h"
#include <DNSServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager 2.0.16-rc.2
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson 6.21.2
#include <SPI.h>
#include <ATM90E32.h>             // https://github.com/CircuitSetup/ATM90E32 6e3c9c6
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient 2.8

#define FULL_RESET false
#define FORMAT_SPIFFS_IF_FAILED true
#define CONFIG_WIFI_PASSWORD "password"
#define DHTTYPE DHT22

#define CALIBRATION_SCALE 1000
#define CALIBRATION_SCALE_S "1000"

//define your default values here, if there are different values in config.json, they are overwritten.
char room_name[40] = "Roomname";
char mqtt_server[40] = "10.60.1.15";
char mqtt_port[6] = "1883";
char dht_pin[6] = "4";
char temperature_calibration[6] = CALIBRATION_SCALE_S;
char humidity_calibration[6] = CALIBRATION_SCALE_S;
int t_calib_raw = CALIBRATION_SCALE;
int h_calib_raw = CALIBRATION_SCALE;
float t_calib = 1.0f;
float h_calib = 1.0f;

void mqtt_callback(char* topic, byte* payload, unsigned int length);

DHT *dht;

WiFiClient esp_wifi_client;
PubSubClient mqtt_client(esp_wifi_client);
unsigned long last_message_publish = 0;

//flag for saving data
bool shouldSaveConfig = false;

//topics to publish
const char* temperature_topic;
const char* humidity_topic;
const char* h_calib_topic;
const char* t_calib_topic;
const char* reset_topic;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  #if FULL_RESET
    //clean FS, for testing
    SPIFFS.format();
  #endif

  Serial.println("Reading config from FileSystem");
  readConfigsFromFileSystem();
  Serial.println("Setting up Wifi");
  setupWiFi();
  Serial.println("Configuring variables");
  configureVariables();
  Serial.println("Setting up Sensors");
  initializeSensors();
  Serial.println("Setting up MQTT Client");
  initializeMQTTClient();
}

void initializeMQTTClient() {
  Serial.printf("MQTT Server: %s, port: %d\n", mqtt_server, atoi(mqtt_port));
  mqtt_client.setServer(mqtt_server, atoi(mqtt_port));
  mqtt_client.setCallback(mqtt_callback);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if(strcmp(topic, reset_topic) == 0) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.println("] ");
    WiFi.disconnect(true);
    ESP.restart();
    delay(5000);
  }
  if(strcmp(topic, h_calib_topic) == 0) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = '\0';
    if (atoi(p) != h_calib_raw) {
      Serial.print("Message arrived [");
      Serial.print(topic);
      Serial.println("] ");
      h_calib = atof(p) / CALIBRATION_SCALE;
      h_calib_raw = atoi(p);
      Serial.print("Updating h_calib to ");
      Serial.println(h_calib_raw);
      strcpy(humidity_calibration, String(h_calib_raw).c_str());
      writeConfig();
    }
  }
  if(strcmp(topic, t_calib_topic) == 0) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = '\0';
    if (atoi(p) != t_calib_raw) {
      Serial.print("Message arrived [");
      Serial.print(topic);
      Serial.println("] ");
      t_calib = atof(p) / CALIBRATION_SCALE;
      t_calib_raw = atoi(p);
      Serial.print("Updating t_calib to ");
      Serial.println(t_calib_raw);
      strcpy(temperature_calibration, String(t_calib_raw).c_str());
      writeConfig();
    }
  }
}

void writeConfig() {
  Serial.println("saving config");
  DynamicJsonDocument jsonBuffer(10000);
  JsonObject json = jsonBuffer.to<JsonObject>();
  json["room_name"] = room_name;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["dht_pin"] = dht_pin;
  json["temperature_calibration"] = temperature_calibration;
  json["humidity_calibration"] = humidity_calibration;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(json, Serial);
  Serial.println("");
  serializeJson(json, configFile);
  configFile.close();
  //end save
}

void setupWiFi() {
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_room_name("room_name", "room name", room_name, 40);
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_dht_pin("dht_pin", "dht_pin", dht_pin, 5);
  WiFiManagerParameter custom_temperature_calibration("temperature_calibration", "Temperature Calibration", temperature_calibration, 6);
  WiFiManagerParameter custom_humidity_calibration("humidity_calibration", "Humidity Calibration", humidity_calibration, 6);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_room_name);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_dht_pin);
  wifiManager.addParameter(&custom_temperature_calibration);
  wifiManager.addParameter(&custom_humidity_calibration);

  #if FULL_RESET
    //reset settings - for testing
    wifiManager.resetSettings();
    Serial.println("FULL_RESET = true.\nReset is complete.\nReflash with FULL_RESET = false to proceed.");
    delay(60000);
    ESP.restart();
    delay(60000);
    return;
  #endif

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("New-ESP32-Env-Probe", CONFIG_WIFI_PASSWORD)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(room_name, custom_room_name.getValue());
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(dht_pin, custom_dht_pin.getValue());
  strcpy(temperature_calibration, custom_temperature_calibration.getValue());
  strcpy(humidity_calibration, custom_humidity_calibration.getValue());


  //save the custom parameters to FS
  if (shouldSaveConfig) {
    writeConfig();
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
}

void readConfigsFromFileSystem() {
  Serial.println("mounting FS...");

  if (SPIFFS.begin(true)) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument jsonBuffer(10000);
        DeserializationError error = deserializeJson(jsonBuffer, buf.get());
        JsonObject json = jsonBuffer.as<JsonObject>();
        serializeJson(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");
          strcpy(room_name, json["room_name"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(dht_pin, json["dht_pin"]);
          strcpy(temperature_calibration, json["temperature_calibration"]);
          strcpy(humidity_calibration, json["humidity_calibration"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}

void configureVariables() {
  t_calib_raw = atoi(temperature_calibration);
  h_calib_raw = atoi(humidity_calibration);
  t_calib = atof(temperature_calibration) / CALIBRATION_SCALE;
  h_calib = atof(humidity_calibration) / CALIBRATION_SCALE;

  String room_name_string = String(room_name);
  humidity_topic = (new String("ESP32Env/" + room_name_string + "/humidity"))->c_str();
  temperature_topic = (new String("ESP32Env/" + room_name_string + "/temperature"))->c_str();
  h_calib_topic = (new String("ESP32Env/" + room_name_string + "/calibration/humidity"))->c_str();
  t_calib_topic = (new String("ESP32Env/" + room_name_string + "/calibration/temperature"))->c_str();
  reset_topic = (new String("ESP32Env/" + room_name_string + "/reset"))->c_str();
}

void initializeSensors() {  
  dht = new DHT(atoi(dht_pin), DHTTYPE);
  
  delay(1000);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = String("ESP32Env/" + String(room_name));
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str())) {
      Serial.println("connected");
      Serial.print("Reset topic is = ");
      Serial.println(reset_topic);
      mqtt_client.subscribe(reset_topic);
      mqtt_client.subscribe(h_calib_topic);
      mqtt_client.subscribe(t_calib_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqtt_loop() {
  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();
}

void loop() {
  unsigned long now = millis();
  mqtt_loop();
  if (now - last_message_publish > 5000) {
    float h = dht->readHumidity() * h_calib;
    float t = dht->readTemperature() * t_calib; // in C

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.println(F("°C"));

    mqtt_client.publish(temperature_topic, String(t).c_str());
    mqtt_client.publish(humidity_topic, String(h).c_str());
    mqtt_client.publish(h_calib_topic, String(h_calib_raw).c_str());
    mqtt_client.publish(t_calib_topic, String(t_calib_raw).c_str());
    last_message_publish = now;
  }
}
