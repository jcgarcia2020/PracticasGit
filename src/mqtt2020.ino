/****************************************************************************************************************************
   AutoConnectWithFSParameters.ino
   For ESP8266 / ESP32 boards

   ESP_WiFiManager is a library for the ESP8266/ESP32 platform (https://github.com/esp8266/Arduino) to enable easy
   configuration and reconfiguration of WiFi credentials using a Captive Portal.

   Forked from Tzapu https://github.com/tzapu/WiFiManager
   and from Ken Taylor https://github.com/kentaylor

   Built by Khoi Hoang https://github.com/khoih-prog/ESP_WiFiManager
   Licensed under MIT license
   Version: 1.0.7

   Version Modified By   Date       Comments
   ------- -----------  ----------- ----------------------
    1.0.0   jcgarcia     5/15/2020   se incia el proyecto
    1.0.0   jcgarcia     07/22/2020  se esta implementando OTA
    command borrar flash pio run -t erase

 *****************************************************************************************************************************/

//Ported to ESP32
#ifdef ESP32
#include <FS.h>
#include "SPIFFS.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_BUILTIN       2
#define LED_ON            HIGH
#define LED_OFF           LOW

#else
#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
// LIBRARY OTA
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define ESP_getChipId()   (ESP.getChipId())

#define LED_ON      LOW
#define LED_OFF     HIGH
#endif

// Pin D2 mapped to pin GPIO2/ADC12 of ESP32, or GPIO2/TXD1 of NodeMCU control on-board LED
#define LED_BUILTIN   16
#define PIN_LED       LED_BUILTIN

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

// Now support ArduinoJson 6.0.0+ ( tested with v6.14.1 )
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// variable con el nombre del archivo de configuraciones
char configFileName[] = "/config.json";
char configEstatus[]  = "/estatus.json";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// SSID and PW for Config Portal
String AP_SSID;
String AP_PASS;

//for LED status
#include <Ticker.h>
Ticker ticker;

//Librerias necesarias para la comunicacion mqtt
#include <PubSubClient.h>
// LED CONECTADO A GPIO2 DEL ESP8226 MARCADO COMO PIN D4
#define PIN_LED_ACTION 2
String inString = "";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//define your default values here, if there are different values in configFileName (config.json), they are overwritten.
#define MQTT_SERVER_MAX_LEN             40
#define MQTT_PUBLISH_MAX_LEN            40
#define MQTT_SUBSCRIBE_MAX_LEN          40
#define MQTT_SERVER_PORT_LEN            6

char mqtt_topicPublish [MQTT_PUBLISH_MAX_LEN] = "newhouse/cuarto/lampara";    // TopicPublish
char mqtt_Subscribe [MQTT_SUBSCRIBE_MAX_LEN]  = "newhouse/cuarto/lampara";    // topicSubscribe
char mqtt_server    [MQTT_SERVER_MAX_LEN]     = "broker.hivemq.com";          // Broker Server
char mqtt_port      [MQTT_SERVER_PORT_LEN]    = "1883";                       // Port mqtt
int mapeo = 0;
int Power = 0;
int Brightness = 0;
  
//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback(void)
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//monta el sistema de archivos para el esp8266
bool loadSPIFFSConfigFile(void)
{
  //clean FS, for testing
  //SPIFFS.format();
  //read configuration from FS json
  Serial.println("Mounting FS...");
  if (SPIFFS.begin())
  {
    Serial.println("Mounted file system");

    if (SPIFFS.exists(configFileName))
    {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open(configFileName, "r");

      if (configFile)
      {
        Serial.print("Opened config file, size = ");
        size_t configFileSize = configFile.size();
        Serial.println(configFileSize);

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);
        configFile.readBytes(buf.get(), configFileSize);
        Serial.print("\nJSON parseObject() result : ");

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);
        if ( deserializeError )
        {
          Serial.println("failed");
          return false;
        }
        else
        {
          Serial.println("OK");

          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));

          if (json["mqtt_topicPublish"])
            strncpy(mqtt_topicPublish, json["mqtt_topicPublish"], sizeof(mqtt_topicPublish));

          if (json["mqtt_Subscribe"])
            strncpy(mqtt_Subscribe, json["mqtt_Subscribe"], sizeof(mqtt_Subscribe));
        }

        //serializeJson(json, Serial);
        serializeJsonPretty(json, Serial);
#else
        DynamicJsonBuffer jsonBuffer;
        // Parse JSON string
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        // Test if parsing succeeds.

        if (json.success())
        {
          Serial.println("OK");
          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));

          if (json["mqtt_topicPublish"])
            strncpy(mqtt_topicPublish, json["mqtt_topicPublish"], sizeof(mqtt_topicPublish));

          if (json["mqtt_Subscribe"])
            strncpy(mqtt_Subscribe, json["mqtt_Subscribe"], sizeof(mqtt_Subscribe));
        }
        else
        {
          Serial.println("failed");
          return false;
        }
        //json.printTo(Serial);
        json.prettyPrintTo(Serial);
#endif
        configFile.close();
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
    return false;
  }
  return true;
}

bool saveSPIFFSConfigFile(void)
{
  Serial.println("Saving config");

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"]   = mqtt_port;
  json["mqtt_topicPublish"]  = mqtt_topicPublish;
  json["mqtt_Subscribe"]  = mqtt_Subscribe;
  File configFile = SPIFFS.open(configFileName, "w");
  if (!configFile)
  {
    Serial.println("Failed to open config file for writing");
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  //serializeJson(json, Serial);
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, configFile);
#else
  //json.printTo(Serial);
  json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(configFile);
#endif

  configFile.close();
  //end save
}

void heartBeatPrint(void)
{
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("conectado a WiFi");              // H means connected to WiFi
  else
    Serial.println("no se conectado a WiFi");        // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(" ");
  }
}

void toggleLED()
{
  //toggle state
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong LEDstatus_timeout    = 0;
  static ulong currentMillis;

#define HEARTBEAT_INTERVAL    10000L
#define LED_INTERVAL          2000L

  currentMillis = millis();

  if ((currentMillis > LEDstatus_timeout) || (LEDstatus_timeout == 0))
  {
    // Toggle LED at LED_INTERVAL = 2s
    toggleLED();
    LEDstatus_timeout = currentMillis + LED_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((currentMillis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint(); // imprime por consola que esta conectado a el wifi
    // controlador de servicio OTA
    ArduinoOTA.handle();
    checkstatus_timeout = currentMillis + HEARTBEAT_INTERVAL;
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\nStarting AutoConnectWithFSParams");

  //set led pin as output
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_LED_ACTION, OUTPUT);
  digitalWrite(PIN_LED_ACTION, HIGH);  //apago el led del gpo2 del nodemecu

  // start ticker with 0.5s because we start in AP mode and try to connect
  ticker.attach(0.5, toggleLED);

  // carga los datos guardados en el sistema de particion del esp8266
  loadSPIFFSConfigFile();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  ESP_WMParameter custom_mqtt_server("mqtt_server",   "mqtt_server", mqtt_server, MQTT_SERVER_MAX_LEN + 1);
  ESP_WMParameter custom_mqtt_port  ("mqtt_port",     "mqtt_port",   mqtt_port,   MQTT_SERVER_PORT_LEN + 1);
  ESP_WMParameter custom_mqtt_topic ("mqtt_topicPublish",    "mqtt_topicPublish", mqtt_topicPublish,  MQTT_PUBLISH_MAX_LEN  + 1);
  ESP_WMParameter custom_mqtt_Subscribe ("mqtt_Subscribe",    "mqtt_Subscribe",  mqtt_Subscribe,  MQTT_SUBSCRIBE_MAX_LEN  + 1);
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  // ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("NewHouse Mqtt2020");

  //set config save notify callback
  ESP_wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  ESP_wifiManager.addParameter(&custom_mqtt_server);
  ESP_wifiManager.addParameter(&custom_mqtt_port);
  ESP_wifiManager.addParameter(&custom_mqtt_topic);
  ESP_wifiManager.addParameter(&custom_mqtt_Subscribe);

  //reset settings - for testing
  //ESP_wifiManager.resetSettings();

  ESP_wifiManager.setDebugOutput(true);

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //ESP_wifiManager.setMinimumSignalQuality();

  //set custom ip for portal
  //ESP_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 100, 1), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));

  //ESP_wifiManager.setMinimumSignalQuality(-1);
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5+
  //ESP_wifiManager.setSTAStaticIPConfig(IPAddress(192, 168, 2, 114), IPAddress(192, 168, 2, 1), IPAddress(255, 255, 255, 0),
  //IPAddress(192, 168, 2, 1), IPAddress(8, 8, 8, 8));

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  //Router_SSID = ESP_wifiManager.WiFi_SSID();
  //Router_Pass = ESP_wifiManager.WiFi_Pass();
  AP_SSID = "NewHouse Beta2020";
  AP_PASS = "admin2020";

  //Remove this line if you do not want to see WiFi password printed
  Serial.println("\nStored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  if (Router_SSID != "")
  {
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    Serial.println("Got stored Credentials. Timeout 120s");
  }
  else
  {
    Serial.println("No stored Credentials. No timeout");
  }

  String chipID = String(ESP_getChipId(), HEX);
  chipID.toUpperCase();

  // SSID and PW for Config Portal
  AP_SSID = "NewHouse Beta2020";
  AP_PASS = "admin2020";

  // Get Router SSID and PASS from EEPROM, then open Config portal AP named "ESP_XXXXXX_AutoConnectAP" and PW "MyESP_XXXXXX"
  // 1) If got stored Credentials, Config portal timeout is 60s
  // 2) If no stored Credentials, stay in Config portal until get WiFi Credentials
  if (!ESP_wifiManager.autoConnect(AP_SSID.c_str(), AP_PASS.c_str()))
  {
    Serial.println("failed to connect and hit timeout");
    // ingresó al modo de configuración, haga que el led cambie más rápido a 0.2s
    ticker.attach(0.2, toggleLED);

    //reset and try again, or maybe put it to deep sleep
#ifdef ESP8266
    ESP.reset();
#else   //ESP32
    ESP.restart();
#endif
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected");

  //read updated parameters
  strncpy(mqtt_server, custom_mqtt_server.getValue(),       sizeof(mqtt_server));
  strncpy(mqtt_port, custom_mqtt_port.getValue(),           sizeof(mqtt_port));
  strncpy(mqtt_topicPublish, custom_mqtt_topic.getValue(),  sizeof(mqtt_topicPublish));
  strncpy(mqtt_Subscribe, custom_mqtt_Subscribe.getValue(), sizeof(mqtt_Subscribe));

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveSPIFFSConfigFile();
  }
  Serial.println("Conectando a " + String(mqtt_server) + String(mqtt_port));
  // Conexion con el broker mqtt
  client.setServer(mqtt_server, String(mqtt_port).toInt());
  //Se establece la rellamada para cuando entre un dato desde el broker
  client.setCallback(DatosEntrantes);
  ticker.detach();
  //keep LED on
  digitalWrite(PIN_LED, LED_ON);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi conectado, iniciando servicio OTA con direccion ip");
    Serial.println(WiFi.localIP());

    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
  }
  else {
    Serial.println("");
    Serial.println("Error de conexion");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // chequea el estado del wifi led flash
  check_status();
  //Inicia operacion el cliente mqtt
  client.loop();
  if (!client.connected()) {
    reconnect();
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Funcion que se encarga de publicar los datos al broker
void publicar () {
  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;
    ++value;
    //snprintf (msg, 50, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println("mensaje de prueba");
    client.publish(mqtt_topicPublish, msg);
  }
}

// Funcion que se encarga de recibir los datos desde el servidor mqtt
void DatosEntrantes(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println(payload[0]);
  // iniciamos la Deserialize del json de node-red
  StaticJsonDocument<200> doc;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, payload);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  Power = doc["state"]["power"];
  Brightness = doc["state"]["brightness"];
  // Print values.
  Serial.println(Power);
  Serial.println(Brightness);

  if (Power == 1) {
    //Serial.println("ENCENDIDO");
    //digitalWrite(PIN_LED_ACTION, LOW);   // Turn the LED on (Note that LOW is the voltage level
    if (Brightness <=0){
      Brightness = 100;
      }
    Brightness = map(Brightness, 0, 100, 1024, 0);
    analogWrite(PIN_LED_ACTION, Brightness);
    Serial.println(Brightness);
  }
  else {
    //Serial.println("APAGADO");
    digitalWrite(PIN_LED_ACTION, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

// Rutina de reconexion al broker
void reconnect() {
  // Bucle hasta que estemos reconectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT... ");
    // Crear un ID de cliente aleatorio
    String clientId = "NewHouseClient-";
    clientId += String(random(0xffff), HEX);
    // Intento de conectar
    if (client.connect(clientId.c_str())) {
      Serial.println(" connected");
      // Una vez conectado, publica un anuncio ...
      client.publish(mqtt_topicPublish, "OK");
      // ... y volver a suscribirte
      client.subscribe(mqtt_Subscribe);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
