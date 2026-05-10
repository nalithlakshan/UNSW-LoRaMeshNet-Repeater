#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const char* ssid = "YN-WiFi";
const char* password = "dimobatta1";

const char* mqttServer = "7a95f954fa024e02b2abee411a96e857.s1.eu.hivemq.cloud";
const int mqttPort = 8883;

const char* mqttUser = "nalithlakshan";
const char* mqttPass = "Nalith96";

const char* publishTopic = "BBB";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

char clientID[32];

String serialData = "";

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(1000);

  Serial.println("Starting...");

  connectWiFi();

  snprintf(clientID, sizeof(clientID), "ESP01-%06X", ESP.getChipId());
  Serial.printf("Client ID: %s\n", clientID);

  espClient.setInsecure();

  mqttClient.setServer(mqttServer, mqttPort);
  connectMqtt();
}

void loop() {
  if (!mqttClient.connected()) {
    connectMqtt();
  }

  mqttClient.loop();

  if (Serial.available() > 0) {
    serialData += Serial.readString();
    serialData.trim();

    if (serialData.length() > 0) {
      mqttClient.publish(publishTopic, serialData.c_str());

      Serial.print("Published: ");
      Serial.print(serialData);
      Serial.print(" to topic ");
      Serial.println(publishTopic);

      serialData = "";
    }
  }

  delay(1000);
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void connectMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect(clientID, mqttUser, mqttPass)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(mqttClient.state());
      delay(3000);
    }
  }
}