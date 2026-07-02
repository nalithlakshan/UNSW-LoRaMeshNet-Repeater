#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const char* ssid1 = "UNSW-IoT";
const char* password1 = "mKGLxytpM93Kf5p6nQ==";

const char* ssid2 = "YN-WiFi";
const char* password2 = "dimobatta1";

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

  Serial.print("STA MAC Address: ");
  Serial.println(WiFi.macAddress());

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

bool tryConnectWiFi(const char* ssid, const char* password, unsigned long timeoutMs) {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.disconnect();
  delay(500);

  WiFi.begin(ssid, password);

  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeoutMs) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  return WiFi.status() == WL_CONNECTED;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);

  if (!tryConnectWiFi(ssid1, password1, 10000)) {
    Serial.println("Failed to connect to UNSW-IoT. Trying YN-WiFi...");

    if (!tryConnectWiFi(ssid2, password2, 10000)) {
      Serial.println("Failed to connect to both WiFi networks.");

      while (WiFi.status() != WL_CONNECTED) 
      {
        delay(1000);
        Serial.println("Retrying...");
        if (tryConnectWiFi(ssid1, password1, 10000)) {
          break;
        }
        if (tryConnectWiFi(ssid2, password2, 10000)) {
          break;
        }
      }
    }
  }

  Serial.println("WiFi connected");
  Serial.print("Connected SSID: ");
  Serial.println(WiFi.SSID());
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