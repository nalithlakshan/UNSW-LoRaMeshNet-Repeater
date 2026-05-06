#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// const char* ssid = "YN-WiFi";
// const char* password = "dimobatta1";

extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
}

const char* ssid = "eduroam";

// Eduroam requires WPA2-Enterprise credentials. Use your full realm username,
// for example your UNSW zID in the format required by UNSW IT.
const char* eapIdentity = "z5440292";
const char* eapUsername = "z5440292";
const char* eapPassword = "HarveySpector@1";

const char* mqttServer = "7a95f954fa024e02b2abee411a96e857.s1.eu.hivemq.cloud";
const int mqttPort = 8883;

const char* mqttUser = "nalithlakshan";
const char* mqttPass = "Nalith96";

const char* publishTopic = "BBB";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

char clientID[32];

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting...");

  connectWiFi();

  snprintf(clientID, sizeof(clientID), "ESP01-%06X", ESP.getChipId());
  Serial.printf("Client ID: %s\n", clientID);

  // For quick testing only: disables certificate validation
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
    String serialData = "";

    while (Serial.available() > 0){
      serialData += Serial.readString();
      delay(2);
    }

    serialData.trim();

    if (serialData.length() > 0) {
      mqttClient.publish(publishTopic, serialData.c_str());

      Serial.print("Published: ");
      Serial.print(serialData);
      Serial.print(" to topic ");
      Serial.println(publishTopic);
    }
  }

  delay(1000);
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");

  WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(1000);
  wifi_station_disconnect();
  wifi_station_clear_cert_key();
  wifi_station_clear_enterprise_ca_cert();
  wifi_station_set_wpa2_enterprise_auth(1);
  wifi_station_set_enterprise_identity((uint8*)eapIdentity, strlen(eapIdentity));
  wifi_station_set_enterprise_username((uint8*)eapUsername, strlen(eapUsername));
  wifi_station_set_enterprise_password((uint8*)eapPassword, strlen(eapPassword));
  WiFi.begin(ssid);

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
