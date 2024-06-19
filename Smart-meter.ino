#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Broker MQTT
const char* mqtt_server = "rmq2.pptik.id";
const int mqtt_port = 1883;
const char* mqtt_username = "/smart-meter:smart-meter";
const char* mqtt_password = "MkeF65py";
const char* mqtt_clientid = "esp32_client";

// GUID
const char* device_guid = "725a76fd-6d7c-9475-6431-499e432d2d0a";

WiFiClient espClient;
PubSubClient client(espClient);

volatile int flow_frequency; // Mengukur pulsa sensor aliran
float vol = 0.0; // Volume total
unsigned char flowsensor = 34; // Input Sensor
unsigned long currentTime;
unsigned long cloopTime;
float volume_per_pulse = 0.00222;

const char* reset_topic = "Log";
const char* feedback_topic = "feedback";
const char* relay_topic = "Log"; //topic relay jika mau diganti
o
#define EEPROM_SIZE 512
#define VOLUME_ADDR 0
#define CHECKSUM_ADDR (VOLUME_ADDR + sizeof(float))

const int relayPin = 5; // Pin Untuk Relay
const int yellowLedPin = 2; // Pin Untuk LED Kuning
const int greenLedPin = 4; // Pin Untuk LED Hijau
const int buttonPin = 15; // pin untuk tombol reset

bool buttonPressed = false;
unsigned long lastDebounceTime = 0; // Waktu terakhir tombol diubah
unsigned long debounceDelay = 50; // Debounce delay

Preferences preferences;
WiFiManager wifiManager;

void IRAM_ATTR flow() {
  flow_frequency++;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  // Parsing message
  int separatorIndex = message.indexOf('#');
  String receivedDeviceGuid = message.substring(0, separatorIndex);
  String value = message.substring(separatorIndex + 1);

  Serial.print("Received GUID: ");
  Serial.println(receivedDeviceGuid);
  Serial.print("Value: ");
  Serial.println(value);

  if (receivedDeviceGuid == device_guid) {
    if (value == "0") {
      digitalWrite(relayPin, LOW); // Menyalakan relay
      Serial.println("Relay ON");
    } else if (value == "1") {
      digitalWrite(relayPin, HIGH); // Mematikan relay
      Serial.println("Relay OFF");
    } else if (value == "reset data") {
      flow_frequency = 0; // Mereset pembacaan sensor water flow
      vol = 0; // Mereset volume air menjadi 0
      Serial.println("Sensor water flow dan volume reset");
    }
  } else {
    Serial.println("GUID tidak cocok.");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Mencoba menghubungkan ke MQTT...");
    if (client.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
      Serial.println("terhubung");
      client.subscribe(reset_topic);
      client.subscribe(relay_topic); // Subscribe to the new relay control topic
      client.subscribe("Log");
      
      // Blink yellow LED twice
      for (int i = 0; i < 2; i++) {
        digitalWrite(greenLedPin, HIGH);
        delay(500);
        digitalWrite(greenLedPin, LOW);
        delay(500);
      }
    digitalWrite(greenLedPin, HIGH);
    } else {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

uint8_t calculateChecksum(float volume) {
  uint8_t* p = (uint8_t*)&volume;
  uint8_t checksum = 0;
  for (int i = 0; i < sizeof(float); i++) {
    checksum ^= p[i];
  }
  return checksum;
}

void saveVolumeToEEPROM() {
  EEPROM.put(VOLUME_ADDR, vol);
  uint8_t checksum = calculateChecksum(vol);
  EEPROM.put(CHECKSUM_ADDR, checksum);
  EEPROM.commit();
  Serial.println("Volume tersimpan ke EEPROM");
}

bool readVolumeFromEEPROM() {
  float storedVolume;
  uint8_t storedChecksum, calculatedChecksum;
  EEPROM.get(VOLUME_ADDR, storedVolume);
  EEPROM.get(CHECKSUM_ADDR, storedChecksum);
  calculatedChecksum = calculateChecksum(storedVolume);

  if (storedChecksum == calculatedChecksum) {
    vol = storedVolume;
    Serial.print("Volume tersimpan di EEPROM: ");
    Serial.println(storedVolume);
    return true;
  } else {
    Serial.println("Checksum mismatch, data mungkin korup.");
    return false;
  }
}

void sendVolumeFromEEPROM() {
  readVolumeFromEEPROM(); // Pastikan nilai terbaru dibaca dari EEPROM
  String message = "EEPROM : " + String(vol);
  char mqtt_topic[70];
  snprintf(mqtt_topic, sizeof(mqtt_topic), "Log");
  client.publish(mqtt_topic, message.c_str(), true);
  Serial.print("Mengirimkan data dari EEPROM ke MQTT: ");
  Serial.println(message);
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  bool res;

  digitalWrite(yellowLedPin, HIGH); // Turn on yellow LED while connecting to WiFi

  res = wm.autoConnect("SMART-METER", "12345678"); // AP dengan password

  if (!res) {
    Serial.println("Failed to connect");
  } else {
    Serial.println("connected...yeey :)");
  }

  delay(10);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(greenLedPin, LOW); // Green LED off while not connected to WiFi
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(greenLedPin, HIGH); // Green LED on when connected to WiFi
  digitalWrite(yellowLedPin, LOW); // Turn off yellow LED when connected to WiFi
}

void turnOffYellowLed() {
  digitalWrite(yellowLedPin, LOW); // Matikan LED kuning
}

void connectToWiFi() {
  digitalWrite(yellowLedPin, HIGH); // LED kuning menyala saat mencoba menghubungkan ke WiFi

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Menghubungkan ke WiFi...");
    wifiManager.autoConnect("SMART-METER", "12345678");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Terhubung!");
    Serial.println(WiFi.localIP());
    digitalWrite(greenLedPin, HIGH); // LED hijau menyala setelah terhubung ke WiFi
    turnOffYellowLed(); // Panggil fungsi untuk mematikan LED kuning
  } else {
    Serial.println("Tidak dapat terhubung ke WiFi.");
    digitalWrite(greenLedPin, LOW); // LED hijau mati jika tidak terhubung ke WiFi
  }
}
void resetWiFi() {
  Serial.println("Resetting WiFi configuration...");
  preferences.clear();
  wifiManager.resetSettings(); // Reset pengaturan WiFiManager

  // Matikan LED hijau
  digitalWrite(greenLedPin, LOW);

  // Nyalakan LED kuning
  digitalWrite(yellowLedPin, HIGH); // LED kuning menyala saat konfigurasi WiFi direset

  // Buat Access Point untuk konfigurasi WiFi baru
  Serial.println("Membuat Access Point baru untuk konfigurasi WiFi...");
  wifiManager.startConfigPortal("SMART-METER", "12345678");

  // Tunggu hingga WiFi terhubung setelah konfigurasi
  Serial.println("Menunggu konfigurasi WiFi selesai...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  // Jika konfigurasi berhasil, restart ESP
  Serial.println("Konfigurasi selesai, restart ESP...");
  ESP.restart();
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  digitalWrite(yellowLedPin, HIGH); // LED kuning menyala saat setup dimulai
  digitalWrite(greenLedPin, LOW); // Pastikan LED hijau mati saat setup dimulai

  preferences.begin("wifi-config", false);

  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Tombol reset ditekan saat startup, masuk ke mode konfigurasi WiFi.");
    resetWiFi();
  } else {
    connectToWiFi();
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH);

  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING);
  currentTime = millis();
  cloopTime = currentTime;
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Inisialisasi relay OFF
  if (!readVolumeFromEEPROM()) {
    vol = 0.0; // Jika data di EEPROM rusak, reset volume ke 0
  }
  sendVolumeFromEEPROM();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  currentTime = millis();
  if (currentTime >= (cloopTime + 5000)) {
    cloopTime = currentTime;
    if (flow_frequency != 0) {
      vol += (flow_frequency * volume_per_pulse);
      Serial.print("Volume: ");
      Serial.print(vol);
      Serial.println(" LITER");

      // Membuat objek JSON
      StaticJsonDocument<200> jsonDocument;
      jsonDocument["guidDevice"] = device_guid;

      // Konversi nilai volume ke string dengan 2 desimal dan tambahkan "L"
      char volumeString[10];
      dtostrf(vol, 1, 2, volumeString);
      strcat(volumeString, "L");
      jsonDocument["Volume"] = volumeString;

      // Serialize objek JSON ke string
      char jsonBuffer[512];
      serializeJson(jsonDocument, jsonBuffer);

      // Publish pesan MQTT dengan objek JSON sebagai payload
      client.publish("Log", jsonBuffer, true);

      flow_frequency = 0;

      saveVolumeToEEPROM(); // Simpan volume yang diperbarui ke EEPROM
    } else {
      Serial.println("Volume: 0 LITER");
    }
  }

  // Baca status tombol
  int reading = digitalRead(buttonPin);

  // Jika tombol ditekan (reading == LOW) dan debounce delay terpenuhi
  if (reading == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis(); // Perbarui waktu debounce
    if (!buttonPressed) {
      Serial.println("Tombol ditekan!");
      buttonPressed = true;
      resetWiFi(); // Panggil fungsi reset WiFi
    }
  } else if (reading == HIGH) {
    buttonPressed = false;
  }
}