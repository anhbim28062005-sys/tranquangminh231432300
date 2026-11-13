
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ================== KHAI BAO CHAN ==================
#define SDA_PIN 21
#define SCL_PIN 22
#define LED_PIN 18
#define BUZZER_PIN 19

// ================== WIFI ==================
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ================== MQTT ==================
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;

const char* TOPIC_DATA =
  "fall-detect/demo01/data";

const char* TOPIC_ALERT =
  "fall-detect/demo01/alert";

const char* TOPIC_CONTROL =
  "fall-detect/demo01/control";

WiFiClient espClient;
PubSubClient client(espClient);

// ================== CAM BIEN ==================
Adafruit_MPU6050 mpu;

// ================== CAU TRUC DU LIEU ==================
struct SensorData {
  float ax;
  float ay;
  float az;
  float acc;
  float pitch;
  float roll;
  bool fall;
};

// ================== FREERTOS QUEUE ==================
QueueHandle_t dataQueue;
QueueHandle_t dataAlert;

// ================== BIEN DIEU KHIEN ==================
volatile bool manualMode = false;
volatile bool manualLED = false;
volatile bool manualBuzzer = false;

// ================== HAM NHAN LENH MQTT ==================
int getJsonInt(const char* msg, const char* key, int def) {
  const char* p = strstr(msg, key);

  if (p == NULL) return def;

  p = strchr(p, ':');

  if (p == NULL) return def;

  return atoi(p + 1);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  char message[200];

  if (length >= sizeof(message)) return;

  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.print("MQTT nhan: ");
  Serial.println(message);

  if (strcmp(topic, TOPIC_CONTROL) == 0) {

    if (strstr(message, "\"mode\":\"manual\"") != NULL ||
        strstr(message, "\"mode\": \"manual\"") != NULL) {

      manualMode = true;

      manualLED = getJsonInt(message, "\"led\"", 0);
      manualBuzzer = getJsonInt(message, "\"buzzer\"", 0);
    }

    else if (strstr(message, "\"mode\":\"auto\"") != NULL ||
             strstr(message, "\"mode\": \"auto\"") != NULL) {

      manualMode = false;
    }
  }
}

// ================== KET NOI WIFI ==================
void connectWiFi() {

  Serial.println("Dang ket noi WiFi...");

  WiFi.begin(ssid, password, 6);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  Serial.println("\nWiFi da ket noi!");
  Serial.println(WiFi.localIP());
}

// ================== KET NOI MQTT ==================
void reconnectMQTT() {

  while (!client.connected()) {

    Serial.println("Dang ket noi MQTT...");

    String clientID = "ESP32-FALL-";
    clientID += String((uint32_t)ESP.getEfuseMac(), HEX);

    if (client.connect(clientID.c_str())) {

      Serial.println("MQTT connected!");

      client.subscribe(TOPIC_CONTROL);

    } else {

      Serial.print("MQTT loi: ");
      Serial.println(client.state());

      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// =====================================================
// TASK 1: DOC DU LIEU CAM BIEN - CORE 1
// =====================================================
void TaskReadData(void *pvParameters) {

  SensorData data;

  bool lastFallState = false;
  unsigned long lastImpactTime = 0;

  while (1) {

    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    // Doc gia toc (don vi g)
    data.ax = a.acceleration.x / 9.80665;
    data.ay = a.acceleration.y / 9.80665;
    data.az = a.acceleration.z / 9.80665;

    // Tinh gia toc tong hop
    data.acc = sqrt(
      data.ax * data.ax +
      data.ay * data.ay +
      data.az * data.az
    );

    // Tinh goc nghieng (do)
    data.pitch = atan2(
      -data.ax,
      sqrt(data.ay * data.ay + data.az * data.az)
    ) * 180.0 / PI;

    data.roll = atan2(
      data.ay,
      data.az
    ) * 180.0 / PI;

    // Dieu kien gia toc bat thuong
    bool impact = (data.acc < 0.5 || data.acc > 2.5);

    if (impact) {
      lastImpactTime = millis();
    }

    // Dieu kien goc nghieng
    bool tilted = (
      fabs(data.pitch) > 55 ||
      fabs(data.roll) > 55
    );

    // Phat hien te nga trong khoang 1.5 giay
    data.fall = tilted &&
      (millis() - lastImpactTime < 1500);

    // Dua du lieu vao dataQueue
    xQueueSend(dataQueue, &data, 0);

    // Neu trang thai te nga thay doi
    if (data.fall != lastFallState) {

      xQueueSend(dataAlert, &data.fall, 0);

      lastFallState = data.fall;
    }

    // Hien thi Serial
    Serial.printf(
      "Ax: %.2f | Ay: %.2f | Az: %.2f | Acc: %.2f | Pitch: %.1f | Roll: %.1f | Fall: %d\n",
      data.ax,
      data.ay,
      data.az,
      data.acc,
      data.pitch,
      data.roll,
      data.fall
    );

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// =====================================================
// TASK 2: TRUYEN DU LIEU MQTT - CORE 0
// =====================================================
void TaskTransmitData(void *pvParameters) {

  SensorData data;

  bool lastPublishedFall = false;

  connectWiFi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  while (1) {

    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }

    if (!client.connected()) {
      reconnectMQTT();
    }

    client.loop();

    // Lay du lieu tu dataQueue
    if (xQueueReceive(
      dataQueue,
      &data,
      pdMS_TO_TICKS(100)
    ) == pdTRUE) {

      char payload[300];

      snprintf(
        payload,
        sizeof(payload),
        "{\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"acc\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"fall\":%d}",
        data.ax,
        data.ay,
        data.az,
        data.acc,
        data.pitch,
        data.roll,
        data.fall
      );

      // Gui du lieu len MQTT
      client.publish(TOPIC_DATA, payload);

      // Gui trang thai canh bao khi thay doi
      if (data.fall != lastPublishedFall) {

        client.publish(
          TOPIC_ALERT,
          data.fall ? "1" : "0"
        );

        lastPublishedFall = data.fall;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =====================================================
// TASK 3: DIEU KHIEN CANH BAO - CORE 1
// =====================================================
void TaskWarning(void *pvParameters) {

  bool alertState = false;

  bool ledState;
  bool buzzerState;

  bool newAlert;

  while (1) {

    // Nhan du lieu tu dataAlert
    if (xQueueReceive(
      dataAlert,
      &newAlert,
      pdMS_TO_TICKS(50)
    ) == pdTRUE) {

      alertState = newAlert;
    }

    // Chon che do dieu khien
    if (manualMode) {

      ledState = manualLED;
      buzzerState = manualBuzzer;

    } else {

      ledState = alertState;
      buzzerState = alertState;
    }

    // Dieu khien LED va Buzzer
    digitalWrite(LED_PIN, ledState);
    digitalWrite(BUZZER_PIN, buzzerState);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Khoi tao I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Khoi tao MPU6050
  if (!mpu.begin(0x68)) {

    Serial.println("Khong tim thay MPU6050!");

    while (1) {
      delay(100);
    }
  }

  Serial.println("MPU6050 da san sang!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Tao Queue
  dataQueue = xQueueCreate(
    10,
    sizeof(SensorData)
  );

  dataAlert = xQueueCreate(
    5,
    sizeof(bool)
  );

  if (dataQueue == NULL || dataAlert == NULL) {
    Serial.println("Loi tao Queue!");
    while (1) delay(100);
  }

  // Tao Task doc du lieu - Core 1
  xTaskCreatePinnedToCore(
    TaskReadData,
    "ReadData",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  // Tao Task truyen du lieu - Core 0
  xTaskCreatePinnedToCore(
    TaskTransmitData,
    "TransmitData",
    8192,
    NULL,
    1,
    NULL,
    0
  );

  // Tao Task canh bao - Core 1
  xTaskCreatePinnedToCore(
    TaskWarning,
    "Warning",
    4096,
    NULL,
    2,
    NULL,
    1
  );

  Serial.println("He thong Fall Detection da khoi dong!");
}

void loop() {

  vTaskDelay(pdMS_TO_TICKS(1000));

}

