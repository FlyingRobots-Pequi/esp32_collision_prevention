#include <Arduino.h>                          
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MAVlink.h>      

// ——— Configuração dos VL53L0X ———
#define NUM_SENSORS 4
const uint8_t XSHUT_PINS[NUM_SENSORS]    = {4, 3, 1, 1};
const uint8_t NEW_I2C_ADDRS[NUM_SENSORS] = {0x30,0x31,0x32,0x33};
Adafruit_VL53L0X sensors[NUM_SENSORS];

// ——— Configuração MAVLink / UART ———
// vamos criar explicitamente um HardwareSerial para UART2
HardwareSerial mavlinkSerial(2);
#define MAV_BAUD   57600
#define MAV_RX_PIN 19   // ESP32 RX2
#define MAV_TX_PIN 18   // ESP32 TX2

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 por default no ESP32

  // Desliga todos os VL53L0X
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);

  // Liga um a um e reendereco
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    if (! sensors[i].begin(0x29, &Wire)) {
      Serial.printf("Erro sensor %u!\n", i+1);
      while (1);
    }
    sensors[i].setAddress(NEW_I2C_ADDRS[i]);
    Serial.printf("Sensor %u -> 0x%02X\n", i+1, NEW_I2C_ADDRS[i]);
  }

  // Inicia UART2 (MAVLink) no TELEM2 do Pixhawk
  mavlinkSerial.begin(MAV_BAUD, SERIAL_8N1, MAV_RX_PIN, MAV_TX_PIN);
}

void loop() {
  uint16_t distances[NUM_SENSORS];
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    VL53L0X_RangingMeasurementData_t m;
    sensors[i].rangingTest(&m, false);
    distances[i] = (m.RangeStatus != 4)
                 ? m.RangeMilliMeter
                 : UINT16_MAX;
  }
  sendObstacleDistances(distances, NUM_SENSORS);
  delay(100); // ~10 Hz
}

void sendObstacleDistances(const uint16_t *dists, uint8_t count) {
  mavlink_message_t msg;
  mavlink_obstacle_distance_t od{};

  od.time_usec   = static_cast<uint64_t>(micros());  // em μs, substitui time_boot_ms :contentReference[oaicite:1]{index=1}
  od.sensor_type = MAV_DISTANCE_SENSOR_LASER;
  od.frame       = MAV_FRAME_BODY_FRD;              
  od.increment   = static_cast<uint8_t>(360 / count);
  od.min_distance = 20;     // mm
  od.max_distance = 2000;   // mm

  // preenche as leituras
  for (uint8_t i = 0; i < count; i++) {
    od.distances[i] = dists[i];
  }
  // resto como “não usado”
  for (uint8_t i = count; i < MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN; i++) {
    od.distances[i] = UINT16_MAX;
  }

  mavlink_msg_obstacle_distance_encode(1, 0, &msg, &od);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlinkSerial.write(buf, len);
}
