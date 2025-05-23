#include <Arduino.h>                          
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <mavlink.h>  // Certifique-se que está usando a versão correta da lib MAVLink

// ——— Configuração dos VL53L0X ———
#define NUM_SENSORS 4
const uint8_t XSHUT_PINS[NUM_SENSORS]    = {3, 2, 1, 0};
const uint8_t NEW_I2C_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33};
Adafruit_VL53L0X sensors[NUM_SENSORS];

// ——— Configuração MAVLink via Serial padrão (UART0) ———
HardwareSerial mavSerial(1); // Usamos UART1
#define MAV_BAUD   57600
#define RATE_PUBLICATION 10 // Hz
#define MAV_COMP_ID_OBSTACLE_AVOIDANCE 196
#define SERIAL_TX_PIN 20  // TX da ESP32-C3 (conectado ao RX do Pixhawk TELEM2)
#define SERIAL_RX_PIN 21  // RX da ESP32-C3 (não é necessário neste teste)

void sendObstacleDistances(const uint16_t *dists, uint8_t count) {
  mavlink_message_t msg;
  mavlink_obstacle_distance_t od{};

  od.time_usec    = static_cast<uint64_t>(micros());
  od.sensor_type  = MAV_DISTANCE_SENSOR_LASER;
  od.frame        = MAV_FRAME_BODY_FRD;
  od.increment    = 90;          // 360° / 4 sensores
  od.angle_offset = 0;           // ajustar conforme posição do sensor frontal
  od.min_distance = 20;          // mm
  od.max_distance = 2000;        // mm

  // preenche as leituras
  for (uint8_t i = 0; i < count; i++) {
    od.distances[i] = dists[i];
  }
  // resto como “não usado”
  for (uint8_t i = count; i < MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN; i++) {
    od.distances[i] = UINT16_MAX;  // marcados como "não utilizados"
  }

  mavlink_msg_obstacle_distance_encode(99, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, &od);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);  // Usando Serial (UART0) para enviar MAVLink
  mavSerial.flush();
}

void setup() {
  // Serial.begin(MAV_BAUD); // Inicia UART0 para MAVLink
  Wire.begin();           // SDA=21, SCL=22 (padrão no ESP32-C3)

  // Desliga todos os VL53L0X
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);

  // Inicializa e reendereça cada sensor
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    if (!sensors[i].begin(0x29, &Wire)) {
      Serial.printf("Erro ao iniciar sensor %u!\n", i + 1);
      while (1);  // trava em caso de erro
    }
    sensors[i].setAddress(NEW_I2C_ADDRS[i]);
    Serial.printf("Sensor %u -> 0x%02X\n", i+1, NEW_I2C_ADDRS[i]);
  }
  mavSerial.begin(MAV_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
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

  // Print das distâncias no Serial
  // Serial.print("Distâncias (mm): ");
  // for (uint8_t i = 0; i < NUM_SENSORS; i++) {
  //   if (distances[i] == UINT16_MAX) {
  //     Serial.print("----");
  //   } else {
  //     Serial.print(distances[i]);
  //   }
  //   Serial.print((i < NUM_SENSORS - 1) ? ", " : "\n");
  // }

  sendObstacleDistances(distances, NUM_SENSORS);
  delay(1000/RATE_PUBLICATION); // ~10 Hz
}
