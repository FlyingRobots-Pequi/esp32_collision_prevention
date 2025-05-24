#include <Arduino.h>                          
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <mavlink.h>  

// ——— Configuração dos VL53L0X ———
#define NUM_SENSORS 4
const uint8_t XSHUT_PINS[NUM_SENSORS]    = {3, 2, 1, 0}; // pinos conectados ao XSHUT
const uint8_t NEW_I2C_ADDRS[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33}; // futuros enderecos dos sensores
Adafruit_VL53L0X sensors[NUM_SENSORS];

// ——— Configuração MAVLink via Serial padrão (UART1) ———
HardwareSerial mavSerial(1); // Usamos UART1 da esp32
#define MAV_BAUD   57600 // frequencia de publicacao da esp32
#define RATE_PUBLICATION 10 // essa frequencia de publicacao esta atrelada a taxa de atualizacao do algoritmo, ela influencia o CP_DELAY
#define MAV_COMP_ID_OBSTACLE_AVOIDANCE 196 // eh o id que representa o obstacle_avoidance
#define SERIAL_TX_PIN 20  // TX da ESP32-C3 (conectado ao RX do Pixhawk TELEM2)
#define SERIAL_RX_PIN 21  // RX da ESP32-C3 (não é necessário neste teste)

// funcao que manda a mensagem de distancia
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

  // preenche as leituras dos sensores
  for (uint8_t i = 0; i < count; i++) {
    od.distances[i] = dists[i];
  }
  // as outras posicoes das matrizes devem ser preenchidas, conforme a documentacao fala
  for (uint8_t i = count; i < MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN; i++) {
    od.distances[i] = UINT16_MAX;  // marcados como "não utilizados"
  }

  // manda a mensagem via mavlink
  mavlink_msg_obstacle_distance_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, &od);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);  // Usando Serial (UART0) para enviar MAVLink
  mavSerial.flush();
}

// funcao que manda um heartbeat
// o que ela basicamente fala eh que a esp32 esta "saudavel", ou seja, o topico que ela manda eh confiavel
void sendHeartbeat() {
  mavlink_message_t msg;

  mavlink_heartbeat_t hb{};
  hb.type = MAV_TYPE_ONBOARD_CONTROLLER;      // 18
  hb.autopilot = MAV_AUTOPILOT_INVALID;       // 8
  hb.base_mode = 0;                           // Sem modo específico
  hb.custom_mode = 0;
  hb.system_status = MAV_STATE_ACTIVE;        // 4
  hb.mavlink_version = 3;                     // MAVLink v2

  mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, &hb);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavSerial.write(buf, len);
}

void setup() {
  // inicia a leitura dos sensores via I2C
  Wire.begin();

  // Desliga todos os VL53L0X
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);

  // Inicializa e reendereça cada sensor
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH); // liga o sensor
    delay(10);
    if (!sensors[i].begin(0x29, &Wire)) {
      Serial.printf("Erro ao iniciar sensor %u!\n", i + 1);
      while (1);  // trava em caso de erro
    }
    sensors[i].setAddress(NEW_I2C_ADDRS[i]); // muda o endereco
    Serial.printf("Sensor %u -> 0x%02X\n", i+1, NEW_I2C_ADDRS[i]);
  }
  mavSerial.begin(MAV_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
}

void loop() {
  uint16_t distances[NUM_SENSORS];

  // envia o heartbeat
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {  // Envia a cada 1 segundo
    sendHeartbeat();
    lastHeartbeat = millis();
  }

  // le os sensores, cabe resaltar que a mensagem de chegada na px4 deve ser em cm, por isso a conversao
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    VL53L0X_RangingMeasurementData_t m;
    sensors[i].rangingTest(&m, false);
    distances[i] = (m.RangeStatus != 4) ? m.RangeMilliMeter/10 : UINT16_MAX;
  }

  // Print das distâncias no Serial, para debug
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
  delay(1000/RATE_PUBLICATION); // 1000 / RATE_PUBLICATION = tempo do delay em ms
}
