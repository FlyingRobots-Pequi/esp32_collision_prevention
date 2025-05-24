## esp32_collision_prevention

Este projeto é uma implementação de um sistema de prevenção de colisão utilizando sensores de distância VL53L0X em um drone controlado pela PX4, com um microcontrolador ESP32 responsável pela leitura dos sensores e envio das informações via MAVLink.

## Objetivo 

Criar um sistema de **prevenção de colisões** baseado em sensores VL53L0X fixados em pontos estratégicos do drone, utilizando a infraestrutura do PX4 e a funcionalidade `collision prevention`.


## Contexto

A PX4 possui suporte nativo ao algoritmo de `collision prevention`, originalmente desenvolvido para sensores LiDAR 2D. Este projeto visa uma alternativa mais barata e leve utilizando sensores de distância baseados em tempo de voo (ToF) comunicando-se com a PX4 via MAVLink, através de uma ESP32.

## Hardware Utilizado
 - ESP32 Super Mini
 - 4x VL53L0X (Time-of-Flight sensor)
 - Pixhawk 6x

## Esquema Elétrico
Conforme a imagem abaixo

![EsquemaEletrico](docs/esquema_eletrico.png)

cada sensor VL53L0X está conectado ao barramento I2C da esp32, mas com controle individual via pino XSHUT. A principal função do pino XSHUT é desligar o sensor para mudar seu endereço no I2C, ou seja, todos os sensores serão desligados e ligados um a um mudando o endereço do I2C

## Comunicação com PX4
A ESP32 envia os dados dos sensores via MAVLink utilizando a mensagem OBSTACLE_DISTANCE. É necessário também enviar um HEARTBEAT.

#### Parâmetros da PX4
Esses parâmetros devem ser ajustados para ativar e configurar o algoritmo de prevenção de colisão:

| Parâmetro       | Valor               | Descrição                             |
| --------------- | ------------------- | ------------------------------------- |
| `COM_OBS_AVOID` | 1                   | Ativa o algoritmo de avoidance        |
| `CP_DIST`       | 0.5                 | Distância mínima de colisão           |
| `CP_DELAY`      | 0                   | Delay de resposta                     |
| `MPC_POS_MODE`  | `smoothed_velocity` | Modo de controle de posição           |
| `MAV_SYS_ID`    | 1                   | ID do sistema MAVLink                 |
| `MAV_2_CONFIG`  | `TELEM2`            | Porta MAVLink usada                   |
| `MAV_2_BAUD`    | 57600               | Baud rate máxima suportada pela ESP32 |

## Explicação do algoritmo

Esse algoritmo implementa, em uma ESP32, a leitura de quatro sensores de distância VL53L0X via I2C, gerenciando seus endereços dinamicamente com os pinos XSHUT, e envia os dados em tempo real para a PX4 utilizando o protocolo MAVLink. Durante a inicialização, os sensores são ligados um a um para que possam receber endereços únicos no barramento I2C, permitindo leitura simultânea. No loop principal, a ESP32 coleta as leituras de distância de cada sensor, converte os dados para o formato esperado pela mensagem MAVLink `OBSTACLE_DISTANCE`, e os envia via UART (porta TELEM2) para a PX4. Um heartbeat também é transmitido periodicamente para manter a conexão com o sistema PX4 ativa. O sistema funciona a uma taxa de publicação de aproximadamente 10 Hz, e os dados enviados podem ser usados pelo algoritmo de collision prevention do PX4 para evitar obstáculos em tempo real.

## Debug e Verificação
Comandos úteis para depurar a comunicação com a PX4 no console do QGroundControl:

```
mavlink stream -d /dev/ttyACM0 -s OBSTACLE_DISTANCE -r 4
uorb top obstacle_distance
listener obstacle_distance
```

📚 Referências

[PX4 Collision Prevention Documentation](https://docs.px4.io/main/en/computer_vision/collision_prevention.html)

[Adafruit VL53L0X Library](https://github.com/adafruit/Adafruit_VL53L0X)