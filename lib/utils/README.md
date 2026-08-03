# utils

Helpers compartilhados pelo `main.cpp`: drivers de sensores (MPU6050, BMP280, QMC5883L), controle de LEDs/bateria e a alocação de controle X-quad.

## Conteúdo

| Arquivo                  | Responsabilidade                                                                 |
|--------------------------|----------------------------------------------------------------------------------|
| `utils.h` / `utils.cpp`  | Inicialização, calibração e leitura do MPU6050; driver bare-metal do QMC5883L (I2C direto); alocação X-quad (torques → ω²); helpers de display no Serial (incluindo `displayGains` para K e Kr do SDRE). |
| `bmp280.h` / `.cpp`      | Driver bare-metal do barômetro BMP280 (I2C direto): pressão, temperatura e altura relativa a uma referência capturada no boot. **Somente leitura** — não realimenta o controle. |
| `qmc5883p.h` / `.cpp`    | Driver do magnetômetro QMC5883P da TPJ-01 (`0x2C`): init, leitura em µT com calibração hard/soft-iron e leitura crua em LSB para o sketch de calibração. |
| `led_control.h` / `.cpp` | Classe `LEDControl` — gerencia os 4 LEDs da TPJ-01 em dois canais (verdes = status, vermelhos = alerta), modos blink, leitura de bateria via ADC e thresholds (low/critical). |

## Conexões I2C

Pinos e endereços em [`include/board_config.h`](../../include/board_config.h) — barramento único.

- **SDA** = IO39, **SCL** = IO40, **clock** = 400 kHz (pull-ups de 4,7 kΩ na placa)
- MPU6050 em `0x68` — `start_IMU_MPU6050` é o **único** `Wire.begin()` do firmware
- BMP280 em `0x76` (`start_BMP280` reusa o `Wire` já inicializado)
- QMC5883**P** em **`0x2C`** (`start_QMC5883P` reusa o `Wire`) — ⚠️ o esquemático anota `0x0D`,
  que é o endereço do QMC5883**L**; o `0x2C` vem do datasheet do 'P' (seção 5.4). Ver `qmc5883p.h`
  para a tabela completa de diferenças entre os dois chips.

As funções `start_QMC5883L` / `read_QMC5883L` em `utils.cpp` são do chip da **placa anterior** e
não funcionam na TPJ-01 — mantidas apenas como referência histórica.

## Leitura rápida do MPU (`read_MPU6050_raw`)

O `mpu.getEvent()` da Adafruit custa **~1 ms** no ESP32-S2 — overhead de software (Adafruit_BusIO + stack `Wire`), não do clock (já em 400 kHz). O loop de voo usa `read_MPU6050_raw`, que faz um **burst read cru** dos 14 registradores (`0x3B`..`0x48`) direto pelo `Wire`, aplicando as **mesmas escalas** da Adafruit (range 2 g / 1000 °/s). Resultado: **~458 µs (≈1.8× mais rápido)** com valores idênticos.

- A **inicialização/configuração** continua pela lib Adafruit (`start_IMU_MPU6050`) — só a leitura quente é crua.
- ⚠️ As escalas estão fixas para os ranges de `start_IMU_MPU6050` (2 g, 1000 °/s). **Se mudar os ranges, atualizar as constantes** em `read_MPU6050_raw`.
- Esse ganho foi o que permitiu o loop de controle **síncrono** caber abaixo de 5 ms.

## API principal

```cpp
// Sensores
void start_IMU_MPU6050(Adafruit_MPU6050& mpu);
void read_MPU6050(...);                 // saída: ax,ay,az em g; gx,gy,gz em rad/s (via lib Adafruit)
void read_MPU6050_raw(...);             // idem — burst read cru via Wire, ~1.8x mais rápido (loop de voo)
void start_QMC5883L(TwoWire& wire);
void setQMC5883LCalibration(...);       // hard-iron offsets + soft-iron scales
void read_QMC5883L(float& mx, float& my, float& mz);  // saída em μT

// Alocação de controle (X-quad)
void calculateMotorOmegaSq(float thrust, float u_torques[3],
                           float b, float d, float L_arm,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq);

// Display (Serial Monitor, modo DEBUG)
void displayStates(float states[]);
void displayControlSignals(float u[], float thrust);
void displayMotorOmegaSq(...);
void displayMotorOmegaSqDetailed(float w1_sq, ...);
```

## LEDs e bateria

Detalhes em [`docs/LED_BATTERY_GUIDE.md`](../../docs/LED_BATTERY_GUIDE.md).

```cpp
LEDControl leds;
leds.begin();
leds.setSystemReady(true);     // azul piscando
leds.setUDPReceiving(true);    // verde piscando
leds.setLowPower(true);        // vermelho aceso

if (leds.isCriticalBattery()) { /* halt */ }
```

## Calibração

Antes do primeiro voo, rodar:

- `test/calibrate_mpu.cpp` — offsets do MPU6050 (cole os valores em `main.cpp`)
- `test/calibrate_magnetometer.cpp` — offsets hard/soft-iron do QMC5883L (idem)
