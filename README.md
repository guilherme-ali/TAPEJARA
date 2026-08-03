# TAPEJARA

> 📌 **Origem do Projeto:** Este repositório é derivado do projeto [SDRE_VECTORIZED](https://github.com/guilherme-ali/SDRE_VECTORIZED), baseado no [Release V1.0.0](https://github.com/guilherme-ali/SDRE_VECTORIZED/releases/tag/V1.0.0).

Controle de atitude de quadricóptero na placa **Tapejara TPJ-01** (ESP32-S3) com **SDRE-LQR** (State-Dependent Riccati Equation) recalculado em tempo real. O sistema resolve a DARE a cada ciclo via uma das implementações otimizadas disponíveis (SDA, ADDA, etc.), com PID como alternativa selecionável.

> Até 2026-07 o firmware rodava numa montagem sobre **ESP32-S2 Saola-1** com sensores em fio. Os benchmarks de solver marcados como "ESP32-S2" em `lib/AUTOLQR/` e `lib/KalmanFilter/` são dessa medição e ainda não foram refeitos no S3 — que, ao contrário do S2, **tem FPU**.

## Visão geral

- **Controle SDRE em tempo real** — ganhos K recalculados ciclo a ciclo a partir de A(x)
- **Multiplos solvers DARE** — 7 algoritmos implementados em `lib/AUTOLQR/` (benchmark abaixo)
- **Riccati em ponto fixo Q13.18** — método padrão `"SDA_FIXED"` (default de `computeGains()`), ~2,7× mais rápido que o SDA `float`; em overflow/saturação retorna `false` e mantém o ganho `K` do ciclo anterior. O SDA `float` (`"SDA"`) segue disponível como referência exata (detalhes em `lib/AUTOLQR/`)
- **PID alternativo** — controlador PID compatível com a mesma interface, selecionável via flag
- **Madgwick (AHRS)** — estimação de orientação a partir de MPU6050 (leitura crua ~1.8× + QMC5883L opcional)
- **Butterworth digital** — filtro passa-baixa por eixo antes do AHRS (anti-aliasing em 200 Hz)
- **Riccati desacoplada do ciclo** — modo **síncrono reordenado** (padrão: aplica K(x_{k−1}) e resolve K(x_k) *após* atuar, ao final do loop) ou **assíncrono** (FreeRTOS task, loop fixo a 200 Hz); em ambos a Riccati sai do caminho sensor→atuador
- **Telemetria em RAM + LittleFS** — buffer circular de 1000 amostras, persistido em flash ao desarmar
- **WiFi/UDP (CRTP)** — protocolo Crazyflie, compatível com o app **ESP-Drone** (Espressif)
- **Failsafe de tilt** — desarma e trava motores em inclinação > 60° (zona singular `1/cos(pitch)`)

## Hardware

Placa **Tapejara TPJ-01 v1.0** (X-frame integrado, FR-4 1,2 mm). Esquemático em
`tapejaraBoard_v1.pdf`. **Toda a pinagem — inclusive a lista de GPIOs proibidos — vive em
[`include/board_config.h`](include/board_config.h)**; não redefina pinos em outro lugar.

| Componente       | Modelo                        | Observação                              |
|------------------|-------------------------------|-----------------------------------------|
| Microcontrolador | ESP32-S3-WROOM-1-N16R8        | 240 MHz, dual-core, 16 MB flash, FPU    |
| IMU              | MPU6050 `0x68`                | I2C SDA=IO39, SCL=IO40, 400 kHz         |
| Barômetro        | BMP280 `0x76`                 | Mesmo barramento — **somente leitura**  |
| Magnetômetro     | QMC5883**P** `0x2C`           | Mesmo barramento — ⚠️ **não** é `0x0D` (ver abaixo) |
| Câmera           | conector OV2640 (24 vias)     | Não usada pelo firmware                 |
| Motores          | 4× coreless 8520, hélice 55 mm| PWM 25 kHz, 10-bit — IO8/IO48/IO2/IO4   |
| Bateria          | LiPo 1S 3.7 V, 500 mAh        | ADC em IO1 (divisor 100k/100k)          |
| LEDs status      | 2 verdes + 2 vermelhos        | IO41/IO43 (status), IO6/IO5 (alerta)    |

> ⚠️ **O esquemático erra o endereço do magnetômetro.** O bloco *Magnetometer / Compass* do
> `tapejaraBoard_v1.pdf` anota `I2C Addr.: 0x0D`, que é o endereço do QMC5883**L** da placa
> anterior. O chip montado é o QMC5883**P**, cujo endereço de fábrica é **`0x2C`** (datasheet QST
> 13-52-19 rev A, seção 5.4). Falar com `0x0D` produz
> `requestFrom(): i2cRead returned Error -1`. Os dois chips também têm mapas de registradores
> diferentes (dados em `0x01`, não `0x00`; control 1 em `0x0A`, não `0x09`), por isso o 'P' tem
> driver próprio em `lib/utils/qmc5883p.*` — o `start_QMC5883L` de `utils.cpp` é do chip antigo e
> ficou como código morto. Em dúvida sobre quem está no barramento, rode `test/i2c_scan.cpp`.

**Magnetômetro:** habilitado por `USE_MAGNETOMETER` em `src/main.cpp`. Se o sensor não responder
na inicialização, o AHRS cai automaticamente para 6-DOF em vez de alimentar o Madgwick com zeros.
**Recalibre antes de voar** com `test/calibrate_magnetometer.cpp`: os coeficientes hard/soft-iron
do chip antigo não valem aqui (sensibilidade 3750 vs 3000 LSB/G e outra orientação na PCB), e por
isso estão zerados no `main.cpp`.

**Serial:** o USB-C vai direto ao chip (IO19/IO20), sem conversor USB-UART, e o U0TXD (IO43) é
usado como LED. Por isso o build define `-DARDUINO_USB_CDC_ON_BOOT=1` — sem essa flag não há
saída serial nenhuma.

**Numeração dos motores** (frente = topo do PCB): M1=IO8 frente-esquerda CCW, M2=IO48
frente-direita CW, M3=IO2 trás-direita CCW, M4=IO4 trás-esquerda CW. Antes do primeiro voo,
valide com `test/motor_id_test.cpp` (hélices removidas).

## Estrutura do projeto

```
TAPEJARA/
├── platformio.ini            # Configuração PlatformIO (ESP32-S3, lib_deps)
├── include/
│   └── board_config.h        # Pinagem da TPJ-01 + GPIOs proibidos (fonte de verdade)
├── src/
│   └── main.cpp              # Loop principal + SDRETask + montagem das matrizes
├── lib/
│   ├── AUTOLQR/              # 7 solvers DARE (SDA, ADDA, ASDA, ...) + operações matriciais
│   ├── KalmanFilter/         # Filtro de Kalman linear (opcional — alternativa ao Madgwick)
│   ├── PIDController/        # PID compatível com interface SDRE (controlador alternativo)
│   ├── BiquadFilter/         # Butterworth de 2ª ordem para sinais da IMU
│   ├── Telemetry/            # Buffer circular em RAM + persistência em LittleFS
│   ├── MotorControl/         # PWM dos 4 ESCs + armar/desarmar + mapeamento ω² → throttle
│   ├── WiFiComm/             # Servidor UDP CRTP (compatível com app ESP-Drone)
│   └── utils/                # Drivers MPU6050/BMP280/QMC5883L, LEDs/bateria, alocação X-quad
├── python/
│   ├── atitude_sim.py            # Simulação do controle de atitude
│   ├── compara_solvers.py        # Comparação dos solvers DARE
│   ├── plot_telemetry.py         # Plota CSV exportado pela telemetria do drone
│   ├── simulador/                # Notebook (Jupyter) com simulações exploratórias
│   ├── matriz_otima/             # Busca/análise dos parâmetros ótimos de Q,R
│   ├── execucao_otima/           # Execução da matriz ótima encontrada
│   └── outputs/                  # Resultados (PNG, MP4, XLSX, CSV)
├── test/                     # Calibração de sensores, benchmarks, testes unitários
└── docs/                     # Guias auxiliares (LED, calibração, WiFi, quick start)
```

## Configuração rápida (flags em `src/main.cpp`)

```cpp
const bool DEBUG_MODE       = false; // true: prints detalhados; false: Serial Plotter
const bool PRINT_TELEMETRY  = false; // stream contínuo roll/pitch/yaw/p/q/r
const bool USE_MAGNETOMETER = false; // 9-DOF (QMC5883L) ou 6-DOF (só accel+gyro)
const int  CONTROLLER_TYPE  = 0;     // 0 = SDRE, 1 = PID
const bool USE_ASYNC_SDRE   = false; // true: Riccati em FreeRTOS task; false: síncrono reordenado (padrão)
```

## Compilação

Pré-requisitos: [PlatformIO](https://platformio.org/) (extensão VS Code recomendada), Python 3.8+ para simulações.

```bash
pio run                    # compila
pio run --target upload    # envia para o ESP32-S3 (USB-C direto no chip)
pio device monitor         # monitor serial (115200 baud)
```

## Benchmark dos solvers DARE

ESP32-S2 @ 240 MHz (placa anterior, sem FPU), sistema 6 estados × 3 controles, **800 000 execuções** sob dinâmica real de quadricóptero (resultados publicados em CBA 2026):

### Desempenho temporal

| Método         | Média (μs)       | σ (μs)   | Pior caso (μs) | Falhas / 800k | Iterações (méd ± σ) | Iter. (pior) |
|----------------|------------------|----------|----------------|---------------|---------------------|--------------|
| SDA-SS         | **8 413,26**     | 541,55   | 10 902         | 55 349 (6,9 %) | 7,79 ± 0,52         | 10           |
| **SDA (base)** | **8 663,59**     | **146,49** | **8 750**    | **0**          | 7,99 ± 0,13         | 8            |
| SDA-Scaled     | 8 854,91         | 327,95   | 11 024         | 48 302 (6,0 %) | 8,08 ± 0,31         | 10           |
| ASDA           | 9 114,64         | **24,64** | 9 180         | 0              | 8,00 ± 0,00         | 8            |
| SDA-ADDA       | 10 754,63        | 556,55   | 13 654         | 40 314 (5,0 %) | 7,96 ± 0,42         | 10           |
| Iterativo      | 11 912,00        | 2 868,74 | 16 884         | 0              | 22,49 ± 5,64        | 32           |
| Van Dooren     | 39 281,13        | 3 637,45 | 126 877        | 0              | 1,00 ± 0,00         | 1            |

### Precisão (erro RMS dos ganhos K vs. método iterativo de referência)

| Método         | Erro RMS                  |
|----------------|---------------------------|
| **SDA (base)** | **9,36 × 10⁻⁷**           |
| ASDA           | 1,92 × 10⁻⁵               |
| Van Dooren     | 5,53 × 10⁻⁵               |
| SDA-ADDA       | 1,85 × 10⁻⁴               |
| SDA-SS         | 3,22 × 10⁻⁴               |
| SDA-Scaled     | 3,43 × 10⁻⁴               |
| Iterativo      | — (referência)            |

> **Recomendação:** **SDA (base)** — melhor balanço velocidade × precisão × robustez. Zero falhas em 800 k execuções, menor erro RMS (~10⁻⁷). A tabela acima é medida em `float`.
>
> **No firmware**, o caminho de produção é o `"SDA_FIXED"` (fixed-point Q13.18, ~3,2 ms) derivado do SDA base — default de `computeGains()` e necessário para o solver caber no loop síncrono de 5 ms (o SDA `float` de ~8,6 ms não cabe). O SDA `float` (`"SDA"`) permanece como referência exata e *fallback* manual.
>
> **ASDA** como alternativa quando previsibilidade temporal é crítica (σ de apenas 24,64 μs).
>
> **Evitar** SDA-SS, SDA-Scaled e SDA-ADDA em malha de tempo real: ~5–7 % de falha sob excitações estocásticas do voo. **Van Dooren** descartado por custo: ~4,5× mais lento que o SDA base.

Detalhes, derivações e API em [`lib/AUTOLQR/README.md`](lib/AUTOLQR/README.md). Referências bibliográficas dos algoritmos em [`docs/REFERENCES.md`](docs/REFERENCES.md#2-solvers-dare-equação-algébrica-de-riccati-discreta).

## Modelo do sistema

**Estado** (6): $x = [\phi,\ \theta,\ \psi,\ p,\ q,\ r]^T$ — ângulos de Euler + taxas no corpo

**Controle** (3): $u = [\tau_x,\ \tau_y,\ \tau_z]^T$ — torques nos eixos do corpo

Dinâmica não-linear linearizada por SDRE:

$$\dot{x} = A(x)\,x + B\,u$$

com $A(x)$ recalculada a cada ciclo. `updateSystemMatrix()` em `main.cpp` monta $A_d, B_d, Q_d, R_d$ analiticamente (Taylor 2ª/3ª ordem) explorando a esparsidade do problema — ~14× mais rápido que multiplicação matricial genérica.

Controle aplicado:

$$u = -K\,x + K_r\,r,\quad K_r = -K[:,\,0\!:\!m]$$

## Simulações Python

```bash
cd python

python atitude_sim.py                                # Simulação do controle de atitude
python compara_solvers.py                            # Comparação dos solvers DARE
python plot_telemetry.py outputs/telem.csv           # Plota CSV exportado pelo drone
python matriz_otima/busca_parametros_otimos_sdre.py  # Busca Q,R ótimos
```

## Documentação

- [`docs/REFERENCES.md`](docs/REFERENCES.md) — **referências bibliográficas de todos os métodos**
- [`lib/AUTOLQR/README.md`](lib/AUTOLQR/README.md) — solvers DARE, API, benchmarks, **fixed-point Q13.18**
- [`lib/KalmanFilter/README.md`](lib/KalmanFilter/README.md) — filtro de Kalman linear (opcional)
- [`lib/PIDController/README.md`](lib/PIDController/README.md) — controlador PID alternativo
- [`lib/WiFiComm/README.md`](lib/WiFiComm/README.md) — protocolo CRTP e ESP-Drone
- [`docs/QUICK_TEST.md`](docs/QUICK_TEST.md) — teste rápido de conexão e controle
- [`docs/MOTOR_CALIBRATION_GUIDE.md`](docs/MOTOR_CALIBRATION_GUIDE.md) — calibração de coeficiente de empuxo
- [`docs/WIFI_INTEGRATION.md`](docs/WIFI_INTEGRATION.md) — setup do app ESP-Drone
- [`docs/LED_BATTERY_GUIDE.md`](docs/LED_BATTERY_GUIDE.md) — indicadores LED e thresholds de bateria

## Contribuições

Issues e PRs bem-vindos. Para PRs significativos, abra uma issue antes para discussão.

---

*Pesquisa em controle adaptativo de VANTs — SDRE em tempo real para sistemas embarcados.*
