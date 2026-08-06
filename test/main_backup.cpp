/**
 * Controle de atitude de quadricoptero na placa Tapejara TPJ-01 v1.0
 * (ESP32-S3-WROOM-1-N16R8) com SDRE-LQR (ou PID).
 *
 * Pinagem em include/board_config.h — inclusive a lista de GPIOs proibidos.
 *
 * Loop principal a 200 Hz (5 ms): IMU + Madgwick + alocacao X-quad.
 * Task paralela (FreeRTOS) recalcula ganhos via DARE — desacopla a Riccati do ciclo.
 * Telemetria em RAM (Telemetry) e persistente em LittleFS sob desarme.
 * Comunicacao via WiFi/UDP (CRTP, compativel com app ESP-Drone da Espressif).
 */

#include <AutoLQR.h>
#include "PIDController.h"
#include <MadgwickAHRS.h>
#include <Wire.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===== Flags de configuracao =====
const bool DEBUG_MODE       = false; // true: prints detalhados; false: Serial Plotter
const bool PRINT_TELEMETRY  = true; // true: stream continuo de roll,pitch,yaw,p,q,r
// A TPJ-01 traz um QMC5883P (0x2C), nao o QMC5883L (0x0D) da placa antiga —
// driver proprio em lib/utils/qmc5883p.*. Antes de ligar isto em voo, RECALIBRE:
// os offsets abaixo sao do chip antigo e nao valem para este.
const bool USE_MAGNETOMETER = true; // true: 9-DOF (magnetometro); false: 6-DOF (accel+gyro)
const int  CONTROLLER_TYPE  = 0;     // 0 = SDRE, 1 = PID
const bool USE_ASYNC_SDRE   = false;  // true: Riccati em FreeRTOS task; false: sincrono no loop
// =================================

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "board_config.h" // Pinagem da placa TPJ-01 (unica fonte de verdade)
#include "utils.h"
#include "bmp280.h"       // Barometro (somente leitura)
#include "qmc5883p.h"     // Magnetometro (chip da TPJ-01, nao o QMC5883L antigo)
#include "MotorControl.h" // Incluir controle de motores
#include "WiFiComm.h" // Comunicação WiFi/UDP
#include "led_control.h" // Controle de LEDs e bateria
#include "Telemetry.h"   // Buffer circular de telemetria em RAM
#include <BiquadFilter.h>

#define STATE_SIZE   6
#define CONTROL_SIZE 3

void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r, float omega_r);

// ===== Parametros fisicos do drone (medidos) =====
const float Ixx   = 42.95e-6f;  // kg·m^2 — inercia roll
const float Iyy   = 37.77e-6f;  // kg·m^2 — inercia pitch
const float Izz   = 76.15e-6f;  // kg·m^2 — inercia yaw
const float Ir    = 1.02e-7f;   // kg·m^2 — inercia do rotor
const float L_ARM = 0.0475f * 0.70710678f; // 60 mm * sin(45°) — braco efetivo em config X
const float SAMPLING_TIME_S         = USE_ASYNC_SDRE ? 0.002f : 0.002f;
const unsigned long LOOP_PERIOD_US  = static_cast<unsigned long>(SAMPLING_TIME_S * 1e6f);
// Telemetria decimada: grava 1 amostra a cada N ciclos do loop. Buffer (CAPACITY
// fixo, ver Telemetry.h) cobre CAPACITY*N*SAMPLING_TIME_S segundos de voo.
// N=5 -> dt~25ms (fs~40Hz, margem 5x sobre o fc=8Hz usado na identificacao) ->
// ~25s de voo em vez de ~5s a cada ciclo.
const int TELEMETRY_DECIMATION_CYCLES = 5;

// ===== Coeficientes motor + helice (medidos via test/motor_calibration_test.cpp) =====

// helice 45mm
const float MOTOR_B_COEFF = 1.77e-8f;   // N/(rad/s)^2 — empuxo medido
const float MAX_RPM       = 31086.0f;   // RPM @ 100% duty

// helice 55mm (em uso)
//const float MOTOR_B_COEFF = 2.98e-8f;                      // N/(rad/s)^2 — empuxo medido
//const float MAX_RPM       = 26423.0f;                      // RPM @ 100% duty

const float MOTOR_D_COEFF = 0.05f * MOTOR_B_COEFF;         // N·m/(rad/s)^2 — drag (estimado)
const float MAX_OMEGA     = (MAX_RPM * 2.0f * PI) / 60.0f; // ~2769 rad/s
const float MAX_THRUST    = MOTOR_B_COEFF * MAX_OMEGA * MAX_OMEGA; // N

float omega_r = 0; // soma signed das velocidades de rotor (acoplamento giroscopico)

float ax, ay, az, gx, gy, gz, mx, my, mz; // leituras IMU/mag (rad/s, g, uT)

// true so' quando USE_MAGNETOMETER esta ligado E o QMC5883P respondeu na init.
// O AHRS testa esta flag (nao USE_MAGNETOMETER) para nao alimentar o Madgwick
// com zeros caso o sensor falhe.
bool mag_ok = false;

// ===== Barometro BMP280 (somente leitura — nao realimenta o controle) =====
// O sensor converte a 26,3 Hz (ODR do preset "Indoor navigation"); ler decimado
// evita gastar I2C no loop de 192 Hz. A decimacao tem que ficar abaixo do ODR
// MINIMO de 23,1 Hz (Tabela 13 do datasheet, 43,2 ms de conversao no pior caso),
// senao parte das leituras repete a amostra anterior.
// Cada leitura vira uma amostra ">alt:" no Teleplot (~19 Hz, ~230 B/s).
const int BARO_DECIMATION_CYCLES = 10;  // ~19,2 Hz
bool  baro_ok           = false;
float baro_p0_pa        = 0.0f; // pressao de referencia (altura zero), fixada no setup
float baro_pressure_pa  = 0.0f;
float baro_temperature_c = 0.0f;
float baro_altitude_m   = 0.0f; // altura RELATIVA a baro_p0_pa

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ===== Estado compartilhado entre loop principal e SDRETask (protegido por mutex) =====
SemaphoreHandle_t matricesMutex;
TaskHandle_t      sdreTaskHandle;

float Ad[STATE_SIZE * STATE_SIZE];
float Bd[STATE_SIZE * CONTROL_SIZE];
float Qd[STATE_SIZE * STATE_SIZE];
float Rd[CONTROL_SIZE * CONTROL_SIZE];
float K_shared[CONTROL_SIZE * STATE_SIZE];
bool  new_K_available = false;
float state_shared[7]; // roll, pitch, yaw, p, q, r, omega_r

// Timing da SDRETask (escrito pela task, lido no loop em DEBUG_MODE — sem mutex)
volatile unsigned long sdre_t_updateMatrix = 0;
volatile unsigned long sdre_t_computeGains = 0;
volatile unsigned long sdre_t_total        = 0;

// ===== Matrizes do sistema (preenchidas em updateSystemMatrix) =====
float A[STATE_SIZE * STATE_SIZE] = {
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
    1/Ixx, 0, 0,
    0, 1/Iyy, 0,
    0, 0, 1/Izz
};
// Regra de Bryson sobre envelope realista de voo (nao limites mecanicos):
// Qii = 1/(max_estado_i)^2. Envelope maior baixa Q_ang e melhora razao P/D.
const float roll_max_rad  = 45.0f * DEG_TO_RAD;
const float pitch_max_rad = 45.0f * DEG_TO_RAD;
const float yaw_max_rad   = 90.0f * DEG_TO_RAD;
const float p_max         = 300.0f * DEG_TO_RAD;
const float q_max         = 300.0f * DEG_TO_RAD;
const float r_max         = 200.0f * DEG_TO_RAD;

const float Q_11 = 1.0f / (roll_max_rad * roll_max_rad);
const float Q_22 = 1.0f / (pitch_max_rad * pitch_max_rad);
const float Q_33 = 1.0f / (yaw_max_rad * yaw_max_rad);
const float Q_44 = 1.0f / (p_max * p_max);
const float Q_55 = 1.0f / (q_max * q_max);
const float Q_66 = 1.0f / (r_max * r_max);

float Q[STATE_SIZE * STATE_SIZE] = {
    Q_11, 0, 0, 0, 0, 0,
    0, Q_22, 0, 0, 0, 0,
    0, 0, Q_33, 0, 0, 0,
    0, 0, 0, Q_44, 0, 0,
    0, 0, 0, 0, Q_55, 0,
    0, 0, 0, 0, 0, Q_66
};

// Torques fisicos maximos (aproximados) — usados pela regra de Bryson em R:
const float perc_tau_x_max = 0.5f;
const float perc_tau_y_max = 0.5f;
const float perc_tau_z_max = 0.5f;

const float max_tau_roll  = (2.0f * MOTOR_B_COEFF * L_ARM * MAX_OMEGA * MAX_OMEGA) * perc_tau_x_max;
const float max_tau_pitch = (2.0f * MOTOR_B_COEFF * L_ARM * MAX_OMEGA * MAX_OMEGA) * perc_tau_y_max;
const float max_tau_yaw   = (4.0f * MOTOR_D_COEFF * MAX_OMEGA * MAX_OMEGA) * perc_tau_z_max;

// R_ii = 1 / (max_torque_i)^2
const float R_11 = 1.0f / (max_tau_roll  * max_tau_roll);
const float R_22 = 1.0f / (max_tau_pitch * max_tau_pitch);
const float R_33 = 1.0f / (max_tau_yaw   * max_tau_yaw);

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    R_11, 0, 0,
    0, R_22, 0,
    0, 0, R_33
};

// Controladores
AutoLQR sdreController(STATE_SIZE, CONTROL_SIZE);
PIDController pidController(STATE_SIZE, CONTROL_SIZE);

// Declaração dos sensores
Adafruit_MPU6050 mpu;

// Usando o mesmo barramento I2C para todos os sensores
// Wire já é definido e inicializado

Madgwick filter;

// Instâncias dos filtros Butterworth
BiquadFilter bq_ax, bq_ay, bq_az;
BiquadFilter bq_gx, bq_gy, bq_gz;

// Nyquist do loop (200 Hz) eh 100 Hz; cutoff abaixo disso evita quebrar o filtro.
const float perc_cutoff = 0.8f;
const float SENSOR_CUTOFF_HZ = ((1.0f / SAMPLING_TIME_S)/2.0f) * perc_cutoff;

// ===== Calibracao MPU6050 (obtida via test/calibrate_mpu.cpp) =====
float accel_offset_x = 0.428309f;
float accel_offset_y = 0.075853f;
float accel_offset_z = -0.016839f;
float gyro_offset_x = 0.114426f;
float gyro_offset_y = 0.016302f;
float gyro_offset_z = -0.001749f;

// ===== Calibracao QMC5883P (obter via test/calibrate_magnetometer.cpp) =====
const float MAG_OFFSET_X = -405.0f;
const float MAG_OFFSET_Y = -169.5f;
const float MAG_OFFSET_Z = -102.5f;
const float MAG_SCALE_X = 1.0404f;
const float MAG_SCALE_Y = 0.9786f;
const float MAG_SCALE_Z = 0.9833f;

MotorControl motors;
LEDControl   leds;
WiFiComm     wifiComm("ESP-DRONE", "12345678", 2390);
Telemetry    telemetry; // ~68 KB em RAM (1000 amostras x 68 B)

bool            remote_control_enabled = false;
CommanderPacket remote_command;

// Flags de seguranca
bool enable_motors          = false; // Ativado quando controle conecta
bool motors_armed_by_remote = false; // True apos primeiro comando recebido
bool skip_timing_sample     = false; // Ignora 1 amostra de tempo durante armamento
bool tilt_failsafe_latched  = false; // So libera com reset fisico do drone

// Failsafe de tilt: desarma antes da zona singular 1/cos(pitch) onde Ad explode.
const float MAX_SAFE_TILT_DEG = 80.0f;
const float MAX_SAFE_TILT_RAD = MAX_SAFE_TILT_DEG * DEG_TO_RAD;

float initial_yaw = 0.0f; // travado no setup() para referencia relativa
float yaw_setpoint = 0.0f; // heading acumulado: stick de yaw = taxa (rad/s), integrada a cada ciclo

// ===== Callbacks WiFi =====

void onRemoteCommandReceived(CommanderPacket cmd) {
    if (tilt_failsafe_latched) return;

    remote_command = cmd;
    remote_control_enabled = true;

    // Arma na primeira mensagem recebida
    if (!motors_armed_by_remote && !motors.isArmed()) {
        skip_timing_sample = true;
        motors.armMotors();
        motors_armed_by_remote = true;
        Serial.println("⚡ Motores ARMADOS pelo controle remoto!");
    }
}

void onClientConnected() {
    if (tilt_failsafe_latched) {
        Serial.println("⛔ FAILSAFE de inclinacao ativo: reinicie o drone para rearmar motores.");
        remote_control_enabled = false;
        enable_motors = false;
        leds.setSystemReady(false);
        leds.setUDPReceiving(false);
        return;
    }

    Serial.println("🎮 Controle remoto CONECTADO!");
    remote_control_enabled = false; // aguarda primeiro comando
    enable_motors = true;
    leds.setSystemReady(true);
}

void onClientDisconnected() {
    Serial.println("\n🔴 Controle remoto DESCONECTADO");
    remote_control_enabled = false;
    enable_motors = false;
    motors_armed_by_remote = false;

    motors.stopAllMotors();
    telemetry.saveToFile();

    remote_command = {0};

    leds.setSystemReady(false);
    leds.setUDPReceiving(false);

    Serial.println("⚠️  Motores DESARMADOS por segurança!");
}

// ===== Task FreeRTOS: recalcula Ad,Bd,Qd,Rd e resolve a DARE em paralelo ao loop =====
// Loop principal apenas le K_shared (sob mutex) — assim a Riccati (~5–10 ms) nao
// trava o ciclo de 5 ms dos sensores/motores. Stack grande pois SDA aloca matrizes.
void sdreTaskCode(void * parameter) {
    for (;;) {
        if (CONTROLLER_TYPE == 0) {
            float local_state[7];
            xSemaphoreTake(matricesMutex, portMAX_DELAY);
            memcpy(local_state, state_shared, sizeof(local_state));
            xSemaphoreGive(matricesMutex);

            unsigned long t0 = micros();
            updateSystemMatrix(local_state[0], local_state[1], local_state[2],
                               local_state[3], local_state[4], local_state[5],
                               local_state[6]);
            unsigned long t1 = micros();
            sdreController.computeGains();
            unsigned long t2 = micros();

            sdre_t_updateMatrix = t1 - t0;
            sdre_t_computeGains = t2 - t1;
            sdre_t_total        = t2 - t0;

            xSemaphoreTake(matricesMutex, portMAX_DELAY);
            sdreController.exportGains(K_shared);
            new_K_available = true;
            xSemaphoreGive(matricesMutex);
        }
        // Sem vTaskDelay: roda a maxima frequencia possivel no tempo ocioso
        // que o loop principal libera via vTaskDelay no final do seu ciclo.
    }
}

void setup() {
    // Mutex deve existir antes de qualquer xSemaphoreTake (evita assert fail em pxQueue)
    matricesMutex = xSemaphoreCreateMutex();

    if (USE_ASYNC_SDRE) {
        // Prioridade 0 (loop do Arduino roda em prio 1 — fica em background).
        // Stack 16 KB: SDA aloca varias matrizes temporarias 6x6.
        xTaskCreate(sdreTaskCode, "SDRETask", 16384, NULL, 0, &sdreTaskHandle);
    }

    Serial.begin(115200);
    delay(1000);

    // Persistencia de telemetria: ESP32 reseta quando o Serial Monitor abre, entao
    // salvamos o buffer em LittleFS ao desarmar e recarregamos no boot.
    if (LittleFS.begin(true)) {
        if (telemetry.loadFromFile()) {
            Serial.printf(">> Telemetria anterior carregada: %u amostras\n",
                          (unsigned)telemetry.size());
        } else {
            Serial.println(">> Nenhuma telemetria anterior em flash.");
        }
    } else {
        Serial.println("!! Falha ao montar LittleFS - telemetria nao sera persistida.");
    }

    // ===== Bateria =====
    leds.begin();

    if (leds.isCriticalBattery()) {
        Serial.println("❌ BATERIA CRÍTICA! Sistema não iniciará.");
        Serial.printf("   Tensão atual: %.2fV (mínimo: %.2fV)\n",
                      leds.getBatteryVoltage(), BATTERY_CRITICAL_VOLTAGE);
        leds.setLowPower(true);
        while (1) {
            leds.update();
            delay(100);
        }
    }

    if (leds.isLowBattery()) {
        Serial.println("⚠️  AVISO: Bateria baixa!");
        Serial.printf("   Tensão atual: %.2fV\n", leds.getBatteryVoltage());
        leds.setLowPower(true);
    }

    // ===== Sensores =====
    leds.setSensorsCalibration(true);

    Serial.println("Usando MPU6050");
    start_IMU_MPU6050(mpu);

    if (USE_MAGNETOMETER) {
        Serial.println("Inicializando QMC5883P (Magnetômetro)...");
        mag_ok = start_QMC5883P(Wire);
        if (mag_ok) {
            setQMC5883PCalibration(MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z,
                                   MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);
        } else {
            // Sem magnetometro o AHRS cai para 6-DOF em vez de alimentar o
            // Madgwick com zeros, que arruinaria a estimativa de atitude.
            Serial.println("⚠️  Magnetômetro indisponível - AHRS seguirá em 6-DOF.");
        }
    } else {
        Serial.println("⚠️  Magnetômetro DESABILITADO - usando apenas Accel+Gyro (6-DOF)");
    }

    // ===== Barometro (mesmo barramento I2C do MPU6050) =====
    // Falha aqui nao trava o boot: o barometro nao entra na malha de controle de
    // atitude, ao contrario do MPU6050.
    Serial.println("Inicializando BMP280 (Barômetro)...");
    baro_ok = start_BMP280(Wire);
    if (baro_ok) {
        // Referencia de altura zero: media com o drone parado no chao.
        // Bloqueia ~5,3 s (3 s de assentamento do IIR x16 + 50 * 45 ms).
        baro_p0_pa = bmp280_reference_pressure(50);
        if (baro_p0_pa > 0.0f) {
            Serial.printf("   Referência de altura zero: %.1f Pa (%.2f hPa)\n",
                          baro_p0_pa, baro_p0_pa / 100.0f);
        } else {
            baro_ok = false;
            Serial.println("⚠️  BMP280 parou de responder na referência - sem leitura de altura.");
        }
    } else {
        Serial.println("⚠️  BMP280 indisponível - sem leitura de altura.");
    }

    leds.setSensorsCalibration(false);

    // ===== Filtros Butterworth (anti-aliasing digital para Madgwick) =====
    const float fs = 1.0f / SAMPLING_TIME_S; // 200 Hz
    bq_ax.begin(SENSOR_CUTOFF_HZ, fs);
    bq_ay.begin(SENSOR_CUTOFF_HZ, fs);
    bq_az.begin(SENSOR_CUTOFF_HZ, fs);
    bq_gx.begin(SENSOR_CUTOFF_HZ, fs);
    bq_gy.begin(SENSOR_CUTOFF_HZ, fs);
    bq_gz.begin(SENSOR_CUTOFF_HZ, fs);

    // Primeira chamada de updateSystemMatrix em x=0 — discretiza A,B,Q,R e
    // propaga para o controlador SDRE.
    updateSystemMatrix(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // Sincrono reordenado: o controle usa o K do ciclo ANTERIOR, entao K_shared
    // precisa ser valido antes do 1o ciclo. Calcula um ganho inicial em x=0.
    // (No async, a SDRETask preenche K_shared em background.)
    if (CONTROLLER_TYPE == 0 && !USE_ASYNC_SDRE) {
        sdreController.computeGains();
        sdreController.exportGains(K_shared);
    }

    if (CONTROLLER_TYPE == 0) {
        Serial.println("🎯 Controlador: SDRE (State-Dependent Riccati Equation)");
    } else {
        Serial.println("🎯 Controlador: PID (Proportional-Integral-Derivative)");
        pidController.setRollGains(0.05f, 0.05f, 0.0f);
        pidController.setPitchGains(0.05f, 0.05f, 0.0f);
        pidController.setYawGains(0.01f, 0.0f, 0.0f);
        pidController.setIntegralLimits(1.0f, 1.0f, 0.5f);
        // Clamp ao maximo fisico: roll/pitch sao os eixos mais apertados (~0.016 N·m)
        pidController.setOutputLimits(-max_tau_roll, max_tau_roll);
        pidController.setSamplingTime(SAMPLING_TIME_S);
    }

    filter.begin(1.0f / SAMPLING_TIME_S);

    Serial.println("Estabilizando filtro para travar o Yaw inicial...");
    for (int i = 0; i < 10; i++) {
        read_MPU6050(mpu, ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);
        if (mag_ok) read_QMC5883P(mx, my, mz);
        delay(10);
    }

    // Convergencia do quaternion com gyro=0 (drone parado): 100k iteracoes
    // garantem que o Madgwick esteja em regime antes de travar initial_yaw.
    for (int i = 0; i < 100000; i++) {
        if (mag_ok) {
            filter.update(0.0f, 0.0f, 0.0f, ax, ay, az, mx, my, mz);
        } else {
            filter.updateIMU(0.0f, 0.0f, 0.0f, ax, ay, az);
        }
    }

    initial_yaw = filter.getYawRadians();
    Serial.printf("Yaw inicial travado em: %.2f deg\n", initial_yaw * RAD_TO_DEG);

    // ===== Motores =====
    motors.begin();
    motors.setThrottleLimits(0, 100); // Sem piso: motor para de vez (clamp simetrico no diferencial)
    motors.setOmegaSqLimits(0, MAX_OMEGA * MAX_OMEGA);

    // Calibracao de ESC: descomentar uma unica vez (consultar docs/MOTOR_CALIBRATION_GUIDE.md)
    // motors.calibrateESCs();

    Serial.println("⏸️  Motores em standby - aguardando controle remoto...");

    // ===== WiFi/UDP =====
    wifiComm.enableDebug(false);
    wifiComm.enableVerbose(false);

    if (wifiComm.begin()) {
        Serial.println("✅ WiFi/UDP iniciado com sucesso!");
        wifiComm.onCommandReceived(onRemoteCommandReceived);
        wifiComm.onClientConnected(onClientConnected);
        wifiComm.onClientDisconnected(onClientDisconnected);
    } else {
        Serial.println("❌ Falha ao iniciar WiFi/UDP!");
    }

    Serial.println("Sistema inicializado com sucesso!");
    Serial.println("--------------------------------------------------");
}

void loop(){
    unsigned long startTime = micros();
    
    // ===== PROFILING: Tempos parciais =====
    unsigned long t_leds, t_battery, t_wifi, t_mpu, t_mag, t_filter, t_angles, t_matrix, t_lqr, t_control_logic, t_motor_calc, t_motor_set;
    unsigned long t_checkpoint = micros();

    // ----- LEDs + bateria -----
    leds.update();
    t_leds = micros() - t_checkpoint;
    t_checkpoint = micros();

    // Bateria critica/baixa apenas acende LED — nao desarma motores em voo
    leds.setLowPower(leds.isCriticalBattery() || leds.isLowBattery());
    t_battery = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ----- WiFi/UDP -----
    wifiComm.update();
    t_wifi = micros() - t_checkpoint;
    t_checkpoint = micros();

    leds.setUDPReceiving(!tilt_failsafe_latched && wifiComm.isClientConnected());

    // ----- Sensores + Butterworth -----
    // Leitura crua (burst read via Wire): ~1.8x mais rapida que mpu.getEvent() da
    // Adafruit, com escalas identicas. Init/config continua pela lib (start_IMU_MPU6050).
    read_MPU6050_raw(ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);

    ax = bq_ax.update(ax);
    ay = bq_ay.update(ay);
    az = bq_az.update(az);
    gx = bq_gx.update(gx);
    gy = bq_gy.update(gy);
    gz = bq_gz.update(gz);

    t_mpu = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ----- Barometro (decimado, so' leitura) -----
    // Nao realimenta o controle: apenas atualiza a altura relativa e imprime.
    if (baro_ok) {
        static int baro_cycle = 0;
        if (++baro_cycle >= BARO_DECIMATION_CYCLES) {
            baro_cycle = 0;
            read_BMP280(baro_pressure_pa, baro_temperature_c);
            baro_altitude_m = bmp280_altitude(baro_pressure_pa, baro_p0_pa,
                                              baro_temperature_c);

            // Formato Teleplot (">serie:valor"), o mesmo do stream de atitude no
            // fim do loop — as duas saidas convivem no mesmo grafico.
            // availableForWrite: se o ring buffer do USB CDC estiver cheio (host
            // parou de drenar), Serial.write bloquearia ate 100 ms e estouraria o
            // periodo de 5,2 ms do loop. Preferimos perder a amostra.
            if (PRINT_TELEMETRY && Serial.availableForWrite() >= 32) {
                Serial.printf(">alt:%.2f\n", baro_altitude_m);
            }
        }
        t_checkpoint = micros(); // baro fora do profiling de t_mag/t_filter
    }

    // ----- Magnetometro (se habilitado) + Madgwick -----
    // Adafruit_MPU6050 retorna gyro em rad/s; Madgwick espera deg/s.
    if (mag_ok) {
        read_QMC5883P(mx, my, mz);
        t_mag = micros() - t_checkpoint;
        t_checkpoint = micros();
        filter.update(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG,
                      ax, ay, az, mx, my, mz);
    } else {
        t_mag = 0;
        filter.updateIMU(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG,
                         ax, ay, az);
    }
    t_filter = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ----- Angulos de Euler (yaw relativo ao initial_yaw, normalizado em [-π,π]) -----
    float roll  = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    float yaw   = filter.getYawRadians() - initial_yaw;
    while (yaw >  PI) yaw -= 2.0f * PI;
    while (yaw < -PI) yaw += 2.0f * PI;

    // ----- Failsafe de tilt (so reseta com reboot) -----
    if (!tilt_failsafe_latched &&
        (fabsf(roll) > MAX_SAFE_TILT_RAD || fabsf(pitch) > MAX_SAFE_TILT_RAD)) {
        tilt_failsafe_latched = true;
        remote_control_enabled = false;
        enable_motors = false;
        motors_armed_by_remote = false;

        motors.stopAllMotors();
        telemetry.saveToFile();

        remote_command = {0};

        leds.setSystemReady(false);
        leds.setUDPReceiving(false);

        Serial.printf("\n🚨 FAILSAFE: inclinacao acima de %.0f graus detectada!\n", MAX_SAFE_TILT_DEG);
        Serial.printf("   Roll: %.2f deg | Pitch: %.2f deg\n",
                      roll * RAD_TO_DEG, pitch * RAD_TO_DEG);
        Serial.println("   Motores desligados. Reinicie o drone para rearmar.");
    }

    float p = gx, q = gy, r = gz;
    float z_measurement[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    t_angles = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ----- SDRE: publica estado para a task assincrona OU calcula sincronamente -----
    if (USE_ASYNC_SDRE) {
        xSemaphoreTake(matricesMutex, portMAX_DELAY);
        state_shared[0] = roll;  state_shared[1] = pitch; state_shared[2] = yaw;
        state_shared[3] = p;     state_shared[4] = q;     state_shared[5] = r;
        state_shared[6] = omega_r;
        xSemaphoreGive(matricesMutex);

        t_matrix = micros() - t_checkpoint;
        t_checkpoint = micros();
        t_lqr    = 0; // computado na SDRETask
    } else {
        // SINCRONO REORDENADO: o SDRE NAO roda aqui. Se rodasse, ficaria no caminho
        // sensor->atuador, somando ~4ms de atraso de transporte -> oscilacao. O controle
        // abaixo usa o K do ciclo ANTERIOR (ja em K_shared); o novo K e calculado ao FINAL
        // do loop (apos aplicar os motores), com latencia de atuacao ~0 (como no async).
        t_matrix = micros() - t_checkpoint;
        t_checkpoint = micros();
        t_lqr = 0;
    }

    // ----- Logica de controle: extrai setpoint do comando remoto -----
    float phi_desired = 0, theta_desired = 0, yaw_desired = 0, thrust = 0;

    if (remote_control_enabled && wifiComm.isClientConnected()) {
        phi_desired   = remote_command.roll  * DEG_TO_RAD;
        theta_desired = remote_command.pitch * DEG_TO_RAD;
        // yaw do stick e taxa (rad/s), nao angulo (convencao CRTP: roll/pitch=angulo, yaw=taxa).
        // Integra para formar o heading: stick centralizado -> taxa=0 -> mantem o angulo atual.
        yaw_setpoint += (remote_command.yaw * DEG_TO_RAD) * SAMPLING_TIME_S;
        while (yaw_setpoint >  PI) yaw_setpoint -= 2.0f * PI;
        while (yaw_setpoint < -PI) yaw_setpoint += 2.0f * PI;
        yaw_desired = yaw_setpoint;

        // Desnormaliza yaw_desired ao redor do yaw atual (erro em [-π,π]). Sem isso,
        // SDRE faz x[2]-r[2] linear e o drone gira pela rota longa em setpoints grandes.
        float yaw_err = yaw_desired - yaw;
        while (yaw_err >  PI) yaw_err -= 2.0f * PI;
        while (yaw_err < -PI) yaw_err += 2.0f * PI;
        yaw_desired = yaw + yaw_err;

        thrust = (remote_command.thrust / 65000.0f) * MAX_THRUST * 4.0f; // soma dos 4 motores
    }
    t_control_logic = micros() - t_checkpoint;
    t_checkpoint = micros();

    float x[STATE_SIZE] = {roll, pitch, yaw, p, q, r};
    float ref[3]        = {phi_desired, theta_desired, yaw_desired};
    float u[CONTROL_SIZE];

    if (CONTROLLER_TYPE == 0) {
        // u = -K*x + Kr*ref, com Kr = -K[:,:CONTROL_SIZE] (compensacao de regime estacionario)
        memset(u, 0, CONTROL_SIZE * sizeof(float));

        xSemaphoreTake(matricesMutex, portMAX_DELAY);
        float K_local[CONTROL_SIZE * STATE_SIZE];
        memcpy(K_local, K_shared, sizeof(K_local));
        xSemaphoreGive(matricesMutex);

        for (int i = 0; i < CONTROL_SIZE; i++) {
            for (int j = 0; j < STATE_SIZE; j++) {
                u[i] -= K_local[i * STATE_SIZE + j] * x[j];
                if (j < CONTROL_SIZE) {
                    u[i] += (-K_local[i * STATE_SIZE + j]) * ref[j];
                }
            }
        }
    } else {
        pidController.updateState(x);
        pidController.updateReference(ref);
        pidController.calculateControl(u);
    }

    // ----- Alocacao de torques -> omega^2 dos 4 motores -----
    float w1_sq, w2_sq, w3_sq, w4_sq;
    calculateMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF, L_ARM,
                          w1_sq, w2_sq, w3_sq, w4_sq);

    // Acoplamento giroscopico (Ir << I): sign_i = +1 CCW, -1 CW. Σ sign_i · ω_i.
    // M1 (FR, CW), M2 (RR, CCW), M3 (RL, CW), M4 (FL, CCW).
    omega_r = -sqrtf(w1_sq) + sqrtf(w2_sq) - sqrtf(w3_sq) + sqrtf(w4_sq);
    t_motor_calc = micros() - t_checkpoint;
    t_checkpoint = micros();

    // ----- Aplica nos motores -----
    if (enable_motors && motors.isArmed()) {
        motors.setMotorsFromOmegaSq(w1_sq, w2_sq, w3_sq, w4_sq);
    } else {
        motors.stopAllMotors();
    }
    t_motor_set = micros() - t_checkpoint;

    // ----- SINCRONO: calcula o SDRE para o PROXIMO ciclo, FORA do caminho de atuacao -----
    // Os motores ja foram aplicados acima com o K do ciclo anterior, entao o tempo da
    // Riccati (~3.7ms) nao atrasa a atuacao — mesmo padrao de latencia do async.
    // (Em sync a SDRETask nao existe, entao escreve K_shared direto sem mutex.)
    if (!USE_ASYNC_SDRE && CONTROLLER_TYPE == 0) {
        unsigned long t_start = micros();
        updateSystemMatrix(roll, pitch, yaw, p, q, r, omega_r);
        sdre_t_updateMatrix = micros() - t_start;

        t_start = micros();
        sdreController.computeGains();
        sdreController.exportGains(K_shared);
        sdre_t_computeGains = micros() - t_start;
        t_lqr = sdre_t_updateMatrix + sdre_t_computeGains;  // telemetria
    }

    // ----- Telemetria em RAM (somente quando armado, decimada por TELEMETRY_DECIMATION_CYCLES) -----
    // Custo: ~1 us — apenas escritas em RAM, sem printf nem I/O.
    if (motors.isArmed()) {
        static uint32_t telemetry_cycle = 0;
        if (telemetry_cycle % TELEMETRY_DECIMATION_CYCLES == 0) {
            // Referencia EFETIVA que o drone segue: como Kr = -K[:,:3], a malha rastreia
            // x_ang -> -phi_desired. Salvamos o negativo para o grafico casar com o angulo
            // medido (roll/pitch/yaw). Apenas telemetria — nao afeta o controle.
            telemetry.log(millis(),
                          roll, pitch, yaw,
                          -phi_desired, -theta_desired, -yaw_desired,
                          p, q, r,
                          u[0], u[1], u[2],
                          w1_sq, w2_sq, w3_sq, w4_sq);
        }
        telemetry_cycle++;
    } else {
        // Desarmado: aceita comandos serial — 'D' dump CSV, 'R' reset buffer.
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'D' || c == 'd') {
                telemetry.dumpCSV(Serial);
            } else if (c == 'R' || c == 'r') {
                telemetry.reset();
                telemetry.saveToFile(); // persiste reset (evita restaurar dados antigos no boot)
                Serial.println(">> Buffer de telemetria resetado.");
            }
        }
    }

    // ----- Mantem periodo de amostragem exato (5 ms) -----
    unsigned long processingTime = micros() - startTime;

    if (processingTime < LOOP_PERIOD_US) {
        unsigned long timeLeft = LOOP_PERIOD_US - processingTime;
        // Cede CPU para SDRETask em granularidade de ms…
        vTaskDelay(timeLeft / 1000);
        // …e finaliza com busy-wait para o periodo exato em us.
        while (micros() - startTime < LOOP_PERIOD_US) { /* spin */ }
    }

    unsigned long loopTime = micros() - startTime;

    // ----- Estatisticas de timing -----
    static unsigned long maxTime = 0;
    static unsigned long totalTime = 0;
    static unsigned long loopCount = 0;
    static unsigned long prev_ms = 0;
    static unsigned long last_print_time = 0; // custo dos prints do ultimo ciclo de debug

    bool skipThisSample = skip_timing_sample;
    if (skipThisSample) skip_timing_sample = false;

    bool newMaxTime = false;
    if (!skipThisSample) {
        loopCount++;
        totalTime += loopTime;
        if (loopTime > maxTime) {
            maxTime = loopTime;
            newMaxTime = true;
        }
    }

    float avgTime = (loopCount > 0) ? ((float)totalTime / loopCount) : 0.0f;
    
    if (PRINT_TELEMETRY) {
        Serial.printf(">roll:%.2f\n>pitch:%.2f\n>yaw:%.2f\n>p:%.2f\n>q:%.2f\n>r:%.2f\n",
                      roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG,
                      p * RAD_TO_DEG, q * RAD_TO_DEG, r * RAD_TO_DEG);
    }

    if (DEBUG_MODE) {
        // ===== MODO DEBUG: Prints detalhados a cada 1 segundo =====
        if (micros() >= prev_ms + 1000000) {
            unsigned long print_start = micros();
            
            // Calcula tempo total do loop incluindo prints anteriores
            unsigned long total_with_prints = processingTime + last_print_time;
            
            Serial.println("\n========== STATUS DO SISTEMA ==========");
            
            // ===== PROFILING DE DESEMPENHO =====
            Serial.println("\n⏱️  PROFILING DE EXECUÇÃO:");
            Serial.printf("   LEDs:            %4lu μs (%5.1f%%)\n", t_leds, (t_leds * 100.0f) / total_with_prints);
            Serial.printf("   Bateria:         %4lu μs (%5.1f%%)\n", t_battery, (t_battery * 100.0f) / total_with_prints);
            Serial.printf("   WiFi/UDP:        %4lu μs (%5.1f%%)\n", t_wifi, (t_wifi * 100.0f) / total_with_prints);
            Serial.printf("   Leitura MPU:     %4lu μs (%5.1f%%)\n", t_mpu, (t_mpu * 100.0f) / total_with_prints);
            Serial.printf("   Leitura Mag:     %4lu μs (%5.1f%%)\n", t_mag, (t_mag * 100.0f) / total_with_prints);
            Serial.printf("   Filtro Madgwick: %4lu μs (%5.1f%%)\n", t_filter, (t_filter * 100.0f) / total_with_prints);
            Serial.printf("   Cálc. Ângulos:   %4lu μs (%5.1f%%)\n", t_angles, (t_angles * 100.0f) / total_with_prints);
            Serial.printf("   Matriz Sistema:  %4lu μs (%5.1f%%)\n", t_matrix, (t_matrix * 100.0f) / total_with_prints);
            Serial.printf("   LQR (Ganhos):    %4lu μs (%5.1f%%)\n", t_lqr, (t_lqr * 100.0f) / total_with_prints);
            Serial.printf("   Lógica Controle: %4lu μs (%5.1f%%)\n", t_control_logic, (t_control_logic * 100.0f) / total_with_prints);
            Serial.printf("   Cálc. Omega²:    %4lu μs (%5.1f%%)\n", t_motor_calc, (t_motor_calc * 100.0f) / total_with_prints);
            Serial.printf("   Set Motores:     %4lu μs (%5.1f%%)\n", t_motor_set, (t_motor_set * 100.0f) / total_with_prints);
            Serial.printf("   Prints (ant.):   %4lu μs (%5.1f%%)\n", last_print_time, (last_print_time * 100.0f) / total_with_prints);
            Serial.println("   -----------------------------------");
            unsigned long sum_profiled = t_leds + t_battery + t_wifi + t_mpu + t_mag + t_filter + 
                                         t_angles + t_matrix + t_lqr + t_control_logic + 
                                         t_motor_calc + t_motor_set + last_print_time;
            unsigned long overhead = (total_with_prints > sum_profiled) ? (total_with_prints - sum_profiled) : 0;
            Serial.printf("   Soma Medida:     %4lu μs (%5.1f%%)\n", sum_profiled, (sum_profiled * 100.0f) / total_with_prints);
            Serial.printf("   Overhead/Outros: %4lu μs (%5.1f%%)\n", overhead, (overhead * 100.0f) / total_with_prints);
            
            // Tempo de execução
            Serial.println("\n📊 ESTATÍSTICAS DE TEMPO:");
            Serial.printf("   Tempo_Loop: %lu μs\n", loopTime);
            Serial.printf("   Tempo_Processamento: %lu μs\n", processingTime);
            Serial.printf("   Tempo_Maximo: %lu μs\n", maxTime);
            Serial.printf("   Tempo_Medio: %.2f μs\n", avgTime);

            if (CONTROLLER_TYPE == 0) {
                Serial.println("\n⚙️  SDRE TASK (fora do loop de 5ms):");
                Serial.printf("   updateSystemMatrix: %4lu μs\n", sdre_t_updateMatrix);
                Serial.printf("   computeGains (DARE): %4lu μs\n", sdre_t_computeGains);
                // Iteracoes/residuo do SDA: se iter bater no teto (30) o solver esta
                // estagnando e o tempo esta sendo gasto a toa — ver criterio de parada.
                Serial.printf("   SDA: %d iteracoes, residuo %.2e\n",
                              sdreController.getLastIterations(),
                              sdreController.getLastResidual());
                Serial.printf("   Total SDRE:          %4lu μs  (rodando a ~20 Hz em task separada)\n", sdre_t_total);
            }
            
            // Status de controle
            if (tilt_failsafe_latched) {
                Serial.println("\n🚨 MODO: FAILSAFE TRAVADO");
                Serial.println("   Motivo: roll/pitch acima de 60 graus");
                Serial.println("   Acao: motores desligados ate reset total do drone");
            } else if (remote_control_enabled && wifiComm.isClientConnected()) {
                Serial.println("\n🎮 MODO: CONTROLE REMOTO ATIVO");
                Serial.println("   Comandos Joystick:");
                Serial.printf("     Roll:   %+.3f\n", remote_command.roll);
                Serial.printf("     Pitch:  %+.3f\n", remote_command.pitch);
                Serial.printf("     Yaw:    %+.3f\n", remote_command.yaw);
                Serial.printf("     Thrust: %d (%.1f%%)\n", remote_command.thrust, 
                             (remote_command.thrust / 65000.0f) * 100.0f);
                Serial.println("   Setpoints (rad):");
                Serial.printf("     φ_desired: %+.4f\n", phi_desired);
                Serial.printf("     θ_desired: %+.4f\n", theta_desired);
                Serial.printf("     ψ_desired: %+.4f\n", yaw_desired);
                Serial.printf("     Thrust: %.4f N\n", thrust);
            } else {
                Serial.println("\n🤖 MODO: AUTÔNOMO");
            }
            
            // Estados do sistema
            Serial.println();
            displayStates(const_cast<float*>(z_measurement));
            
            // Dados do magnetômetro (se habilitado)
            if (mag_ok) {
                Serial.println("\n🧭 MAGNETÔMETRO (QMC5883P):");
                Serial.printf("   Campo Magnético: X=%+.2f  Y=%+.2f  Z=%+.2f μT\n", mx, my, mz);
                float heading = atan2(my, mx) * RAD_TO_DEG;
                if (heading < 0) heading += 360.0f;
                Serial.printf("   Azimute (Heading): %.1f°\n", heading);
                // Intensidade total do campo magnético
                float mag_intensity = sqrt(mx*mx + my*my + mz*mz);
                Serial.printf("   Intensidade Total: %.2f μT\n", mag_intensity);
            } else {
                Serial.println("\n🧭 MAGNETÔMETRO: DESABILITADO (6-DOF mode)");
            }
            
            // Sinais de controle
            displayControlSignals(u, thrust);
            
            // Omega dos motores
            displayMotorOmegaSq(thrust, u, MOTOR_B_COEFF, MOTOR_D_COEFF, L_ARM);
            
            // Valores dos motores
            motors.printMotorValues();
            
            // Status dos motores
            Serial.print("\n🔧 Motores: ");
            if (motors.isArmed()) {
                Serial.println("ARMADOS ✅");
            } else {
                Serial.println("DESARMADOS ⏸️");
            }
            
            // Status WiFi
            if (wifiComm.isClientConnected()) {
                Serial.print("📡 WiFi: CONECTADO | Pacotes: ");
                Serial.print(wifiComm.getPacketCount());
                Serial.print(" | Cliente: ");
                Serial.println(wifiComm.getClientIP());
            } else {
                Serial.println("📡 WiFi: Aguardando conexão...");
            }

            // Exibe a Matriz P (Solução da Equação de Riccati) - apenas para SDRE
            if (CONTROLLER_TYPE == 0) {
                Serial.println("\n📊 Matriz P (Solução de Riccati):");
                const float* P_ptr = sdreController.getRicattiSolution();
                if (P_ptr) {
                    for (int i = 0; i < STATE_SIZE; i++) {
                        Serial.print("   | ");
                        for (int j = 0; j < STATE_SIZE; j++) {
                            Serial.print(P_ptr[i * STATE_SIZE + j], 4);
                            if (j < STATE_SIZE - 1) Serial.print("\t");
                        }
                        Serial.println(" |");
                    }
                }
                
                // Exibe a Matriz K (Ganhos de Controle) - apenas para SDRE
                Serial.println("\n🎮 Matriz K (Ganhos de Controle):");
                float K_current[CONTROL_SIZE * STATE_SIZE];
                if (sdreController.exportGains(K_current)) {
                    for (int i = 0; i < CONTROL_SIZE; i++) {
                        Serial.print("   | ");
                        for (int j = 0; j < STATE_SIZE; j++) {
                            Serial.print(K_current[i * STATE_SIZE + j], 4);
                            if (j < STATE_SIZE - 1) Serial.print("\t");
                        }
                        Serial.println(" |");
                    }
                }
            } else {
                // Para PID, exibe os ganhos de forma mais apropriada
                Serial.println("\n📊 Ganhos PID:");
                Serial.println("   Eixo     |   Kp    |   Ki    |   Kd    |");
                Serial.println("   ---------|---------|---------|---------|");
                float kp, ki, kd;
                pidController.getGains(0, kp, ki, kd);  // Roll
                Serial.printf("   Roll     |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                pidController.getGains(1, kp, ki, kd);  // Pitch
                Serial.printf("   Pitch    |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                pidController.getGains(2, kp, ki, kd);  // Yaw
                Serial.printf("   Yaw      |  %5.2f  |  %5.2f  |  %5.2f  |\n", kp, ki, kd);
                
                Serial.println("\n📈 Estado do PID:");
                float roll_int, pitch_int, yaw_int;
                pidController.getIntegrals(roll_int, pitch_int, yaw_int);
                Serial.printf("   Integrais: Roll=%+.4f  Pitch=%+.4f  Yaw=%+.4f\n", 
                             roll_int, pitch_int, yaw_int);
                float roll_err, pitch_err, yaw_err;
                pidController.getErrors(roll_err, pitch_err, yaw_err);
                Serial.printf("   Erros:     Roll=%+.4f  Pitch=%+.4f  Yaw=%+.4f\n", 
                             roll_err, pitch_err, yaw_err);
            }

            Serial.println("========================================\n");
            
            // Mede o tempo que os prints custaram
            last_print_time = micros() - print_start;
            Serial.printf("⏱️  Tempo dos Prints: %lu μs\n", last_print_time);
            Serial.printf("========================================\n");        

            
            prev_ms = micros();
        }
    } else {
        // ===== MODO SERIAL PLOTTER: Printa apenas quando há novo máximo =====
        if (newMaxTime) {
            Serial.print("Atual:");
            Serial.print(loopTime);
            Serial.print(",Media:");
            Serial.print(avgTime, 1);
            Serial.print(",Max:");
            Serial.println(maxTime);
        }
    }
}

// Atualiza A(x), B, Q, R e suas versoes discretizadas Ad,Bd,Qd,Rd a partir do
// estado atual (modelo SDRE de atitude do quadricoptero, configuracao X-quad).
//
// Versao esparsa: A tem 11 nao-nulos; B tem 3 nao-nulos (B[3,0]=1/Ixx,
// B[4,1]=1/Iyy, B[5,2]=1/Izz); Q diagonal. Ad/Bd/Qd sao montados diretamente
// pelas formulas analiticas (Taylor 2a/3a ordem em dt), evitando ~700 muls
// que matrixMultiply generico custaria.
void updateSystemMatrix(float roll, float pitch, float yaw, float p, float q, float r, float omega_r) {

    static const float inv_Ixx          = 1.0f / Ixx;
    static const float inv_Iyy          = 1.0f / Iyy;
    static const float inv_Izz          = 1.0f / Izz;
    static const float Iyy_Izz_over_Ixx = (Iyy - Izz) / Ixx;
    static const float Izz_Ixx_over_Iyy = (Izz - Ixx) / Iyy;
    static const float Ixx_Iyy_over_Izz = (Ixx - Iyy) / Izz;
    static const float Ir_over_Ixx      = Ir / Ixx;
    static const float Ir_over_Iyy      = Ir / Iyy;
    static const float dt               = SAMPLING_TIME_S;
    static const float dt2_2            = dt * dt * 0.5f;

    float sR, cR, sP, cP;
    sincosf(roll,  &sR, &cR);
    sincosf(pitch, &sP, &cP);
    const float inv_cP = 1.0f / cP;
    const float tP     = sP * inv_cP;

    // 11 entradas nao-nulas de A.
    const float A03 = 1.0f;
    const float A04 = sR * tP;
    const float A05 = cR * tP;
    const float A14 = cR;
    const float A15 = -sR;
    const float A24 = sR * inv_cP;
    const float A25 = cR * inv_cP;
    const float A34 = Iyy_Izz_over_Ixx * r - Ir_over_Ixx * omega_r;
    const float A43 = Ir_over_Iyy * omega_r;
    const float A45 = Izz_Ixx_over_Iyy * p;
    const float A54 = Ixx_Iyy_over_Izz * p;

    // Reflete no array global A (apenas posicoes variaveis - restante ja eh 0).
    A[0 * STATE_SIZE + 3] = A03;
    A[0 * STATE_SIZE + 4] = A04;
    A[0 * STATE_SIZE + 5] = A05;
    A[1 * STATE_SIZE + 4] = A14;
    A[1 * STATE_SIZE + 5] = A15;
    A[2 * STATE_SIZE + 4] = A24;
    A[2 * STATE_SIZE + 5] = A25;
    A[3 * STATE_SIZE + 4] = A34;
    A[3 * STATE_SIZE + 5] = 0.0f;
    A[4 * STATE_SIZE + 3] = A43;
    A[4 * STATE_SIZE + 5] = A45;
    A[5 * STATE_SIZE + 3] = 0.0f;
    A[5 * STATE_SIZE + 4] = A54;

    // ===== A^2 esparso: 14 entradas (Σ_k A[i,k]·A[k,j], k em {3,4,5}) =====
    const float A2_03 = A04 * A43;
    const float A2_04 = A34 + A05 * A54;
    const float A2_05 = A04 * A45;
    const float A2_13 = A14 * A43;
    const float A2_14 = A15 * A54;
    const float A2_15 = A14 * A45;
    const float A2_23 = A24 * A43;
    const float A2_24 = A25 * A54;
    const float A2_25 = A24 * A45;
    const float A2_33 = A34 * A43;
    const float A2_35 = A34 * A45;
    const float A2_44 = A43 * A34 + A45 * A54;
    const float A2_53 = A54 * A43;
    const float A2_55 = A54 * A45;

    // ===== Ad = I + A*dt + A^2*dt^2/2 =====
    memset(Ad, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
    Ad[0 * STATE_SIZE + 0] = 1.0f;
    Ad[1 * STATE_SIZE + 1] = 1.0f;
    Ad[2 * STATE_SIZE + 2] = 1.0f;
    Ad[3 * STATE_SIZE + 3] = 1.0f + A2_33 * dt2_2;
    Ad[4 * STATE_SIZE + 4] = 1.0f + A2_44 * dt2_2;
    Ad[5 * STATE_SIZE + 5] = 1.0f + A2_55 * dt2_2;

    Ad[0 * STATE_SIZE + 3] = A03 * dt + A2_03 * dt2_2;
    Ad[0 * STATE_SIZE + 4] = A04 * dt + A2_04 * dt2_2;
    Ad[0 * STATE_SIZE + 5] = A05 * dt + A2_05 * dt2_2;
    Ad[1 * STATE_SIZE + 3] =            A2_13 * dt2_2;
    Ad[1 * STATE_SIZE + 4] = A14 * dt + A2_14 * dt2_2;
    Ad[1 * STATE_SIZE + 5] = A15 * dt + A2_15 * dt2_2;
    Ad[2 * STATE_SIZE + 3] =            A2_23 * dt2_2;
    Ad[2 * STATE_SIZE + 4] = A24 * dt + A2_24 * dt2_2;
    Ad[2 * STATE_SIZE + 5] = A25 * dt + A2_25 * dt2_2;
    Ad[3 * STATE_SIZE + 4] = A34 * dt;
    Ad[3 * STATE_SIZE + 5] =            A2_35 * dt2_2;
    Ad[4 * STATE_SIZE + 3] = A43 * dt;
    Ad[4 * STATE_SIZE + 5] = A45 * dt;
    Ad[5 * STATE_SIZE + 3] =            A2_53 * dt2_2;
    Ad[5 * STATE_SIZE + 4] = A54 * dt;

    // ===== Bd = B*dt + A*B*dt^2/2; B esparso => (A*B)[i,j] = A[i,3+j]·(1/I_jj) =====
    memset(Bd, 0, sizeof(float) * STATE_SIZE * CONTROL_SIZE);
    Bd[0 * CONTROL_SIZE + 0] = A03 * inv_Ixx * dt2_2;
    Bd[0 * CONTROL_SIZE + 1] = A04 * inv_Iyy * dt2_2;
    Bd[0 * CONTROL_SIZE + 2] = A05 * inv_Izz * dt2_2;
    Bd[1 * CONTROL_SIZE + 1] = A14 * inv_Iyy * dt2_2;
    Bd[1 * CONTROL_SIZE + 2] = A15 * inv_Izz * dt2_2;
    Bd[2 * CONTROL_SIZE + 1] = A24 * inv_Iyy * dt2_2;
    Bd[2 * CONTROL_SIZE + 2] = A25 * inv_Izz * dt2_2;
    Bd[3 * CONTROL_SIZE + 0] = inv_Ixx * dt;
    Bd[3 * CONTROL_SIZE + 1] = A34 * inv_Iyy * dt2_2;
    Bd[4 * CONTROL_SIZE + 0] = A43 * inv_Ixx * dt2_2;
    Bd[4 * CONTROL_SIZE + 1] = inv_Iyy * dt;
    Bd[4 * CONTROL_SIZE + 2] = A45 * inv_Izz * dt2_2;
    Bd[5 * CONTROL_SIZE + 1] = A54 * inv_Iyy * dt2_2;
    Bd[5 * CONTROL_SIZE + 2] = inv_Izz * dt;

    // ===== Qd = Q*dt + (A^T·Q + Q·A)*dt^2/2; Q diagonal, simetrico =====
    // (A^T·Q + Q·A)[i,j] = A[j,i]·Q[j,j] + Q[i,i]·A[i,j].
    memset(Qd, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
    Qd[0 * STATE_SIZE + 0] = Q_11 * dt;
    Qd[1 * STATE_SIZE + 1] = Q_22 * dt;
    Qd[2 * STATE_SIZE + 2] = Q_33 * dt;
    Qd[3 * STATE_SIZE + 3] = Q_44 * dt;
    Qd[4 * STATE_SIZE + 4] = Q_55 * dt;
    Qd[5 * STATE_SIZE + 5] = Q_66 * dt;

    const float q03 = Q_11 * A03 * dt2_2; // A[3,0]=0
    const float q04 = Q_11 * A04 * dt2_2; // A[4,0]=0
    const float q05 = Q_11 * A05 * dt2_2; // A[5,0]=0
    const float q14 = Q_22 * A14 * dt2_2; // A[4,1]=0
    const float q15 = Q_22 * A15 * dt2_2; // A[5,1]=0
    const float q24 = Q_33 * A24 * dt2_2; // A[4,2]=0
    const float q25 = Q_33 * A25 * dt2_2; // A[5,2]=0
    const float q34 = (A43 * Q_55 + Q_44 * A34) * dt2_2;
    const float q45 = (A54 * Q_66 + Q_55 * A45) * dt2_2;

    Qd[0 * STATE_SIZE + 3] = q03; Qd[3 * STATE_SIZE + 0] = q03;
    Qd[0 * STATE_SIZE + 4] = q04; Qd[4 * STATE_SIZE + 0] = q04;
    Qd[0 * STATE_SIZE + 5] = q05; Qd[5 * STATE_SIZE + 0] = q05;
    Qd[1 * STATE_SIZE + 4] = q14; Qd[4 * STATE_SIZE + 1] = q14;
    Qd[1 * STATE_SIZE + 5] = q15; Qd[5 * STATE_SIZE + 1] = q15;
    Qd[2 * STATE_SIZE + 4] = q24; Qd[4 * STATE_SIZE + 2] = q24;
    Qd[2 * STATE_SIZE + 5] = q25; Qd[5 * STATE_SIZE + 2] = q25;
    Qd[3 * STATE_SIZE + 4] = q34; Qd[4 * STATE_SIZE + 3] = q34;
    Qd[4 * STATE_SIZE + 5] = q45; Qd[5 * STATE_SIZE + 4] = q45;

    // ===== Rd = R*dt + (B^T·Q·B)*dt^3/3 — B esparso + Q diag => Rd diagonal =====
    static const float dt3_over_3 = dt * dt * dt / 3.0f;
    memset(Rd, 0, sizeof(float) * CONTROL_SIZE * CONTROL_SIZE);
    Rd[0 * CONTROL_SIZE + 0] = R_11 * dt + (Q_44 * inv_Ixx * inv_Ixx) * dt3_over_3;
    Rd[1 * CONTROL_SIZE + 1] = R_22 * dt + (Q_55 * inv_Iyy * inv_Iyy) * dt3_over_3;
    Rd[2 * CONTROL_SIZE + 2] = R_33 * dt + (Q_66 * inv_Izz * inv_Izz) * dt3_over_3;

    if (CONTROLLER_TYPE == 0) {
        sdreController.setStateMatrix(Ad);
        sdreController.setInputMatrix(Bd);
        sdreController.setCostMatrices(Qd, Rd);
    }
}

