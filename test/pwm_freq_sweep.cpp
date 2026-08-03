// =====================================================================
// VARREDURA DE FREQUENCIA PWM DOS MOTORES
//
// Objetivo:
//   Medir vibracao induzida pelos motores em diferentes freqs de PWM
//   e indicar a freq que minimiza ruido no MPU6050. Util para escolher
//   PWM_FREQUENCY em lib/MotorControl/MotorControl.h
//
// PROCEDIMENTO (LEIA ANTES DE EXECUTAR):
//   1. Comente todo o conteudo de src/main.cpp (ou renomeie p/ main.cpp.bak)
//   2. Prenda o drone com fita/peso em superficie firme. Helices ABERTAS.
//      (a vibracao das helices e justamente o que queremos medir)
//   3. Bateria carregada. Drone APONTANDO PRA CIMA, longe de objetos.
//   4. pio run -t upload
//   5. Abrir monitor serial (115200 baud)
//   6. Quando aparecer prompt, mande qualquer caractere pra iniciar
//   7. Aguarde ~30-40 segundos. CSV completo + ranking impresso no fim.
//
// METRICA:
//   score = ||sigma_gyro||_dps + 5 * ||sigma_acc||_g
//   - sigma e std-dev por eixo, computado por Welford (numericamente estavel)
//   - gyro em dps (escolhido pq controle de atitude usa dps)
//   - peso 5 no acc pra equilibrar magnitudes tipicas
//   - quanto MENOR o score, melhor (menos vibracao captada)
//   - subtrair baseline pra remover ruido intrinseco do sensor
//
// SEGURANCA:
//   Throttle de teste limitado a TEST_THROTTLE_PCT. Motores parados
//   entre freqs. Aborta no fim do sweep. Reset do ESP32 pra repetir.
// =====================================================================

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ---- Pinos: unica fonte de verdade em include/board_config.h ----
#include "board_config.h"

#define MOTOR_1_PIN PIN_MOTOR_1
#define MOTOR_2_PIN PIN_MOTOR_2
#define MOTOR_3_PIN PIN_MOTOR_3
#define MOTOR_4_PIN PIN_MOTOR_4

#define I2C_SDA PIN_I2C_SDA
#define I2C_SCL PIN_I2C_SCL

// ---- Parametros do teste ----
static const uint32_t FREQS_HZ[] = {
    1000, 2000, 4000, 8000, 12000,
    16000, 20000, 24000, 32000, 48000
};
static const size_t N_FREQS = sizeof(FREQS_HZ) / sizeof(FREQS_HZ[0]);

static const uint8_t  PWM_RES_BITS         = 10;
static const uint32_t PWM_MAX_DUTY         = (1u << PWM_RES_BITS) - 1;
static const float    TEST_THROTTLE_PCT    = 20.0f;   // alto o bastante p/ excitar vibr.
static const uint32_t SAMPLES_PER_FREQ     = 2000;    // ~2 s @ ~1 kHz de leitura I2C
static const uint32_t SAMPLES_BASELINE     = 3000;    // baseline mais longo (ref. confiavel)
static const uint32_t SETTLE_MS            = 400;     // espera motor estabilizar
static const uint32_t COOLDOWN_MS          = 600;     // descanso entre freqs
static const float    GYRO_WEIGHT          = 1.0f;
static const float    ACC_WEIGHT           = 5.0f;

static const int CHANNELS[4] = {0, 1, 2, 3};
static const int PINS[4]     = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};

Adafruit_MPU6050 mpu;

// Welford: media e variancia incrementais, numericamente estaveis
struct Welford {
    uint32_t n = 0;
    double mean = 0.0;
    double M2   = 0.0;
    void add(double x) {
        n++;
        double d = x - mean;
        mean += d / n;
        M2   += d * (x - mean);
    }
    double stddev() const { return (n > 1) ? sqrt(M2 / (n - 1)) : 0.0; }
};

struct Stats6 {
    Welford gx, gy, gz; // rad/s
    Welford ax, ay, az; // m/s^2
};

static void writeAll(float throttle_pct) {
    if (throttle_pct < 0)   throttle_pct = 0;
    if (throttle_pct > 100) throttle_pct = 100;
    uint32_t duty = (uint32_t)((throttle_pct / 100.0f) * (float)PWM_MAX_DUTY + 0.5f);
    for (int i = 0; i < 4; i++) ledcWrite(CHANNELS[i], duty);
}

static void detachAll() {
    for (int i = 0; i < 4; i++) ledcDetachPin(PINS[i]);
}

static void setupPWM(uint32_t freq_hz) {
    for (int i = 0; i < 4; i++) {
        ledcSetup(CHANNELS[i], freq_hz, PWM_RES_BITS);
        ledcAttachPin(PINS[i], CHANNELS[i]);
    }
}

static void collect(Stats6 &s, uint32_t n_samples) {
    sensors_event_t a, g, t;
    for (uint32_t i = 0; i < n_samples; i++) {
        mpu.getEvent(&a, &g, &t);
        s.gx.add(g.gyro.x);
        s.gy.add(g.gyro.y);
        s.gz.add(g.gyro.z);
        s.ax.add(a.acceleration.x);
        s.ay.add(a.acceleration.y);
        s.az.add(a.acceleration.z);
    }
}

static double norm3(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

void setup() {
    Serial.begin(115200);
    delay(2500);

    Serial.println();
    Serial.println("==========================================================");
    Serial.println("       PWM FREQUENCY SWEEP - QUAD MOTOR VIBRATION");
    Serial.println("==========================================================");
    Serial.printf ("Freqs: ");
    for (size_t i = 0; i < N_FREQS; i++) Serial.printf("%lu ", (unsigned long)FREQS_HZ[i]);
    Serial.println("Hz");
    Serial.printf ("Throttle de teste: %.1f%%\n", TEST_THROTTLE_PCT);
    Serial.printf ("Amostras por freq: %lu (~%.1f s @ 1 kHz)\n",
                   (unsigned long)SAMPLES_PER_FREQ, SAMPLES_PER_FREQ / 1000.0f);
    Serial.println();
    Serial.println("!! PRENDA O DRONE. HELICES VAO GIRAR A 35% !!");
    Serial.println("Mande qualquer caractere no serial para iniciar...");
    while (!Serial.available()) { delay(100); }
    while (Serial.available()) Serial.read();

    // ---- I2C / MPU ----
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("ERRO: MPU6050 nao encontrado");
        while (1) delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // Largura de banda MAXIMA do filtro built-in pra nao mascarar ruido do PWM
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    // ---- Baseline: motores parados ----
    Serial.println(">> Coletando baseline (motores parados)...");
    setupPWM(25000);
    writeAll(0);
    delay(1500); // segura o drone enquanto motores ficam quietos
    Stats6 base;
    collect(base, SAMPLES_BASELINE);

    const float R2D = 180.0f / (float)M_PI;
    double base_g_norm_dps = norm3(base.gx.stddev() * R2D,
                                   base.gy.stddev() * R2D,
                                   base.gz.stddev() * R2D);
    double base_a_norm_g   = norm3(base.ax.stddev(),
                                   base.ay.stddev(),
                                   base.az.stddev()) / 9.80665;
    Serial.printf(">> Baseline: gyro_norm=%.4f dps | acc_norm=%.5f g\n",
                  base_g_norm_dps, base_a_norm_g);
    Serial.println();

    // ---- CSV header ----
    Serial.println("freq_hz,gx_dps,gy_dps,gz_dps,gyro_norm_dps,"
                   "ax_g,ay_g,az_g,acc_norm_g,score_raw,score_minus_baseline");

    double best_score = 1e30;
    uint32_t best_freq = 0;
    double scores[N_FREQS];

    for (size_t f = 0; f < N_FREQS; f++) {
        detachAll();
        setupPWM(FREQS_HZ[f]);
        writeAll(TEST_THROTTLE_PCT);
        delay(SETTLE_MS);

        Stats6 s;
        collect(s, SAMPLES_PER_FREQ);
        writeAll(0);

        double gx = s.gx.stddev() * R2D;
        double gy = s.gy.stddev() * R2D;
        double gz = s.gz.stddev() * R2D;
        double g_norm = norm3(gx, gy, gz);

        double ax = s.ax.stddev() / 9.80665;
        double ay = s.ay.stddev() / 9.80665;
        double az = s.az.stddev() / 9.80665;
        double a_norm = norm3(ax, ay, az);

        double score_raw = GYRO_WEIGHT * g_norm + ACC_WEIGHT * a_norm;
        double score_adj = GYRO_WEIGHT * fmax(0.0, g_norm - base_g_norm_dps)
                         + ACC_WEIGHT  * fmax(0.0, a_norm - base_a_norm_g);
        scores[f] = score_adj;

        Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,"
                      "%.5f,%.5f,%.5f,%.5f,%.3f,%.3f\n",
                      (unsigned long)FREQS_HZ[f],
                      gx, gy, gz, g_norm,
                      ax, ay, az, a_norm,
                      score_raw, score_adj);

        if (score_adj < best_score) {
            best_score = score_adj;
            best_freq  = FREQS_HZ[f];
        }
        delay(COOLDOWN_MS);
    }

    // Safety: desliga e desanexa tudo
    writeAll(0);
    detachAll();

    Serial.println();
    Serial.println("==========================================================");
    Serial.printf ("MELHOR FREQ: %lu Hz  (score_minus_baseline=%.3f)\n",
                   (unsigned long)best_freq, best_score);
    Serial.println("Ranking (menor score = melhor):");
    // simple selection sort para imprimir ranking
    bool used[N_FREQS] = {false};
    for (size_t rank = 0; rank < N_FREQS; rank++) {
        size_t pick = 0;
        double min_s = 1e30;
        for (size_t i = 0; i < N_FREQS; i++) {
            if (!used[i] && scores[i] < min_s) {
                min_s = scores[i];
                pick  = i;
            }
        }
        used[pick] = true;
        Serial.printf("  %2u. %lu Hz  score=%.3f\n",
                      (unsigned)rank + 1,
                      (unsigned long)FREQS_HZ[pick],
                      scores[pick]);
    }
    Serial.println("==========================================================");
    Serial.println("Teste concluido. RESET do ESP32 para repetir.");
}

void loop() {
    delay(1000);
}
