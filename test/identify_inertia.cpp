/**
 * IDENTIFICACAO EMPIRICA DE INERCIA (Ixx, Iyy, Izz, Ir)
 *
 * NOVO FLUXO (sem cabo durante ensaio):
 * - Em modo bateria (sem USB), o teste roda automaticamente e grava dados no SPIFFS.
 * - Depois, conecte USB e use o menu serial para exportar CSV ao computador.
 *
 * Comandos no menu USB:
 * - r: Executar identificacao agora (com cabo, apenas se desejar)
 * - e: Exportar log salvo em CSV
 * - s: Mostrar ultimo resumo salvo
 * - c: Limpar arquivos salvos
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <FS.h>
#include <math.h>
#include <ctype.h>

#include "MotorControl.h"
#include "board_config.h"
#include "MatrixOperations.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ======== Parametros fisicos conhecidos (ajuste se necessario) ========
static constexpr float MOTOR_B_COEFF = 1.11e-8f;                 // N/(rad/s)^2
static constexpr float MOTOR_D_COEFF = 0.05f * MOTOR_B_COEFF;    // N.m/(rad/s)^2
static constexpr float L_ARM = 0.060f;                           // m
static constexpr float MAX_RPM = 31086.0f;
static constexpr float MAX_OMEGA = (MAX_RPM * 2.0f * PI) / 60.0f;
static constexpr float MAX_OMEGA_SQ = MAX_OMEGA * MAX_OMEGA;

// Tensor de inercia medido do JIG (kg.m^2).
// Inclui termos cruzados completos: Ixy/Iyx, Ixz/Izx e Iyz/Izy.
static constexpr float I_JIG_XX =  6.391e-4f;
static constexpr float I_JIG_XY =  3.3998e-8f;
static constexpr float I_JIG_XZ = -5.5664e-7f;
static constexpr float I_JIG_YX =  3.3998e-8f;
static constexpr float I_JIG_YY =  8.365e-4f;
static constexpr float I_JIG_YZ =  3.03228e-7f;
static constexpr float I_JIG_ZX = -5.5664e-7f;
static constexpr float I_JIG_ZY =  3.03228e-7f;
static constexpr float I_JIG_ZZ =  9.776e-4f;

static constexpr float I_JIG[3][3] = {
    {I_JIG_XX, I_JIG_XY, I_JIG_XZ},
    {I_JIG_YX, I_JIG_YY, I_JIG_YZ},
    {I_JIG_ZX, I_JIG_ZY, I_JIG_ZZ}
};

// ======== Configuracao do experimento ========
static constexpr float MASS_EST_KG = 0.040f;
static constexpr float G = 9.80665f;
static constexpr float BASE_THRUST_N = 2.0f * MOTOR_B_COEFF * MAX_OMEGA_SQ; // Meio exato da faixa [Tmin, Tmax]

static constexpr float TAU_ROLL_AMP = 3.5e-4f;   // N.m
static constexpr float TAU_PITCH_AMP = 3.5e-4f;  // N.m
static constexpr float TAU_YAW_AMP = 8.0e-5f;    // N.m

static constexpr uint32_t CONTROL_DT_US = 4000;      // 250 Hz
static constexpr uint32_t WARMUP_TIME_MS = 3000;
static constexpr uint32_t IDENT_TIME_MS = 25000;

static constexpr float RATE_LPF_ALPHA = 0.22f;
static constexpr float MAX_VALID_RATE = 12.0f;       // rad/s
static constexpr float MAX_VALID_ACCEL = 300.0f;     // rad/s^2

// ======== Persistencia local (SPIFFS) ========
static constexpr const char* LOG_FILE_PATH = "/inertia_log.bin";
static constexpr const char* RESULT_FILE_PATH = "/inertia_result.txt";
static constexpr uint32_t LOG_MAGIC = 0x314E5249UL;   // "IRN1"
static constexpr uint16_t LOG_VERSION = 1;
static constexpr uint32_t LOG_DECIMATION = 2;         // 125 Hz de log com loop a 250 Hz

// ======== Sensores ========
static constexpr int I2C_SDA = PIN_I2C_SDA;
static constexpr int I2C_SCL = PIN_I2C_SCL;
Adafruit_MPU6050 imu;



MotorControl motors;

struct GyroRates {
    float p;
    float q;
    float r;
};

struct CommandResult {
    float omega_r;
    bool saturated;
};

struct IdentificationSummary {
    bool ok;
    float Ixx;
    float Iyy;
    float Izz;
    float Ir;
    float Ir_from_p;
    float Ir_from_q;
    float r2_p;
    float r2_q;
    float r2_r;
    uint32_t samples_total;
    uint32_t samples_model_p;
    uint32_t samples_model_q;
    uint32_t samples_model_r;
    uint32_t saturations;
};

#pragma pack(push, 1)
struct LogHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t sample_size;
    uint32_t control_dt_us;
    uint32_t log_decimation;
    uint32_t ident_time_ms;
};

struct LogSample {
    uint32_t t_ms;
    float tau_roll;
    float tau_pitch;
    float tau_yaw;
    float p;
    float q;
    float r;
    float pdot;
    float qdot;
    float rdot;
    float omega_r;
    uint8_t saturated;
};
#pragma pack(pop)

float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

bool storage_ready = false;

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void computeJigInertialTorqueComp(float p, float q, float r,
                                  float pdot, float qdot, float rdot,
                                  float& tau_jig_roll,
                                  float& tau_jig_pitch,
                                  float& tau_jig_yaw) {
    const float omega[3] = {p, q, r};
    const float omega_dot[3] = {pdot, qdot, rdot};

    float j_alpha[3] = {0.0f, 0.0f, 0.0f};
    float j_w[3] = {0.0f, 0.0f, 0.0f};
    float wx_jw[3] = {0.0f, 0.0f, 0.0f};
    float tau_jig[3] = {0.0f, 0.0f, 0.0f};

    MatrixOperations::matrixVectorMultiply(&I_JIG[0][0], omega_dot, j_alpha, 3, 3);
    MatrixOperations::matrixVectorMultiply(&I_JIG[0][0], omega, j_w, 3, 3);
    MatrixOperations::crossProduct(omega, j_w, wx_jw, 3);
    MatrixOperations::matrixAdd(j_alpha, wx_jw, tau_jig, 3, 1);

    tau_jig_roll = tau_jig[0];
    tau_jig_pitch = tau_jig[1];
    tau_jig_yaw = tau_jig[2];
}

bool initStorage() {
    if (!SPIFFS.begin(true)) {
        Serial.println("AVISO: SPIFFS nao inicializou. O teste roda sem salvar log.");
        return false;
    }

    Serial.println("SPIFFS inicializado.");
    return true;
}

void clearStoredData() {
    if (!storage_ready) {
        Serial.println("SPIFFS indisponivel.");
        return;
    }

    bool removed_log = !SPIFFS.exists(LOG_FILE_PATH) || SPIFFS.remove(LOG_FILE_PATH);
    bool removed_result = !SPIFFS.exists(RESULT_FILE_PATH) || SPIFFS.remove(RESULT_FILE_PATH);

    Serial.print("Remocao log: ");
    Serial.println(removed_log ? "OK" : "FALHOU");
    Serial.print("Remocao resumo: ");
    Serial.println(removed_result ? "OK" : "FALHOU");
}

bool beginBinaryLog(File& logFile) {
    if (!storage_ready) return false;

    if (SPIFFS.exists(LOG_FILE_PATH)) {
        SPIFFS.remove(LOG_FILE_PATH);
    }

    logFile = SPIFFS.open(LOG_FILE_PATH, FILE_WRITE);
    if (!logFile) {
        Serial.println("AVISO: nao foi possivel criar log binario.");
        return false;
    }

    LogHeader header;
    header.magic = LOG_MAGIC;
    header.version = LOG_VERSION;
    header.sample_size = sizeof(LogSample);
    header.control_dt_us = CONTROL_DT_US;
    header.log_decimation = LOG_DECIMATION;
    header.ident_time_ms = IDENT_TIME_MS;

    size_t wr = logFile.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    if (wr != sizeof(header)) {
        Serial.println("AVISO: falha ao escrever cabecalho do log.");
        logFile.close();
        return false;
    }

    return true;
}

void appendBinarySample(File& logFile, const LogSample& sample) {
    if (!logFile) return;
    logFile.write(reinterpret_cast<const uint8_t*>(&sample), sizeof(sample));
}

void saveSummaryToFile(const IdentificationSummary& s) {
    if (!storage_ready) return;

    if (SPIFFS.exists(RESULT_FILE_PATH)) {
        SPIFFS.remove(RESULT_FILE_PATH);
    }

    File f = SPIFFS.open(RESULT_FILE_PATH, FILE_WRITE);
    if (!f) {
        Serial.println("AVISO: nao foi possivel salvar resumo.");
        return;
    }

    f.printf("ok=%d\n", s.ok ? 1 : 0);
    f.printf("Ixx=%.9e\n", s.Ixx);
    f.printf("Iyy=%.9e\n", s.Iyy);
    f.printf("Izz=%.9e\n", s.Izz);
    f.printf("Ir=%.9e\n", s.Ir);
    f.printf("Ir_from_p=%.9e\n", s.Ir_from_p);
    f.printf("Ir_from_q=%.9e\n", s.Ir_from_q);
    f.printf("R2_p=%.6f\n", s.r2_p);
    f.printf("R2_q=%.6f\n", s.r2_q);
    f.printf("R2_r=%.6f\n", s.r2_r);
    f.printf("samples_total=%lu\n", static_cast<unsigned long>(s.samples_total));
    f.printf("samples_model_p=%lu\n", static_cast<unsigned long>(s.samples_model_p));
    f.printf("samples_model_q=%lu\n", static_cast<unsigned long>(s.samples_model_q));
    f.printf("samples_model_r=%lu\n", static_cast<unsigned long>(s.samples_model_r));
    f.printf("saturations=%lu\n", static_cast<unsigned long>(s.saturations));

    f.close();
}

void printStoredSummary() {
    if (!storage_ready) {
        Serial.println("SPIFFS indisponivel.");
        return;
    }

    if (!SPIFFS.exists(RESULT_FILE_PATH)) {
        Serial.println("Nenhum resumo salvo encontrado.");
        return;
    }

    File f = SPIFFS.open(RESULT_FILE_PATH, FILE_READ);
    if (!f) {
        Serial.println("Falha ao abrir resumo salvo.");
        return;
    }

    Serial.println("\n===== ULTIMO RESUMO SALVO =====");
    while (f.available()) {
        Serial.write(f.read());
    }
    Serial.println("===== FIM RESUMO =====\n");

    f.close();
}

void exportLogAsCSV() {
    if (!storage_ready) {
        Serial.println("SPIFFS indisponivel.");
        return;
    }

    if (!SPIFFS.exists(LOG_FILE_PATH)) {
        Serial.println("Nenhum log binario salvo encontrado.");
        return;
    }

    File f = SPIFFS.open(LOG_FILE_PATH, FILE_READ);
    if (!f) {
        Serial.println("Falha ao abrir log binario.");
        return;
    }

    LogHeader header;
    if (f.read(reinterpret_cast<uint8_t*>(&header), sizeof(header)) != sizeof(header)) {
        Serial.println("Falha ao ler cabecalho do log.");
        f.close();
        return;
    }

    if (header.magic != LOG_MAGIC || header.version != LOG_VERSION || header.sample_size != sizeof(LogSample)) {
        Serial.println("Formato de log invalido ou versao incompativel.");
        f.close();
        return;
    }

    Serial.println("CSV_BEGIN");
    Serial.println("t_ms,tau_roll,tau_pitch,tau_yaw,p,q,r,pdot,qdot,rdot,omega_r,saturated");

    LogSample s;
    uint32_t count = 0;

    while (f.available() >= static_cast<int>(sizeof(LogSample))) {
        if (f.read(reinterpret_cast<uint8_t*>(&s), sizeof(s)) != sizeof(s)) {
            break;
        }

        Serial.printf("%lu,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%u\n",
                      static_cast<unsigned long>(s.t_ms),
                      s.tau_roll, s.tau_pitch, s.tau_yaw,
                      s.p, s.q, s.r,
                      s.pdot, s.qdot, s.rdot,
                      s.omega_r,
                      static_cast<unsigned>(s.saturated));

        count++;
        if ((count % 200) == 0) {
            delay(2);
        }
    }

    Serial.println("CSV_END");
    Serial.print("Linhas exportadas: ");
    Serial.println(count);

    f.close();
}

void printUSBMenu() {
    Serial.println("\n===== MENU USB (MODO SEGURO) =====");
    Serial.println("r - rodar identificacao agora");
    Serial.println("e - exportar log salvo para CSV");
    Serial.println("s - mostrar ultimo resumo salvo");
    Serial.println("c - limpar log e resumo salvos");
    Serial.println("h - mostrar menu novamente");
    Serial.println("==================================");
}

bool initIMU() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(1000000);

    if (!imu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("ERRO: MPU6050 nao encontrado.");
        return false;
    }

    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    Serial.println("IMU: MPU6050 inicializado.");
    return true;
}

void readGyroRaw(float& gx, float& gy, float& gz) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    gx = g.gyro.x;
    gy = g.gyro.y;
    gz = g.gyro.z;
}

void readGyroBodyRates(GyroRates& rates) {
    float gx, gy, gz;
    readGyroRaw(gx, gy, gz);

    rates.p = gx - gyro_bias_x;
    rates.q = gy - gyro_bias_y;
    rates.r = gz - gyro_bias_z;
}

void calibrateGyroBias(uint16_t samples = 1500, uint8_t sampleDelayMs = 2) {
    Serial.println("Calibrando bias do giroscopio. Mantenha o drone totalmente parado...");

    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;

    for (uint16_t i = 0; i < samples; i++) {
        float gx, gy, gz;
        readGyroRaw(gx, gy, gz);
        sx += gx;
        sy += gy;
        sz += gz;
        delay(sampleDelayMs);
    }

    gyro_bias_x = static_cast<float>(sx / samples);
    gyro_bias_y = static_cast<float>(sy / samples);
    gyro_bias_z = static_cast<float>(sz / samples);

    Serial.print("Bias gyro X: "); Serial.println(gyro_bias_x, 6);
    Serial.print("Bias gyro Y: "); Serial.println(gyro_bias_y, 6);
    Serial.print("Bias gyro Z: "); Serial.println(gyro_bias_z, 6);
}

CommandResult commandFromTorques(float thrustN, float tauRoll, float tauPitch, float tauYaw) {
    // Alocacao de controle (quad X), mesma convencao usada no projeto
    const float inv_4b = 1.0f / (4.0f * MOTOR_B_COEFF);
    const float inv_2bL = 1.0f / (2.0f * MOTOR_B_COEFF * L_ARM);
    const float inv_4d = 1.0f / (4.0f * MOTOR_D_COEFF);

    const float alloc[16] = {
        inv_4b, -inv_2bL,  inv_2bL, -inv_4d,
        inv_4b, -inv_2bL, -inv_2bL,  inv_4d,
        inv_4b,  inv_2bL, -inv_2bL, -inv_4d,
        inv_4b,  inv_2bL,  inv_2bL,  inv_4d
    };

    const float u[4] = {thrustN, tauRoll, tauPitch, tauYaw};
    float w_sq[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    MatrixOperations::matrixVectorMultiply(alloc, u, w_sq, 4, 4);

    float w1_sq = w_sq[0];
    float w2_sq = w_sq[1];
    float w3_sq = w_sq[2];
    float w4_sq = w_sq[3];

    bool saturated = false;

    if (w1_sq < 0.0f || w1_sq > MAX_OMEGA_SQ) saturated = true;
    if (w2_sq < 0.0f || w2_sq > MAX_OMEGA_SQ) saturated = true;
    if (w3_sq < 0.0f || w3_sq > MAX_OMEGA_SQ) saturated = true;
    if (w4_sq < 0.0f || w4_sq > MAX_OMEGA_SQ) saturated = true;

    w1_sq = clampf(w1_sq, 0.0f, MAX_OMEGA_SQ);
    w2_sq = clampf(w2_sq, 0.0f, MAX_OMEGA_SQ);
    w3_sq = clampf(w3_sq, 0.0f, MAX_OMEGA_SQ);
    w4_sq = clampf(w4_sq, 0.0f, MAX_OMEGA_SQ);

    motors.setMotorsFromOmegaSq(w1_sq, w2_sq, w3_sq, w4_sq);

    const float w1 = sqrtf(w1_sq);
    const float w2 = sqrtf(w2_sq);
    const float w3 = sqrtf(w3_sq);
    const float w4 = sqrtf(w4_sq);

    // Modelo de Voos 2006 usado no restante do projeto
    CommandResult out;
    out.omega_r = -w1 + w2 - w3 + w4;
    out.saturated = saturated;
    return out;
}

template <size_t N>
struct RegressionModel {
    double ata[N][N];
    double aty[N];
    double y_sum;
    double y2_sum;
    uint32_t n;

    RegressionModel() : y_sum(0.0), y2_sum(0.0), n(0) {
        for (size_t i = 0; i < N; i++) {
            aty[i] = 0.0;
            for (size_t j = 0; j < N; j++) {
                ata[i][j] = 0.0;
            }
        }
    }

    void add(const float (&phi)[N], float y) {
        for (size_t i = 0; i < N; i++) {
            aty[i] += static_cast<double>(phi[i]) * y;
            for (size_t j = 0; j < N; j++) {
                ata[i][j] += static_cast<double>(phi[i]) * static_cast<double>(phi[j]);
            }
        }

        y_sum += y;
        y2_sum += static_cast<double>(y) * y;
        n++;
    }

    bool solve(float (&theta)[N]) const {
        double aug[N][N + 1];

        for (size_t i = 0; i < N; i++) {
            for (size_t j = 0; j < N; j++) {
                aug[i][j] = ata[i][j];
            }
            aug[i][i] += 1e-9;
            aug[i][N] = aty[i];
        }

        for (size_t col = 0; col < N; col++) {
            size_t pivot = col;
            double best = fabs(aug[col][col]);
            for (size_t row = col + 1; row < N; row++) {
                double v = fabs(aug[row][col]);
                if (v > best) {
                    best = v;
                    pivot = row;
                }
            }

            if (best < 1e-12) {
                return false;
            }

            if (pivot != col) {
                for (size_t k = col; k <= N; k++) {
                    double tmp = aug[col][k];
                    aug[col][k] = aug[pivot][k];
                    aug[pivot][k] = tmp;
                }
            }

            double diag = aug[col][col];
            for (size_t k = col; k <= N; k++) {
                aug[col][k] /= diag;
            }

            for (size_t row = 0; row < N; row++) {
                if (row == col) continue;
                double f = aug[row][col];
                for (size_t k = col; k <= N; k++) {
                    aug[row][k] -= f * aug[col][k];
                }
            }
        }

        for (size_t i = 0; i < N; i++) {
            theta[i] = static_cast<float>(aug[i][N]);
        }

        return true;
    }

    float computeR2(const float (&theta)[N]) const {
        if (n < 2) return 0.0f;

        double theta_aty = 0.0;
        for (size_t i = 0; i < N; i++) {
            theta_aty += theta[i] * aty[i];
        }

        double sse = y2_sum - theta_aty;
        if (sse < 0.0) sse = 0.0;

        double y_mean = y_sum / n;
        double sst = y2_sum - static_cast<double>(n) * y_mean * y_mean;
        if (sst < 1e-12) return 0.0f;

        float r2 = static_cast<float>(1.0 - (sse / sst));
        return clampf(r2, 0.0f, 1.0f);
    }
};

IdentificationSummary runIdentification() {
    IdentificationSummary summary = {};

    RegressionModel<3> model_p; // pdot = [tau_roll, q*r, -omega_r*q] * [a1,a2,a3]
    RegressionModel<3> model_q; // qdot = [tau_pitch, p*r, omega_r*p] * [b1,b2,b3]
    RegressionModel<2> model_r; // rdot = [tau_yaw, p*q] * [c1,c2]

    float p_f = 0.0f, q_f = 0.0f, r_f = 0.0f;
    float p_prev = 0.0f, q_prev = 0.0f, r_prev = 0.0f;
    bool first = true;

    uint32_t sat_count = 0;
    uint32_t total_steps = 0;

    File logFile;
    bool logEnabled = beginBinaryLog(logFile);
    if (logEnabled) {
        Serial.println("Log binario ativado em /inertia_log.bin");
    } else {
        Serial.println("Log binario desativado.");
    }

    Serial.println("\nIniciando warm-up de motores e filtro...");
    uint32_t warmup_start = millis();
    while (millis() - warmup_start < WARMUP_TIME_MS) {
        uint32_t tick_us = micros();

        CommandResult cmd = commandFromTorques(BASE_THRUST_N, 0.0f, 0.0f, 0.0f);
        if (cmd.saturated) sat_count++;

        GyroRates w;
        readGyroBodyRates(w);

        p_f += RATE_LPF_ALPHA * (w.p - p_f);
        q_f += RATE_LPF_ALPHA * (w.q - q_f);
        r_f += RATE_LPF_ALPHA * (w.r - r_f);

        p_prev = p_f;
        q_prev = q_f;
        r_prev = r_f;
        first = false;

        total_steps++;

        uint32_t elapsed_us = micros() - tick_us;
        if (elapsed_us < CONTROL_DT_US) {
            delayMicroseconds(CONTROL_DT_US - elapsed_us);
        }
    }

    Serial.println("Warm-up concluido.");
    Serial.println("Compensacao de inercia do JIG ativa (tensor completo com termos cruzados).");
    Serial.println("Executando excitacao para identificacao...");

    uint32_t ident_start = millis();
    uint32_t last_print = ident_start;
    uint32_t log_counter = 0;

    while (millis() - ident_start < IDENT_TIME_MS) {
        uint32_t tick_us = micros();

        uint32_t t_ms = millis() - ident_start;
        float t = t_ms * 0.001f;

        // Excitacao rica em frequencia para melhor identificacao
        float tau_roll = TAU_ROLL_AMP * (0.70f * sinf(2.0f * PI * 0.7f * t) + 0.30f * sinf(2.0f * PI * 1.9f * t));
        float tau_pitch = TAU_PITCH_AMP * (0.65f * sinf(2.0f * PI * 0.9f * t) + 0.35f * sinf(2.0f * PI * 2.2f * t));
        float tau_yaw = TAU_YAW_AMP * (0.60f * sinf(2.0f * PI * 1.1f * t) + 0.40f * sinf(2.0f * PI * 2.8f * t));

        CommandResult cmd = commandFromTorques(BASE_THRUST_N, tau_roll, tau_pitch, tau_yaw);
        if (cmd.saturated) sat_count++;

        GyroRates w;
        readGyroBodyRates(w);

        p_f += RATE_LPF_ALPHA * (w.p - p_f);
        q_f += RATE_LPF_ALPHA * (w.q - q_f);
        r_f += RATE_LPF_ALPHA * (w.r - r_f);

        float dt = CONTROL_DT_US * 1e-6f;
        float pdot = (p_f - p_prev) / dt;
        float qdot = (q_f - q_prev) / dt;
        float rdot = (r_f - r_prev) / dt;

        p_prev = p_f;
        q_prev = q_f;
        r_prev = r_f;

        if (!first) {
            bool valid_rates = (fabsf(p_f) < MAX_VALID_RATE) &&
                               (fabsf(q_f) < MAX_VALID_RATE) &&
                               (fabsf(r_f) < MAX_VALID_RATE);

            bool valid_accel = (fabsf(pdot) < MAX_VALID_ACCEL) &&
                               (fabsf(qdot) < MAX_VALID_ACCEL) &&
                               (fabsf(rdot) < MAX_VALID_ACCEL);

            if (valid_rates && valid_accel) {
                float tau_jig_roll = 0.0f;
                float tau_jig_pitch = 0.0f;
                float tau_jig_yaw = 0.0f;
                computeJigInertialTorqueComp(p_f, q_f, r_f,
                                             pdot, qdot, rdot,
                                             tau_jig_roll, tau_jig_pitch, tau_jig_yaw);

                const float tau_roll_eff = tau_roll - tau_jig_roll;
                const float tau_pitch_eff = tau_pitch - tau_jig_pitch;
                const float tau_yaw_eff = tau_yaw - tau_jig_yaw;

                const float phi_p[3] = {tau_roll_eff, q_f * r_f, -cmd.omega_r * q_f};
                const float phi_q[3] = {tau_pitch_eff, p_f * r_f,  cmd.omega_r * p_f};
                const float phi_r[2] = {tau_yaw_eff, p_f * q_f};

                model_p.add(phi_p, pdot);
                model_q.add(phi_q, qdot);
                model_r.add(phi_r, rdot);
            }
        }

        if (logEnabled && ((log_counter % LOG_DECIMATION) == 0)) {
            LogSample sample;
            sample.t_ms = t_ms;
            sample.tau_roll = tau_roll;
            sample.tau_pitch = tau_pitch;
            sample.tau_yaw = tau_yaw;
            sample.p = p_f;
            sample.q = q_f;
            sample.r = r_f;
            sample.pdot = pdot;
            sample.qdot = qdot;
            sample.rdot = rdot;
            sample.omega_r = cmd.omega_r;
            sample.saturated = cmd.saturated ? 1 : 0;

            appendBinarySample(logFile, sample);
        }
        log_counter++;

        first = false;
        total_steps++;

        if (millis() - last_print >= 1000) {
            uint32_t elapsed_ms = millis() - ident_start;
            uint32_t remaining_s = (IDENT_TIME_MS > elapsed_ms) ? (IDENT_TIME_MS - elapsed_ms) / 1000 : 0;

            Serial.print("Tempo restante: ");
            Serial.print(remaining_s);
            Serial.print(" s | p=");
            Serial.print(p_f, 3);
            Serial.print(" q=");
            Serial.print(q_f, 3);
            Serial.print(" r=");
            Serial.print(r_f, 3);
            Serial.print(" | sat=");
            Serial.println(sat_count);

            if (logEnabled) {
                logFile.flush();
            }

            last_print = millis();
        }

        uint32_t elapsed_us = micros() - tick_us;
        if (elapsed_us < CONTROL_DT_US) {
            delayMicroseconds(CONTROL_DT_US - elapsed_us);
        }
    }

    Serial.println("Encerrando excitacao...");
    commandFromTorques(BASE_THRUST_N * 0.5f, 0.0f, 0.0f, 0.0f);
    delay(200);
    commandFromTorques(0.0f, 0.0f, 0.0f, 0.0f);
    delay(100);
    motors.stopAllMotors();

    if (logEnabled) {
        logFile.flush();
        logFile.close();
    }

    float theta_p[3] = {0.0f, 0.0f, 0.0f};
    float theta_q[3] = {0.0f, 0.0f, 0.0f};
    float theta_r[2] = {0.0f, 0.0f};

    bool ok_p = model_p.solve(theta_p);
    bool ok_q = model_q.solve(theta_q);
    bool ok_r = model_r.solve(theta_r);

    Serial.println("\n================ RESULTADOS =================");
    Serial.print("Amostras totais de loop: ");
    Serial.println(total_steps);
    Serial.print("Saturacoes de comando: ");
    Serial.println(sat_count);

    summary.ok = false;
    summary.samples_total = total_steps;
    summary.samples_model_p = model_p.n;
    summary.samples_model_q = model_q.n;
    summary.samples_model_r = model_r.n;
    summary.saturations = sat_count;

    if (!ok_p || !ok_q || !ok_r) {
        Serial.println("ERRO: regressao singular. Aumente o tempo de teste ou amplitude da excitacao.");
        Serial.println("=============================================");
        saveSummaryToFile(summary);
        return summary;
    }

    // Parametros identificados:
    // pdot = a1*tau_roll + a2*(q*r) - a3*(omega_r*q)
    // qdot = b1*tau_pitch + b2*(p*r) + b3*(omega_r*p)
    // rdot = c1*tau_yaw + c2*(p*q)
    float a1 = theta_p[0];
    float a3 = theta_p[2];
    float b1 = theta_q[0];
    float b3 = theta_q[2];
    float c1 = theta_r[0];

    if (fabsf(a1) < 1e-9f || fabsf(b1) < 1e-9f || fabsf(c1) < 1e-9f) {
        Serial.println("ERRO: ganho principal muito pequeno. Identificacao invalida.");
        Serial.println("=============================================");
        saveSummaryToFile(summary);
        return summary;
    }

    float Ixx = 1.0f / fabsf(a1);
    float Iyy = 1.0f / fabsf(b1);
    float Izz = 1.0f / fabsf(c1);

    float Ir_from_p = fabsf(a3) * Ixx;
    float Ir_from_q = fabsf(b3) * Iyy;
    float Ir = 0.5f * (Ir_from_p + Ir_from_q);

    float r2_p = model_p.computeR2(theta_p);
    float r2_q = model_q.computeR2(theta_q);
    float r2_r = model_r.computeR2(theta_r);

    summary.ok = true;
    summary.Ixx = Ixx;
    summary.Iyy = Iyy;
    summary.Izz = Izz;
    summary.Ir = Ir;
    summary.Ir_from_p = Ir_from_p;
    summary.Ir_from_q = Ir_from_q;
    summary.r2_p = r2_p;
    summary.r2_q = r2_q;
    summary.r2_r = r2_r;

    Serial.println("\nQualidade do ajuste (R2):");
    Serial.print("  Roll  (pdot): "); Serial.println(r2_p, 4);
    Serial.print("  Pitch (qdot): "); Serial.println(r2_q, 4);
    Serial.print("  Yaw   (rdot): "); Serial.println(r2_r, 4);

    Serial.println("\nMomentos de inercia estimados (drone, com compensacao do JIG):");
    Serial.print("  Ixx = "); Serial.print(Ixx, 9); Serial.println(" kg.m^2");
    Serial.print("  Iyy = "); Serial.print(Iyy, 9); Serial.println(" kg.m^2");
    Serial.print("  Izz = "); Serial.print(Izz, 9); Serial.println(" kg.m^2");

    Serial.println("\nInercia de rotor equivalente:");
    Serial.print("  Ir (via eq. p) = "); Serial.print(Ir_from_p, 9); Serial.println(" kg.m^2");
    Serial.print("  Ir (via eq. q) = "); Serial.print(Ir_from_q, 9); Serial.println(" kg.m^2");
    Serial.print("  Ir (media)     = "); Serial.print(Ir, 9); Serial.println(" kg.m^2");

    Serial.println("\nCole estes valores no codigo principal:");
    Serial.printf("const float Ixx = %.9ef;\n", Ixx);
    Serial.printf("const float Iyy = %.9ef;\n", Iyy);
    Serial.printf("const float Izz = %.9ef;\n", Izz);
    Serial.printf("const float Ir  = %.9ef;\n", Ir);

    if (r2_p < 0.80f || r2_q < 0.80f || r2_r < 0.80f) {
        Serial.println("\nAVISO: algum R2 ficou baixo (< 0.80). Repita o teste com:");
        Serial.println("- Drone mais firme no jig");
        Serial.println("- Menos vibracao mecanica");
        Serial.println("- Maior tempo de identificacao");
        Serial.println("- Ajuste de amplitudes de torque");
    }

    if (logEnabled) {
        Serial.println("\nLog salvo em /inertia_log.bin");
        Serial.println("Conecte USB depois e use comando 'e' para exportar CSV.");
    }

    saveSummaryToFile(summary);

    Serial.println("=============================================");
    return summary;
}

bool executeIdentificationFlow(bool fromUSBCommand) {
    Serial.println("\n=============================================");
    Serial.println("IDENTIFICACAO DE INERCIA - SDRE_VECTORIZED");
    Serial.println("=============================================");

    if (!initIMU()) {
        Serial.println("Falha ao inicializar IMU. Abortando.");
        return false;
    }

    calibrateGyroBias();

    if (!motors.begin()) {
        Serial.println("Falha ao inicializar motores. Abortando.");
        return false;
    }

    motors.setThrottleLimits(0, 100);
    motors.setOmegaSqLimits(0, MAX_OMEGA_SQ);

    if (fromUSBCommand) {
        Serial.println("\nATENCAO: comando manual via USB. Motores armarao em 5 segundos.");
    } else {
        Serial.println("\nModo autonomo (sem USB): motores armarao em 5 segundos.");
    }

    for (int i = 5; i > 0; i--) {
        Serial.print("Iniciando em ");
        Serial.println(i);
        delay(1000);
    }

    motors.armMotors();
    delay(500);

    IdentificationSummary summary = runIdentification();

    motors.stopAllMotors();
    Serial.println("Teste concluido. Motores desarmados.");

    return summary.ok;
}

void handleUSBMode() {
    Serial.println("\nUSB detectado: modo seguro ativado (nao arma motores automaticamente).");
    printUSBMenu();

    while (true) {
        if (Serial.available()) {
            char c = static_cast<char>(Serial.read());
            c = static_cast<char>(tolower(static_cast<unsigned char>(c)));

            if (c == '\n' || c == '\r') {
                continue;
            }

            switch (c) {
                case 'r':
                    executeIdentificationFlow(true);
                    printUSBMenu();
                    break;
                case 'e':
                    exportLogAsCSV();
                    break;
                case 's':
                    printStoredSummary();
                    break;
                case 'c':
                    clearStoredData();
                    break;
                case 'h':
                    printUSBMenu();
                    break;
                default:
                    Serial.print("Comando desconhecido: ");
                    Serial.println(c);
                    Serial.println("Digite 'h' para ajuda.");
                    break;
            }
        }

        delay(20);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1200);

    storage_ready = initStorage();

    bool usbDetected = false;
    uint32_t detectStart = millis();
    while (millis() - detectStart < 1500) {
        if (Serial) {
            usbDetected = true;
            break;
        }
        delay(10);
    }

    if (usbDetected) {
        handleUSBMode();
        return;
    }

    executeIdentificationFlow(false);
}

void loop() {
    delay(1000);
}
