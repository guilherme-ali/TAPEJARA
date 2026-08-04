#include "utils.h"
#include <AutoLQR.h>
#include <Wire.h>

#define STATE_SIZE 6
#define CONTROL_SIZE 3

// Declaração externa do controlador (definido em main.cpp)
extern AutoLQR sdreController;

// ============= FUNÇÕES DE INICIALIZAÇÃO =============

void start_IMU_MPU6050(Adafruit_MPU6050& mpu) {
    // Abre o barramento I2C da TPJ-01. E' o UNICO Wire.begin() do firmware — o
    // barometro e o magnetometro compartilham este barramento e so configuram
    // seus proprios registradores.
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    Wire.setClock(I2C_CLOCK_HZ); // 400 kHz - max do MPU6050 segundo datasheet (1 MHz violava spec)

    if (!mpu.begin(ADDR_MPU6050, &Wire)) {
        Serial.println("Falha ao inicializar MPU6050!");
        while (1) {
            delay(10);
        }
    }

    Serial.println("MPU6050 inicializado com sucesso!");

    // Configuração do MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    // DLPF 260 Hz (passa-banda mais larga, sem filtragem efetiva no MPU):
    // o anti-aliasing eh feito pelo Butterworth digital no main (cutoff 80 Hz,
    // loop a 200 Hz). Modos menores do DLPF interno adicionariam atraso de fase
    // significativo dentro do periodo de 5 ms.
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    mpu.setSampleRateDivisor(0);

    delay(100); // Aguarda estabilização
}

void calibrate_MPU6050(Adafruit_MPU6050& mpu,
                       float& accel_offset_x, float& accel_offset_y, float& accel_offset_z,
                       float& gyro_offset_x, float& gyro_offset_y, float& gyro_offset_z) {
    Serial.println("=========================================");
    Serial.println("Iniciando calibração do MPU6050...");
    Serial.println("IMPORTANTE: Deixe o sensor COMPLETAMENTE IMÓVEL!");
    Serial.println("=========================================");

    const int numSamples = 1000; // Número de amostras para calibração
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;

    // Aguarda 3 segundos antes de iniciar
    for (int i = 3; i > 0; i--) {
        Serial.print("Iniciando em ");
        Serial.print(i);
        Serial.println(" segundos...");
        delay(1000);
    }

    Serial.println("Coletando dados...");

    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        accel_sum_x += a.acceleration.x;
        accel_sum_y += a.acceleration.y;
        accel_sum_z += a.acceleration.z;

        gyro_sum_x += g.gyro.x;
        gyro_sum_y += g.gyro.y;
        gyro_sum_z += g.gyro.z;

        if (i % 100 == 0) {
            Serial.print("Progresso: ");
            Serial.print((i * 100) / numSamples);
            Serial.println("%");
        }

        delay(2); // Pequeno delay entre leituras
    }

    // Calcula as médias
    accel_offset_x = accel_sum_x / numSamples;
    accel_offset_y = accel_sum_y / numSamples;
    accel_offset_z = (accel_sum_z / numSamples) - 9.81f; // Subtrai gravidade (sensor na horizontal, Z aponta para cima)

    gyro_offset_x = gyro_sum_x / numSamples;
    gyro_offset_y = gyro_sum_y / numSamples;
    gyro_offset_z = gyro_sum_z / numSamples;

    Serial.println("=========================================");
    Serial.println("Calibração concluída!");
    Serial.println("Offsets calculados:");
    Serial.print("Accel X: "); Serial.print(accel_offset_x, 6); Serial.println(" m/s²");
    Serial.print("Accel Y: "); Serial.print(accel_offset_y, 6); Serial.println(" m/s²");
    Serial.print("Accel Z: "); Serial.print(accel_offset_z, 6); Serial.println(" m/s²");
    Serial.print("Gyro X: ");  Serial.print(gyro_offset_x, 6);  Serial.println(" rad/s");
    Serial.print("Gyro Y: ");  Serial.print(gyro_offset_y, 6);  Serial.println(" rad/s");
    Serial.print("Gyro Z: ");  Serial.print(gyro_offset_z, 6);  Serial.println(" rad/s");
    Serial.println("=========================================");
}

void read_MPU6050(Adafruit_MPU6050& mpu, float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz,
                  float accel_offset_x, float accel_offset_y, float accel_offset_z,
                  float gyro_offset_x, float gyro_offset_y, float gyro_offset_z) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Aplica os offsets de calibração e converte para as unidades esperadas pelo Madgwick.
    // Adafruit_MPU6050 retorna acel em m/s^2; o filtro espera g (sem normalizar magnitude).
    ax = (a.acceleration.x - accel_offset_x) / 9.81f;
    ay = (a.acceleration.y - accel_offset_y) / 9.81f;
    az = (a.acceleration.z - accel_offset_z) / 9.81f;

    gx = g.gyro.x - gyro_offset_x; // já em rad/s
    gy = g.gyro.y - gyro_offset_y;
    gz = g.gyro.z - gyro_offset_z;
}

// Endereço I2C e registro inicial do MPU6050 (mesmos usados por start_IMU_MPU6050)
#define MPU6050_I2C_ADDR   0x68
#define MPU6050_ACCEL_XOUT 0x3B  // primeiro de 14 regs: accel[6] temp[2] gyro[6]

void read_MPU6050_raw(float& ax, float& ay, float& az,
                      float& gx, float& gy, float& gz,
                      float accel_offset_x, float accel_offset_y, float accel_offset_z,
                      float gyro_offset_x, float gyro_offset_y, float gyro_offset_z) {
    // Burst read dos 14 bytes a partir de ACCEL_XOUT_H. Pula a Adafruit_BusIO/getEvent,
    // que custa ~1 ms por leitura (medido no ESP32-S2 da placa anterior).
    Wire.beginTransmission(MPU6050_I2C_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT);
    Wire.endTransmission(false);                       // repeated-start (sem soltar o barramento)
    Wire.requestFrom((uint8_t)MPU6050_I2C_ADDR, (uint8_t)14);

    if (Wire.available() < 14) {                        // falha de leitura → zera (não propaga lixo)
        ax = ay = az = gx = gy = gz = 0.0f;
        return;
    }

    // Big-endian (H depois L). Leitura em variáveis ordenadas: a ordem de avaliação
    // de Wire.read() em '(read()<<8)|read()' é NÃO-especificada em C++ — daria bug.
    uint8_t axh = Wire.read(), axl = Wire.read();
    uint8_t ayh = Wire.read(), ayl = Wire.read();
    uint8_t azh = Wire.read(), azl = Wire.read();
    uint8_t th  = Wire.read(), tl  = Wire.read();       // temperatura (descartada)
    uint8_t gxh = Wire.read(), gxl = Wire.read();
    uint8_t gyh = Wire.read(), gyl = Wire.read();
    uint8_t gzh = Wire.read(), gzl = Wire.read();
    (void)th; (void)tl;

    int16_t rawAX = (int16_t)((axh << 8) | axl);
    int16_t rawAY = (int16_t)((ayh << 8) | ayl);
    int16_t rawAZ = (int16_t)((azh << 8) | azl);
    int16_t rawGX = (int16_t)((gxh << 8) | gxl);
    int16_t rawGY = (int16_t)((gyh << 8) | gyl);
    int16_t rawGZ = (int16_t)((gzh << 8) | gzl);

    // Escalas IDÊNTICAS às da Adafruit para os ranges de start_IMU_MPU6050:
    //   accel RANGE_2_G     → 16384 LSB/g,        ×9.80665 → m/s²
    //   gyro  RANGE_1000_DEG → 32.8 LSB/(°/s),    ×(π/180) → rad/s
    // SE MUDAR OS RANGES em start_IMU_MPU6050, ATUALIZE ESTAS CONSTANTES.
    const float ACC_LSB_TO_MS2 = 9.80665f / 16384.0f;
    const float GYR_LSB_TO_RAD = 0.017453293f / 32.8f;

    float acc_x = rawAX * ACC_LSB_TO_MS2;
    float acc_y = rawAY * ACC_LSB_TO_MS2;
    float acc_z = rawAZ * ACC_LSB_TO_MS2;
    float gyr_x = rawGX * GYR_LSB_TO_RAD;
    float gyr_y = rawGY * GYR_LSB_TO_RAD;
    float gyr_z = rawGZ * GYR_LSB_TO_RAD;

    // Mesmas conversões finais de read_MPU6050 (offsets + accel normalizado em g)
    ax = (acc_x - accel_offset_x) / 9.81f;
    ay = (acc_y - accel_offset_y) / 9.81f;
    az = (acc_z - accel_offset_z) / 9.81f;
    gx = gyr_x - gyro_offset_x;
    gy = gyr_y - gyro_offset_y;
    gz = gyr_z - gyro_offset_z;
}

// ============= FUNÇÕES DO QMC5883L (Magnetômetro) =============
// Compartilha o barramento I2C do MPU6050 (pinos e clock em board_config.h).
// NOTA: a TPJ-01 traz um QMC5883P, incompativel com este driver (feito para o 'L').
// Implementacao bare-metal (sem lib externa) para nao acoplar a Adafruit_Sensor.

#define QMC5883L_ADDR 0x0D

// Registros do QMC5883L
#define QMC5883L_REG_DATA       0x00
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_CONTROL1   0x09
#define QMC5883L_REG_CONTROL2   0x0A
#define QMC5883L_REG_SET_RESET  0x0B

// Ponteiro global para o barramento I2C do QMC5883L
static TwoWire* _qmc_wire = nullptr;

// Variáveis de calibração do magnetômetro (Hard-Iron e Soft-Iron)
static float _mag_offset_x = 0.0f;
static float _mag_offset_y = 0.0f;
static float _mag_offset_z = 0.0f;
static float _mag_scale_x = 1.0f;
static float _mag_scale_y = 1.0f;
static float _mag_scale_z = 1.0f;

void setQMC5883LCalibration(float offset_x, float offset_y, float offset_z,
                            float scale_x, float scale_y, float scale_z) {
    _mag_offset_x = offset_x;
    _mag_offset_y = offset_y;
    _mag_offset_z = offset_z;
    _mag_scale_x = scale_x;
    _mag_scale_y = scale_y;
    _mag_scale_z = scale_z;

    Serial.println("✅ Calibração do QMC5883L configurada:");
    Serial.printf("   Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", offset_x, offset_y, offset_z);
    Serial.printf("   Escalas: X=%.4f, Y=%.4f, Z=%.4f\n", scale_x, scale_y, scale_z);
}

static void _qmc_writeReg(uint8_t reg, uint8_t value) {
    _qmc_wire->beginTransmission(QMC5883L_ADDR);
    _qmc_wire->write(reg);
    _qmc_wire->write(value);
    _qmc_wire->endTransmission();
}

void start_QMC5883L(TwoWire& wireInstance) {
    // Armazena referência ao barramento I2C
    _qmc_wire = &wireInstance;

    // O MPU6050 já inicializou o Wire em `start_IMU_MPU6050`, então não
    // precisamos chamar `wireInstance.begin` nem `wireInstance.setClock` aqui
    // para evitar conflitos na configuração dos pinos que já foi feita (11 e 10).

    // Verifica se o QMC5883L está presente
    _qmc_wire->beginTransmission(QMC5883L_ADDR);
    uint8_t error = _qmc_wire->endTransmission();

    if (error != 0) {
        Serial.println("❌ ERRO: QMC5883L não encontrado no endereço 0x0D!");
        Serial.println("   Verifique as conexões do barramento I2C");
        return;
    }

    // Soft Reset
    _qmc_writeReg(QMC5883L_REG_CONTROL2, 0x80);
    delay(10);

    // Define período SET/RESET
    _qmc_writeReg(QMC5883L_REG_SET_RESET, 0x01);

    // Configura o sensor:
    // Modo Contínuo (0x01), ODR 200Hz (0x0C), Range 8G (0x10), OSR 512 (0x00)
    _qmc_writeReg(QMC5883L_REG_CONTROL1, 0x01 | 0x0C | 0x10 | 0x00);

    Serial.println("✅ QMC5883L inicializado com sucesso! (Mesmo barramento I2C do MPU)");

    delay(100); // Aguarda estabilização
}

void read_QMC5883L(float& mx, float& my, float& mz) {
    if (_qmc_wire == nullptr) {
        mx = my = mz = 0;
        return;
    }

    // Lê 6 bytes de dados (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    _qmc_wire->beginTransmission(QMC5883L_ADDR);
    _qmc_wire->write(QMC5883L_REG_DATA);
    _qmc_wire->endTransmission();

    _qmc_wire->requestFrom((uint8_t)QMC5883L_ADDR, (uint8_t)6);

    if (_qmc_wire->available() >= 6) {
        // No QMC5883L os dados são LSB primeiro e depois MSB!
        int16_t x_raw = _qmc_wire->read() | (_qmc_wire->read() << 8);
        int16_t y_raw = _qmc_wire->read() | (_qmc_wire->read() << 8);
        int16_t z_raw = _qmc_wire->read() | (_qmc_wire->read() << 8);

        // Aplica calibração Hard-Iron (offset) e Soft-Iron (escala)
        float x_cal = (x_raw - _mag_offset_x) * _mag_scale_x;
        float y_cal = (y_raw - _mag_offset_y) * _mag_scale_y;
        float z_cal = (z_raw - _mag_offset_z) * _mag_scale_z;

        // Converte para µT (microtesla)
        // Range 8G: sensibilidade = 3000 LSB/Gauss, 1 Gauss = 100 µT
        // Fator de conversão: valor_LSB / 3000 * 100 = valor_LSB / 30
        const float SCALE_FACTOR = 1.0f / 30.0f;

        mx = x_cal * SCALE_FACTOR;
        my = y_cal * SCALE_FACTOR;
        mz = z_cal * SCALE_FACTOR;
    } else {
        mx = my = mz = 0;
    }
}

// ============= FUNÇÕES DE CÁLCULO DE CONTROLE =============

void calculateMotorOmegaSq(float thrust_signal, float u_torques[],
                           float b_coeff, float d_coeff, float L_arm,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq) {
    // Verifica se os coeficientes são válidos
    if (b_coeff == 0.0f || d_coeff == 0.0f || L_arm == 0.0f) {
        Serial.println("ERRO: Coeficientes b, d ou L não podem ser zero!");
        w1_sq = w2_sq = w3_sq = w4_sq = 0;
        return;
    }

    // Pré-calcula os inversos para otimização.
    // ATENCAO: o denominador vem da INVERSA da matriz de alocacao, nao de um
    // balanco escalar por eixo. Fatorando M = D*H, com D = diag(b, bL, bL, d) e H
    // a matriz de sinais +-1 abaixo, as linhas de H sao ortogonais e H*H^T = 4*I,
    // logo M^-1 = (1/4)*H^T*D^-1 — o fator 1/4 vale para os QUATRO eixos, porque
    // os quatro motores participam de cada um deles (dois sobem, dois descem).
    // Usar 1/(2bL) em roll/pitch (como estava ate 2026-07-28) aplica o DOBRO do
    // torque comandado nesses dois eixos, dobrando o ganho de malha e derrubando
    // a margem de fase — media medida em voo da razao tau_real/u: exatamente 2.000.
    float inv_4b  = 1.0f / (4.0f * b_coeff);
    float inv_4bL = 1.0f / (4.0f * b_coeff * L_arm); // inclui L_arm (denominador do torque de roll/pitch)
    float inv_4d  = 1.0f / (4.0f * d_coeff);

    // Extrai os sinais de controle
    float u1 = thrust_signal; // Empuxo total          [N]
    float u2 = u_torques[0];  // Torque de Rolagem     [N·m]
    float u3 = u_torques[1];  // Torque de Arfagem     [N·m]
    float u4 = u_torques[2];  // Torque de Guinada     [N·m]

    // Matriz de alocação de controle inversa (ω² = T^(-1)*u), config X-quad.
    // Sinais da coluna de yaw casam com o sentido fisico (os motores CCW devem
    // acelerar para u4 > 0).
    //
    // Numeracao dos motores na TPJ-01 (serigrafia da PCB, frente = topo da placa,
    // pinos em include/board_config.h):
    //   M1 = IO8  frente-esquerda (CCW)
    //   M2 = IO48 frente-direita  (CW)
    //   M3 = IO2  tras-direita    (CCW)
    //   M4 = IO4  tras-esquerda   (CW)
    // Em relacao ao drone anterior (M1=FR, M2=RR, M3=RL, M4=FL) isto e' apenas
    // uma permutacao das linhas — nenhum sinal ou coeficiente muda, a sintonia do
    // SDRE segue valida.
    //
    //           [   u1        u2         u3        u4    ]
    // [ω1²]   [1/(4b)   1/(4bL)   -1/(4bL)   1/(4d)]  FL, CCW
    // [ω2²] = [1/(4b)  -1/(4bL)   -1/(4bL)  -1/(4d)]  FR, CW
    // [ω3²]   [1/(4b)  -1/(4bL)    1/(4bL)   1/(4d)]  RR, CCW
    // [ω4²]   [1/(4b)   1/(4bL)    1/(4bL)  -1/(4d)]  RL, CW
    w1_sq = u1 * inv_4b + u2 * inv_4bL - u3 * inv_4bL + u4 * inv_4d; // Motor 1 (FL, CCW)
    w2_sq = u1 * inv_4b - u2 * inv_4bL - u3 * inv_4bL - u4 * inv_4d; // Motor 2 (FR, CW)
    w3_sq = u1 * inv_4b - u2 * inv_4bL + u3 * inv_4bL + u4 * inv_4d; // Motor 3 (RR, CCW)
    w4_sq = u1 * inv_4b + u2 * inv_4bL + u3 * inv_4bL - u4 * inv_4d; // Motor 4 (RL, CW)

    // Garante que não há valores negativos (motores não podem girar ao contrário)
    if (w1_sq < 0) w1_sq = 0;
    if (w2_sq < 0) w2_sq = 0;
    if (w3_sq < 0) w3_sq = 0;
    if (w4_sq < 0) w4_sq = 0;
}

// ============= FUNÇÕES DE DISPLAY =============

void displayGains() {
    float exportedGains[CONTROL_SIZE * STATE_SIZE];
    sdreController.exportGains(exportedGains);

    float exportedKr[CONTROL_SIZE * CONTROL_SIZE];
    sdreController.exportKr(exportedKr);
    Serial.println("Ganhos do LQR calculados com sucesso");

    Serial.println("Ganhos Exportados (K):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < STATE_SIZE; j++) {
            Serial.print(exportedGains[i * STATE_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }

    Serial.println("Matriz Kr (ganho de referência):");
    for (int i = 0; i < CONTROL_SIZE; i++) {
        for (int j = 0; j < CONTROL_SIZE; j++) {
            Serial.print(exportedKr[i * CONTROL_SIZE + j], 6);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void displayIMU(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz, float temp) {
    Serial.print(ax, 6);
    Serial.print("\t");
    Serial.print(ay, 6);
    Serial.print("\t");
    Serial.print(az, 6);
    Serial.print("\t");
    Serial.print(gx, 6);
    Serial.print("\t");
    Serial.print(gy, 6);
    Serial.print("\t");
    Serial.print(gz, 6);
    Serial.print("\t");
    Serial.print(mx, 6);
    Serial.print("\t");
    Serial.print(my, 6);
    Serial.print("\t");
    Serial.print(mz, 6);
    Serial.print("\t");
    Serial.println(temp, 6);
}

void displayStates(float states[]) {
    Serial.print("Roll:");   Serial.print(states[0]);
    Serial.print(",Pitch:"); Serial.print(states[1]);
    Serial.print(",Yaw:");   Serial.print(states[2]);
    Serial.print(",p:");     Serial.print(states[3]);
    Serial.print(",q:");     Serial.print(states[4]);
    Serial.print(",r:");     Serial.println(states[5]);
}

void displayControlSignals(float u_signal[], float thrust_signal) {
    Serial.print("u1:");  Serial.print(u_signal[0], 6);
    Serial.print(",u2:"); Serial.print(u_signal[1], 6);
    Serial.print(",u3:"); Serial.print(u_signal[2], 8); // u3 costuma ser extremamente pequeno por causa do Yaw
    Serial.print(",T:");  Serial.println(thrust_signal, 4);
}

void displayMotorOmegaSq(float thrust_signal, float u_torques[],
                         float b_coeff, float d_coeff, float L_arm) {
    float w1_sq, w2_sq, w3_sq, w4_sq;

    // Usa a função de cálculo
    calculateMotorOmegaSq(thrust_signal, u_torques, b_coeff, d_coeff, L_arm,
                          w1_sq, w2_sq, w3_sq, w4_sq);

    displayMotorOmegaSqDetailed(w1_sq, w2_sq, w3_sq, w4_sq);
}

void displayMotorOmegaSqDetailed(float w1_sq, float w2_sq, float w3_sq, float w4_sq) {
    Serial.printf("M1/FL/IO%d(ω²): %.2f", PIN_MOTOR_1, w1_sq);
    Serial.printf(" | M2/FR/IO%d(ω²): %.2f", PIN_MOTOR_2, w2_sq);
    Serial.printf(" | M3/RR/IO%d(ω²): %.2f", PIN_MOTOR_3, w3_sq);
    Serial.printf(" | M4/RL/IO%d(ω²): %.2f\n", PIN_MOTOR_4, w4_sq);
}
