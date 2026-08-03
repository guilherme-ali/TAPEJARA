#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <MadgwickAHRS.h>
#include "utils.h"

// Frequência do loop em Hz
const float SAMPLE_RATE = 100.0f;
const unsigned long LOOP_INTERVAL_US = 1000000 / SAMPLE_RATE;

// Instâncias
Adafruit_MPU6050 mpu;
Madgwick filter;

// Valores de calibração do MPU6050 (copiados do main.cpp)
float accel_offset_x = 0.058127f;
float accel_offset_y = -0.148659f;
float accel_offset_z = 0.018737f;
float gyro_offset_x = -0.011538f;
float gyro_offset_y = 0.015284f;
float gyro_offset_z = 0.010474f;

// Hard-Iron Offsets (valores brutos para subtrair)
const float MAG_OFFSET_X = 26.5f;
const float MAG_OFFSET_Y = 221.5f;
const float MAG_OFFSET_Z = -72.5f;
// Soft-Iron Scales (fatores de escala)
const float MAG_SCALE_X = 1.0243f;
const float MAG_SCALE_Y = 0.9575f;
const float MAG_SCALE_Z = 1.0210f;

unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Inicializa sensores
    start_IMU_MPU6050(mpu); // Isso já faz o Wire.begin (pinos em board_config.h)
    
    // Inicializa magnetômetro (no mesmo barramento I2C)
    start_QMC5883L(Wire);
    // Configura valores nulos ou padrão para a calibração do Mag (ajuste se tiver feito o calibrate_magnetometer)
    setQMC5883LCalibration(MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z, MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);

    // Inicializa o filtro Madgwick
    filter.begin(SAMPLE_RATE);
    
    // Pequeno cabeçalho (O serial plotter ignora texto que não segue o formato)
    Serial.println("Inicialização concluída. Iniciando plotagem...");
    
    lastUpdate = micros();
}

void loop() {
    unsigned long now = micros();
    if (now - lastUpdate >= LOOP_INTERVAL_US) {
        lastUpdate = now;

        float ax, ay, az, gx, gy, gz, mx, my, mz;

        // Lê MPU6050
        read_MPU6050(mpu, ax, ay, az, gx, gy, gz,
                     accel_offset_x, accel_offset_y, accel_offset_z,
                     gyro_offset_x, gyro_offset_y, gyro_offset_z);

        // Lê Magnetômetro
        read_QMC5883L(mx, my, mz);

        // Atualiza o Filtro Madgwick
        // Obs: Madgwick espera giroscópio em graus/s
        
        // CORREÇÃO DOS EIXOS:
        // Na maioria das placas, quando o MPU e o QMC estão soldados virados 
        // para a mesma direção, o eixo Y e Z do QMC são invertidos em relação
        // à regra da mão direita do MPU.
        float mag_x = mx;
        float mag_y = -my;
        float mag_z = -mz;
        
        filter.update(gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG,
                      ax, ay, az, mag_x, mag_y, mag_z);

        // Extrai ângulos de Euler em graus
        float roll = filter.getRoll();
        float pitch = filter.getPitch();
        float yaw = filter.getYaw();
        
        // As velocidades angulares gx, gy, gz já estão em rad/s.
        // Vamos convertê-las para graus/s apenas para ficarem na mesma escala do gráfico de ângulos
        float p = gx * RAD_TO_DEG;
        float q = gy * RAD_TO_DEG;
        float r = gz * RAD_TO_DEG;

        // Formato para o Serial Plotter (Chave:Valor separados por vírgula)
        Serial.print("Roll:"); Serial.print(roll); Serial.print(",");
        Serial.print("Pitch:"); Serial.print(pitch); Serial.print(",");
        Serial.print("Yaw:"); Serial.print(yaw); Serial.print(",");
        Serial.print("p_deg/s:"); Serial.print(p); Serial.print(",");
        Serial.print("q_deg/s:"); Serial.print(q); Serial.print(",");
        Serial.print("r_deg/s:"); Serial.println(r);
    }
}
