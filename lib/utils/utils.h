#ifndef UTILS_H
#define UTILS_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "board_config.h"

// Funções de inicialização
// start_IMU_MPU6050 e a unica que chama Wire.begin() (pinos em board_config.h).
void start_IMU_MPU6050(Adafruit_MPU6050& mpu);
void calibrate_MPU6050(Adafruit_MPU6050& mpu,
                       float& accel_offset_x, float& accel_offset_y, float& accel_offset_z,
                       float& gyro_offset_x, float& gyro_offset_y, float& gyro_offset_z);

// Funções do magnetômetro QMC5883L — mesmo barramento I2C do MPU6050.
// ATENCAO: a TPJ-01 traz um QMC5883*P*, com mapa de registradores diferente do
// QMC5883*L*. Estas funcoes NAO funcionam nessa placa; ver board_config.h.
void start_QMC5883L(TwoWire& wireInstance);
void setQMC5883LCalibration(float offset_x, float offset_y, float offset_z,
                            float scale_x, float scale_y, float scale_z);
void read_QMC5883L(float& mx, float& my, float& mz);

// Funções de leitura de sensores
void read_MPU6050(Adafruit_MPU6050& mpu, float& ax, float& ay, float& az,
                  float& gx, float& gy, float& gz,
                  float accel_offset_x, float accel_offset_y, float accel_offset_z,
                  float gyro_offset_x, float gyro_offset_y, float gyro_offset_z);

// Leitura RÁPIDA do MPU6050: burst read cru dos 14 registros via Wire, com as
// MESMAS escalas da Adafruit. Saída idêntica a read_MPU6050 (accel em g, gyro
// rad/s). Mantém a init/config pela lib Adafruit (start_IMU_MPU6050).
// IMPORTANTE: assume os ranges configurados em start_IMU_MPU6050 (2G, 1000°/s).
void read_MPU6050_raw(float& ax, float& ay, float& az,
                      float& gx, float& gy, float& gz,
                      float accel_offset_x, float accel_offset_y, float accel_offset_z,
                      float gyro_offset_x, float gyro_offset_y, float gyro_offset_z);

// Funções de cálculo de controle
void calculateMotorOmegaSq(float thrust_signal, float u_torques[],
                           float b_coeff, float d_coeff, float L_arm,
                           float& w1_sq, float& w2_sq, float& w3_sq, float& w4_sq);

// Funções de display
void displayGains();
void displayIMU(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz, float temp);
void displayStates(float states[]);
void displayControlSignals(float u_signal[], float thrust_signal);
void displayMotorOmegaSq(float thrust_signal, float u_torques[],
                         float b_coeff, float d_coeff, float L_arm);
void displayMotorOmegaSqDetailed(float w1_sq, float w2_sq, float w3_sq, float w4_sq);

#endif
