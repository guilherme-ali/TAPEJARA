#ifndef QMC5883P_H
#define QMC5883P_H

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

/**
 * Driver do magnetômetro QMC5883P da placa Tapejara TPJ-01 (U8).
 *
 * NÃO é o QMC5883L da placa anterior. Diferenças que quebram o driver antigo
 * (datasheet QST doc 13-52-19 rev A):
 *
 *   |                  | QMC5883L        | QMC5883P              |
 *   |------------------|-----------------|-----------------------|
 *   | Endereço I2C     | 0x0D            | **0x2C**              |
 *   | Chip ID          | 0x0D = 0xFF     | 0x00 = 0x80           |
 *   | Dados            | 0x00..0x05      | **0x01..0x06**        |
 *   | Status           | 0x06            | 0x09                  |
 *   | Control 1        | 0x09            | **0x0A**              |
 *   | Control 2        | 0x0A            | 0x0B                  |
 *   | Sensib. em 8 G   | 3000 LSB/G      | **3750 LSB/G**        |
 *
 * O esquemático da TPJ-01 anota "I2C Addr.: 0x0D" no bloco do magnetômetro,
 * mas isso é o endereço do 'L' — a seção 5.4 do datasheet do 'P' diz 0x2C, e as
 * tabelas 10/12 de protocolo confirmam (bits 0101100). Falar com 0x0D produz
 * "requestFrom(): i2cRead returned Error -1", pois não há ninguém lá.
 *
 * Compartilha o barramento I2C do MPU6050; não chama Wire.begin().
 */

// Inicializa o sensor: confere o chip id (0x80) e configura Normal Mode,
// ODR 200 Hz, range ±8 G (sequência do exemplo 7.1 do datasheet).
// Retorna false se o sensor não responder ou o id não bater.
bool start_QMC5883P(TwoWire& wireInstance);

// Calibração hard-iron (offset, em LSB) e soft-iron (escala, adimensional),
// obtida com test/calibrate_magnetometer.cpp.
void setQMC5883PCalibration(float offset_x, float offset_y, float offset_z,
                            float scale_x, float scale_y, float scale_z);

// Lê o campo magnético em µT, já calibrado. Em falha de I2C mantém os últimos
// valores válidos (o Madgwick normaliza o vetor, então um repeat é inofensivo;
// um zero súbito não seria).
void read_QMC5883P(float& mx, float& my, float& mz);

// Leitura crua em LSB, sem calibração — usada por test/calibrate_magnetometer.cpp
// para levantar os coeficientes hard/soft-iron. Retorna false em falha de I2C.
bool read_QMC5883P_raw(int16_t& x, int16_t& y, int16_t& z);

#endif // QMC5883P_H
