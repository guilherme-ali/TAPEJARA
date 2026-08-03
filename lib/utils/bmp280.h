#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"

/**
 * Driver bare-metal do barometro BMP280 da placa Tapejara TPJ-01 (U7, 0x76).
 *
 * Mesmo estilo do driver do QMC5883L em utils.cpp: acesso direto via Wire, sem
 * dependencia externa. NAO chama Wire.begin() — o barramento I2C ja e' aberto por
 * start_IMU_MPU6050() (pinos em board_config.h).
 *
 * Escopo atual: somente leitura. Nao ha estimador de altitude nem realimentacao
 * no controle — o vetor de estados continua sendo so' de atitude.
 */

// Inicializa o sensor: confere o chip id (0x58), le os coeficientes de
// calibracao e configura oversampling/filtro. Retorna false se o sensor nao
// responder ou o id nao bater.
bool start_BMP280(TwoWire& wireInstance);

// Le pressao [Pa] e temperatura [°C] compensadas (burst read de 6 bytes).
// Em caso de falha de I2C mantem os ultimos valores validos.
void read_BMP280(float& pressure_pa, float& temperature_c);

// Altura relativa [m] em relacao a uma pressao de referencia p0 (formula
// barometrica internacional). p0 e' capturado no boot com o drone no chao, entao
// o retorno comeca em ~0.
float bmp280_altitude(float pressure_pa, float p0_pa);

// Media de n leituras de pressao [Pa], usada para fixar a referencia de altura
// zero no setup(). Bloqueia por ~n*10 ms.
float bmp280_reference_pressure(int n_samples);

#endif // BMP280_H
