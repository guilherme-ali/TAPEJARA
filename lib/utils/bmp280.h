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
 *
 * Configuracao: preset "Indoor navigation" da Tabela 7 do datasheet (rev 1.26)
 * — osrs_p x16, osrs_t x2, IIR x16, t_sb 0,5 ms. Da' ODR de 26,3 Hz e ruido de
 * 0,2 Pa (~1,7 cm), ao custo de ~840 ms para 75% de um degrau.
 *
 * Limite do sensor, nao do codigo: o coeficiente de temperatura de offset e' de
 * +-1,5 Pa/K = 12,6 cm/K (Tabela 2). Conforme a PCB esquenta em voo a altura
 * deriva ~12 cm por °C, o que domina o ruido acima. Mitigar isso e' problema de
 * layout termico (espuma sobre o port, distancia do ESP32) ou de fusao com o
 * acelerometro — nenhum registrador resolve.
 */

// Inicializa o sensor: confere o chip id (0x58), le os coeficientes de
// calibracao e configura oversampling/filtro. Retorna false se o sensor nao
// responder ou o id nao bater.
bool start_BMP280(TwoWire& wireInstance);

// Le pressao [Pa] e temperatura [°C] compensadas (burst read de 6 bytes).
// Retorna false se a transacao I2C falhar; nesse caso devolve nas referencias os
// ultimos valores validos.
bool read_BMP280(float& pressure_pa, float& temperature_c);

// Altura relativa [m] em relacao a uma pressao de referencia p0, pela equacao
// hipsometrica com a temperatura medida. p0 e' capturado no boot com o drone no
// chao, entao o retorno comeca em ~0.
float bmp280_altitude(float pressure_pa, float p0_pa, float temperature_c);

// Media de n leituras de pressao [Pa], usada para fixar a referencia de altura
// zero no setup(). Bloqueia por 3000 ms (assentamento do IIR x16) + n*45 ms.
// Retorna 0.0f se o sensor parar de responder — o chamador deve tratar isso.
float bmp280_reference_pressure(int n_samples);

#endif // BMP280_H
