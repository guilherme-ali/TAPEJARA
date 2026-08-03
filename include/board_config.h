#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/**
 * Pinagem da placa Tapejara TPJ-01 v1.0 (ESP32-S3-WROOM-1-N16R8).
 *
 * FONTE DE VERDADE UNICA: todo pino usado pelo firmware sai daqui. Nao redefinir
 * numero de GPIO em MotorControl.h, led_control.h, utils.cpp ou nos sketches de
 * test/ — a placa antiga (ESP32-S2, fiacao manual) sofria com pinagem duplicada
 * em 5 arquivos.
 *
 * Referencia: tapejaraBoard_v1.pdf (Schematic_TPJ01, 2026-06-15).
 *
 * ===================== GPIOs PROIBIDOS / RESERVADOS =====================
 *
 *   19, 20      USB D-/D+. O USB-C vai direto ao chip (nao ha conversor UART na
 *               placa), entao esses pinos NAO podem ser reaproveitados.
 *   35, 36, 37  PSRAM octal do modulo N16R8 — ligados ao die de PSRAM dentro do
 *               modulo. Indisponiveis por hardware, independente de software.
 *   26 - 32     SPI flash interno; nem sao expostos nos pads do modulo.
 *   0, 3, 45,46 Strapping pins. Deixados sem conexao no esquematico — manter.
 *   43          U0TXD, mas a TPJ-01 usa como LED3. Por isso o build define
 *               ARDUINO_USB_CDC_ON_BOOT=1: Serial precisa sair pelo USB nativo,
 *               nao pelo UART0. Ver platformio.ini.
 *   44          U0RXD, roteado ao header de expansao H4.
 *   7, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 38, 47
 *               Conector da camera OV2640 (U3). Reservados mesmo com a camera
 *               ausente — a trilha ja existe na PCB.
 *
 * Sobram livres para uso futuro: 33, 34, 41 e 42 (41 ja e LED2; 42 vai ao H4).
 * ========================================================================
 */

// ===================== I2C (barramento unico) =====================
// Pull-ups de 4,7 kOhm (R19/R20) ja estao na placa. Tres escravos compartilham
// o mesmo barramento; so start_IMU_MPU6050() chama Wire.begin().
#define PIN_I2C_SDA   39
#define PIN_I2C_SCL   40
#define I2C_CLOCK_HZ  400000  // max do MPU6050 por datasheet

#define ADDR_MPU6050    0x68
#define ADDR_BMP280     0x76  // barometro (U7)
#define ADDR_QMC5883P   0x2C  // magnetometro (U8) — ver nota abaixo

// ATENCAO — O ESQUEMATICO ESTA ERRADO NESTE PONTO.
// O bloco "Magnetometer / Compass" do tapejaraBoard_v1.pdf anota "I2C Addr.: 0x0D",
// mas 0x0D e' o endereco do QMC5883*L* (o chip da placa anterior). A TPJ-01 monta
// um QMC5883*P*, cujo endereco de fabrica e' 0x2C — datasheet QST 13-52-19 rev A,
// secao 5.4, confirmado pelas tabelas 10 e 12 de protocolo (bits 0101100).
// Falar com 0x0D produz "requestFrom(): i2cRead returned Error -1".
//
// Os dois chips tambem tem mapas de registradores diferentes, entao o 'P' usa
// driver proprio (lib/utils/qmc5883p.*), nao o start_QMC5883L de utils.cpp.
// Confirme com test/i2c_scan.cpp quem responde no barramento.

// ===================== PWM dos motores =====================
// Ordem Q1..Q4 do bloco PWM do esquematico, casando com os bracos M1..M4
// serigrafados na PCB. Frente do drone = topo do PCB (entre M1 e M2).
#define PIN_MOTOR_1    8   // M1 — frente-esquerda, CCW
#define PIN_MOTOR_2   48   // M2 — frente-direita,  CW
#define PIN_MOTOR_3    2   // M3 — tras-direita,    CCW
#define PIN_MOTOR_4    4   // M4 — tras-esquerda,   CW

// ===================== LEDs (um por braco) =====================
// Bloco "Sinalization": 2 verdes + 2 vermelhos. Nao ha LED azul como na placa
// antiga; LEDControl agrupa os verdes como canal de status e os vermelhos como
// canal de alerta.
#define PIN_LED_M1     6   // vermelho — braco M1
#define PIN_LED_M2    41   // verde    — braco M2
#define PIN_LED_M3    43   // verde    — braco M3 (ex-U0TXD)
#define PIN_LED_M4     5   // vermelho — braco M4

#define PIN_LED_STATUS_A  PIN_LED_M2  // verdes
#define PIN_LED_STATUS_B  PIN_LED_M3
#define PIN_LED_ALERT_A   PIN_LED_M1  // vermelhos
#define PIN_LED_ALERT_B   PIN_LED_M4

// ===================== ADC de bateria =====================
// Divisor R22/R23 = 100k/100k sobre VBAT -> razao 2,0. IO1 = ADC1_CH0.
#define PIN_VBAT_ADC   1

#endif // BOARD_CONFIG_H
