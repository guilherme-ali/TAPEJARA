#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "board_config.h"

// Configurações dos motores
#define NUM_MOTORS 4

// Pinos GPIO para cada motor — definidos em include/board_config.h (TPJ-01).
// M1 frente-esquerda (CCW), M2 frente-direita (CW),
// M3 trás-direita (CCW),    M4 trás-esquerda (CW).
#define MOTOR_1_PIN PIN_MOTOR_1
#define MOTOR_2_PIN PIN_MOTOR_2
#define MOTOR_3_PIN PIN_MOTOR_3
#define MOTOR_4_PIN PIN_MOTOR_4

// Configurações de PWM
#define PWM_FREQUENCY 25000  // 25 kHz
#define PWM_RESOLUTION 10    // 10 bits (0-1023)
#define PWM_MAX_VALUE 1023

// Canais PWM (ESP32 tem 8 canais PWM)
#define MOTOR_1_CHANNEL 0
#define MOTOR_2_CHANNEL 1
#define MOTOR_3_CHANNEL 2
#define MOTOR_4_CHANNEL 3

// Limites de segurança
#define MIN_THROTTLE 0      // 0% (motores parados)
#define MAX_THROTTLE 100    // 100% (potência máxima)
#define IDLE_THROTTLE 1     // 1% (REDUTOS: idle mínimo para evitar brownout)

class MotorControl {
public:
    MotorControl();
    ~MotorControl();
    
    // Inicialização
    bool begin();
    
    // Controle individual de motores (0-100%)
    void setMotor1(float throttle);
    void setMotor2(float throttle);
    void setMotor3(float throttle);
    void setMotor4(float throttle);
    
    // Controle de todos os motores de uma vez
    void setAllMotors(float throttle);
    void setAllMotors(float motor1, float motor2, float motor3, float motor4);
    
    // Controle baseado em omega quadrado (velocidade angular²)
    void setMotorsFromOmegaSq(float w1_sq, float w2_sq, float w3_sq, float w4_sq);
    
    // Funções de segurança
    void stopAllMotors();
    void armMotors();    // Sequência de armar ESCs
    void disarmMotors(); // Desarmar motores
    bool isArmed() const;
    
    // Calibração de ESCs
    void calibrateESCs(); // Calibração automática dos ESCs
    
    // Limites e mapeamento
    void setThrottleLimits(float min, float max);
    void setOmegaSqLimits(float min, float max);
    
    // Debug
    void printMotorValues();
    
private:
    // Valores de throttle de cada motor (0-100%)
    float motor1_throttle;
    float motor2_throttle;
    float motor3_throttle;
    float motor4_throttle;
    
    // Estado do sistema
    bool armed;
    
    // Limites
    float min_throttle;
    float max_throttle;
    float min_omega_sq;
    float max_omega_sq;
    
    // Funções auxiliares
    float constrainThrottle(float throttle);
    int throttleToPWM(float throttle);
    float omegaSqToThrottle(float omega_sq);
    void writeMotorPWM(int channel, float throttle);
};

#endif