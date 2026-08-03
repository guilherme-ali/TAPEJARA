#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include "board_config.h"

// ============= LEDS DA TPJ-01 =============
// A placa tem 4 LEDs, um por braço: 2 verdes (M2, M3) e 2 vermelhos (M1, M4).
// Não há LED azul como na placa antiga, então os LEDs são agrupados em dois
// canais lógicos que preservam a semântica dos setters públicos:
//
//   STATUS (verdes)   -> calibrando / sistema pronto / UDP conectado
//   ALERTA (vermelhos)-> bateria baixa ou crítica
//
// Pinos em include/board_config.h.

// ============= PINO ADC PARA LEITURA DE BATERIA =============
#define BATTERY_PIN     PIN_VBAT_ADC   // IO1 - divisor R22/R23 (VBAT/2)

// ============= THRESHOLDS DE BATERIA (LiPo 1S - 3.7V Nominal) =============
#define BATTERY_LOW_VOLTAGE   3.4f    // Tensão considerada baixa (~20% restante)
#define BATTERY_CRITICAL_VOLTAGE 3.2f // Tensão crítica (mínimo seguro para LiPo)
#define BATTERY_FULL_VOLTAGE  4.2f    // Tensão máxima carregada (1S LiPo)
#define BATTERY_NOMINAL_VOLTAGE 3.7f  // Tensão nominal da bateria
#define VOLTAGE_DIVIDER_RATIO 2.0f    // Divisor de tensão (VBAT/2)

// ============= ESTADOS DOS LEDS =============
enum LEDState {
    LED_OFF,
    LED_FULLY_LIT,
    LED_BLINKING,
    LED_BLINKING_SLOWLY
};

class LEDControl {
private:
    // Condições de sinalização. O canal STATUS é derivado delas por prioridade
    // (UDP conectado > sistema pronto > calibrando), já que os dois LEDs verdes
    // precisam representar os três estados que antes tinham LEDs separados.
    bool calibrating;
    bool system_ready;
    bool udp_receiving;

    // Estados resolvidos de cada canal
    LEDState status_state;  // LEDs verdes  (M2, M3)
    LEDState alert_state;   // LEDs vermelhos (M1, M4)

    // Controle de blinking
    unsigned long last_blink_time;
    bool blink_state;
    unsigned long blink_interval;

    // Bateria
    float battery_voltage;
    bool low_battery_detected;
    unsigned long last_battery_check;

    // Métodos privados
    void updateLED(int pin, LEDState state);
    void resolveStatusState();
    float readBatteryVoltage();
    
public:
    LEDControl();
    void begin();
    void update();
    
    // Controle individual dos LEDs (LED branco não é controlável)
    void setSensorsCalibration(bool active);
    void setSystemReady(bool ready);
    void setUDPReceiving(bool receiving);
    void setLowPower(bool low);
    
    // Bateria
    float getBatteryVoltage();
    float getBatteryPercentage();
    bool isLowBattery();
    bool isCriticalBattery();
    
    // Debug
    void printStatus();
};

#endif // LED_CONTROL_H
