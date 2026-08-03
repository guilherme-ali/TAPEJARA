#include "led_control.h"

LEDControl::LEDControl() {
    calibrating = false;
    system_ready = false;
    udp_receiving = false;

    status_state = LED_OFF;
    alert_state = LED_OFF;

    last_blink_time = 0;
    blink_state = false;
    blink_interval = 500; // 500ms para blinking normal

    battery_voltage = 0.0f;
    low_battery_detected = false;
    last_battery_check = 0;
}

void LEDControl::begin() {
    // 4 LEDs da TPJ-01: 2 verdes (status) + 2 vermelhos (alerta)
    pinMode(PIN_LED_STATUS_A, OUTPUT);
    pinMode(PIN_LED_STATUS_B, OUTPUT);
    pinMode(PIN_LED_ALERT_A, OUTPUT);
    pinMode(PIN_LED_ALERT_B, OUTPUT);

    // Configura o pino de leitura da bateria
    pinMode(BATTERY_PIN, INPUT);

    // Inicializa todos os LEDs apagados
    digitalWrite(PIN_LED_STATUS_A, LOW);
    digitalWrite(PIN_LED_STATUS_B, LOW);
    digitalWrite(PIN_LED_ALERT_A, LOW);
    digitalWrite(PIN_LED_ALERT_B, LOW);

    // Lê tensão inicial da bateria
    battery_voltage = readBatteryVoltage();
}

void LEDControl::update() {
    unsigned long current_time = millis();

    // Atualiza estado de blinking a cada intervalo
    if (current_time - last_blink_time >= blink_interval) {
        blink_state = !blink_state;
        last_blink_time = current_time;
    }

    // Atualiza os dois canais (o LED branco de energia não é controlável)
    updateLED(PIN_LED_STATUS_A, status_state);
    updateLED(PIN_LED_STATUS_B, status_state);
    updateLED(PIN_LED_ALERT_A, alert_state);
    updateLED(PIN_LED_ALERT_B, alert_state);

    // Verifica bateria a cada 1 segundo
    if (current_time - last_battery_check >= 1000) {
        battery_voltage = readBatteryVoltage();
        last_battery_check = current_time;

        // Atualiza estado de bateria baixa
        if (battery_voltage < BATTERY_LOW_VOLTAGE && battery_voltage > 0.1f) {
            low_battery_detected = true;
        } else if (battery_voltage > BATTERY_LOW_VOLTAGE + 0.2f) {
            low_battery_detected = false;
        }
    }
}

void LEDControl::updateLED(int pin, LEDState state) {
    if (pin < 0) return; // LED não configurado

    switch (state) {
        case LED_OFF:
            digitalWrite(pin, LOW);
            break;

        case LED_FULLY_LIT:
            digitalWrite(pin, HIGH);
            break;

        case LED_BLINKING:
            // Blinking rápido (500ms on/off)
            digitalWrite(pin, blink_state ? HIGH : LOW);
            break;

        case LED_BLINKING_SLOWLY:
            // Blinking lento (1000ms on/off) - usa blink_state mas com intervalo maior
            // Para simplicidade, usa o mesmo blink_state mas poderia ter lógica separada
            digitalWrite(pin, blink_state ? HIGH : LOW);
            break;
    }
}

// Só existem dois LEDs verdes para três condições de status, então elas são
// resolvidas por prioridade — a mais informativa no momento vence.
void LEDControl::resolveStatusState() {
    if (udp_receiving) {
        status_state = LED_FULLY_LIT;   // app conectado: aceso fixo
    } else if (system_ready) {
        status_state = LED_BLINKING;    // pronto, aguardando conexão
        blink_interval = 500;
    } else if (calibrating) {
        status_state = LED_BLINKING_SLOWLY;
        blink_interval = 1000;
    } else {
        status_state = LED_OFF;
    }
}

float LEDControl::readBatteryVoltage() {
    // Lê o ADC (0-4095 para ESP32, 12-bit ADC)
    int adc_value = analogRead(BATTERY_PIN);

    // Converte para tensão (0-3.3V no ADC)
    float adc_voltage = (adc_value / 4095.0f) * 3.3f;

    // Aplica o divisor de tensão (VBAT/2)
    float battery_volts = adc_voltage * VOLTAGE_DIVIDER_RATIO;

    return battery_volts;
}

// ============= CONTROLE DOS LEDS =============
// O LED branco de energia não é controlável (sempre aceso quando há energia).

void LEDControl::setSensorsCalibration(bool active) {
    calibrating = active;
    resolveStatusState();
}

void LEDControl::setSystemReady(bool ready) {
    system_ready = ready;
    resolveStatusState();
}

void LEDControl::setUDPReceiving(bool receiving) {
    udp_receiving = receiving;
    resolveStatusState();
}

void LEDControl::setLowPower(bool low) {
    alert_state = low ? LED_FULLY_LIT : LED_OFF;
}

// ============= FUNÇÕES DE BATERIA =============

float LEDControl::getBatteryVoltage() {
    return battery_voltage;
}

float LEDControl::getBatteryPercentage() {
    // Calcula porcentagem baseado em LiPo 1S (3.0V a 4.2V)
    if (battery_voltage <= BATTERY_CRITICAL_VOLTAGE) return 0.0f;
    if (battery_voltage >= BATTERY_FULL_VOLTAGE) return 100.0f;

    float percentage = ((battery_voltage - BATTERY_CRITICAL_VOLTAGE) /
                       (BATTERY_FULL_VOLTAGE - BATTERY_CRITICAL_VOLTAGE)) * 100.0f;

    return constrain(percentage, 0.0f, 100.0f);
}

bool LEDControl::isLowBattery() {
    return (battery_voltage < BATTERY_LOW_VOLTAGE && battery_voltage > 0.1f);
}

bool LEDControl::isCriticalBattery() {
    return (battery_voltage < BATTERY_CRITICAL_VOLTAGE && battery_voltage > 0.1f);
}

void LEDControl::printStatus() {
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║                   STATUS DOS LEDS                      ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");

    const char* state_names[] = {"OFF", "FULLY_LIT", "BLINKING", "BLINKING_SLOWLY"};

    Serial.println("⚪ WHITE (Power):        Sempre aceso (hardware)");
    Serial.printf("🟢 VERDES (M2/M3):      %s  [calib=%d ready=%d udp=%d]\n",
                  state_names[status_state], calibrating, system_ready, udp_receiving);
    Serial.printf("🔴 VERMELHOS (M1/M4):   %s\n", state_names[alert_state]);

    Serial.println();

    // Status da bateria
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║                  STATUS DA BATERIA                     ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.printf("🔋 Tensão:      %.2fV\n", battery_voltage);
    Serial.printf("📊 Carga:       %.1f%%\n", getBatteryPercentage());

    if (isCriticalBattery()) {
        Serial.println("⚠️  Status:      CRÍTICA! POUPE IMEDIATAMENTE!");
    } else if (isLowBattery()) {
        Serial.println("⚠️  Status:      BAIXA - Considere recarregar");
    } else if (battery_voltage > BATTERY_LOW_VOLTAGE + 0.5f) {
        Serial.println("✅ Status:      OK");
    } else {
        Serial.println("ℹ️  Status:      Normal");
    }

    Serial.println("════════════════════════════════════════════════════════\n");
}
