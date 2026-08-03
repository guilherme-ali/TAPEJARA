/**
 * ============================================================================
 * CALIBRAÇÃO DO MAGNETÔMETRO QMC5883P
 * ============================================================================
 * 
 * Este script realiza a calibração do magnetômetro QMC5883P para corrigir:
 * 1. Hard-Iron Distortion (offset) - Campos magnéticos fixos do hardware
 * 2. Soft-Iron Distortion (escala) - Distorções causadas por materiais próximos
 * 
 * COMO USAR:
 * ----------
 * 1. Descomente este arquivo no platformio.ini (ou renomeie main.cpp temporariamente)
 * 2. Compile e faça upload
 * 3. Abra o Serial Monitor (115200 baud)
 * 4. Quando aparecer "INICIANDO CALIBRAÇÃO", gire o sensor LENTAMENTE em todas
 *    as direções por 30 segundos, formando um "8" no ar
 * 5. Copie os valores de calibração exibidos no final
 * 6. Cole os valores no main.cpp nas variáveis de calibração
 * 
 * CONEXÕES:
 * ---------
 * Barramento I2C da placa (pinos em include/board_config.h).
 *
 * CHIP: QMC5883P (Tapejara TPJ-01), endereço 0x2C. Init e leitura vêm de
 * lib/utils/qmc5883p.* — este sketch não duplica o mapa de registradores, que é
 * justamente o que fazia a versão antiga (escrita para o QMC5883L, 0x0D) falhar.
 *
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "board_config.h"
#include "qmc5883p.h"

// ===== CONFIGURAÇÃO =====
#define CALIBRATION_TIME_MS 30000  // Tempo de calibração (30 segundos)
#define SAMPLE_DELAY_MS 20         // Delay entre amostras (50Hz)

// ===== VARIÁVEIS DE CALIBRAÇÃO =====
int16_t x_min = 32767, x_max = -32768;
int16_t y_min = 32767, y_max = -32768;
int16_t z_min = 32767, z_max = -32768;

// Valores calculados
float offset_x, offset_y, offset_z;
float scale_x, scale_y, scale_z;

// ===== ACESSO AO SENSOR (via lib/utils/qmc5883p.*) =====
// Este sketch roda sozinho, então precisa abrir o barramento I2C ele mesmo —
// no firmware quem faz isso é start_IMU_MPU6050().
bool initMagnetometer() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    if (!start_QMC5883P(Wire)) {
        Serial.println("   Verifique as conexões:");
        Serial.printf("   - SDA -> GPIO%d\n", PIN_I2C_SDA);
        Serial.printf("   - SCL -> GPIO%d\n", PIN_I2C_SCL);
        Serial.println("   Ou rode test/i2c_scan.cpp para ver quem responde.");
        return false;
    }

    // Calibração neutra: queremos as amostras CRUAS para levantar os coeficientes.
    setQMC5883PCalibration(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════════╗");
    Serial.println("║        CALIBRAÇÃO DO MAGNETÔMETRO QMC5883P                   ║");
    Serial.println("╚══════════════════════════════════════════════════════════════╝");
    Serial.println();
    
    if (!initMagnetometer()) {
        Serial.println("❌ Falha na inicialização. Reinicie o dispositivo.");
        while(1) delay(1000);
    }
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════════════╗");
    Serial.println("║                    INSTRUÇÕES DE CALIBRAÇÃO                  ║");
    Serial.println("╠══════════════════════════════════════════════════════════════╣");
    Serial.println("║  1. Segure o drone/sensor firmemente                         ║");
    Serial.println("║  2. Quando começar, gire em TODAS as direções:               ║");
    Serial.println("║     - Rotacione no eixo X (roll)                             ║");
    Serial.println("║     - Rotacione no eixo Y (pitch)                            ║");
    Serial.println("║     - Rotacione no eixo Z (yaw)                              ║");
    Serial.println("║  3. Faça movimentos em forma de '8' no ar                    ║");
    Serial.println("║  4. Continue girando por 30 segundos                         ║");
    Serial.println("╚══════════════════════════════════════════════════════════════╝");
    Serial.println();
    
    // Countdown
    for (int i = 5; i > 0; i--) {
        Serial.printf("   Iniciando em %d segundos...\n", i);
        delay(1000);
    }
    
    Serial.println();
    Serial.println("🔄 ═══════════════════════════════════════════════════════════");
    Serial.println("🔄       INICIANDO CALIBRAÇÃO - GIRE O SENSOR AGORA!         ");
    Serial.println("🔄 ═══════════════════════════════════════════════════════════");
    Serial.println();
    
    unsigned long startTime = millis();
    unsigned long lastPrint = 0;
    int sampleCount = 0;
    
    while (millis() - startTime < CALIBRATION_TIME_MS) {
        int16_t x, y, z;
        
        if (read_QMC5883P_raw(x, y, z)) {
            // Atualiza mínimos e máximos
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
            if (z < z_min) z_min = z;
            if (z > z_max) z_max = z;
            
            sampleCount++;
            
            // Print de progresso a cada segundo
            if (millis() - lastPrint >= 1000) {
                int remaining = (CALIBRATION_TIME_MS - (millis() - startTime)) / 1000;
                Serial.printf("⏱️  Tempo restante: %2d s | Amostras: %4d | ", remaining, sampleCount);
                Serial.printf("X:[%+6d,%+6d] Y:[%+6d,%+6d] Z:[%+6d,%+6d]\n",
                             x_min, x_max, y_min, y_max, z_min, z_max);
                lastPrint = millis();
            }
        }
        
        delay(SAMPLE_DELAY_MS);
    }
    
    Serial.println();
    Serial.println("✅ ═══════════════════════════════════════════════════════════");
    Serial.println("✅            CALIBRAÇÃO CONCLUÍDA!                           ");
    Serial.println("✅ ═══════════════════════════════════════════════════════════");
    Serial.println();
    
    // Calcula os offsets (Hard-Iron correction)
    offset_x = (x_max + x_min) / 2.0f;
    offset_y = (y_max + y_min) / 2.0f;
    offset_z = (z_max + z_min) / 2.0f;
    
    // Calcula as escalas (Soft-Iron correction)
    float avg_delta_x = (x_max - x_min) / 2.0f;
    float avg_delta_y = (y_max - y_min) / 2.0f;
    float avg_delta_z = (z_max - z_min) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;
    
    scale_x = avg_delta / avg_delta_x;
    scale_y = avg_delta / avg_delta_y;
    scale_z = avg_delta / avg_delta_z;
    
    // Exibe resultados
    Serial.println("📊 VALORES BRUTOS COLETADOS:");
    Serial.printf("   X: min=%+6d  max=%+6d  (range=%d)\n", x_min, x_max, x_max - x_min);
    Serial.printf("   Y: min=%+6d  max=%+6d  (range=%d)\n", y_min, y_max, y_max - y_min);
    Serial.printf("   Z: min=%+6d  max=%+6d  (range=%d)\n", z_min, z_max, z_max - z_min);
    Serial.printf("   Total de amostras: %d\n", sampleCount);
    Serial.println();
    
    Serial.println("🧲 VALORES DE CALIBRAÇÃO CALCULADOS:");
    Serial.println();
    Serial.println("   Hard-Iron Offsets (para subtrair dos valores brutos):");
    Serial.printf("   offset_x = %.1f\n", offset_x);
    Serial.printf("   offset_y = %.1f\n", offset_y);
    Serial.printf("   offset_z = %.1f\n", offset_z);
    Serial.println();
    Serial.println("   Soft-Iron Scales (para multiplicar após subtrair offset):");
    Serial.printf("   scale_x = %.4f\n", scale_x);
    Serial.printf("   scale_y = %.4f\n", scale_y);
    Serial.printf("   scale_z = %.4f\n", scale_z);
    Serial.println();
    
    // Código pronto para copiar
    Serial.println("╔══════════════════════════════════════════════════════════════╗");
    Serial.println("║     COPIE O CÓDIGO ABAIXO PARA O ARQUIVO main.cpp            ║");
    Serial.println("╚══════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("// ===== CALIBRAÇÃO DO MAGNETÔMETRO QMC5883P =====");
    Serial.printf("const float MAG_OFFSET_X = %.1ff;\n", offset_x);
    Serial.printf("const float MAG_OFFSET_Y = %.1ff;\n", offset_y);
    Serial.printf("const float MAG_OFFSET_Z = %.1ff;\n", offset_z);
    Serial.printf("const float MAG_SCALE_X = %.4ff;\n", scale_x);
    Serial.printf("const float MAG_SCALE_Y = %.4ff;\n", scale_y);
    Serial.printf("const float MAG_SCALE_Z = %.4ff;\n", scale_z);
    Serial.println("// ===============================================");
    Serial.println();
    
    Serial.println("╔══════════════════════════════════════════════════════════════╗");
    Serial.println("║  COLE OS 6 VALORES ACIMA EM src/main.cpp                     ║");
    Serial.println("╚══════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("// Substitua o bloco \"Calibracao QMC5883P\" em src/main.cpp.");
    Serial.println("// setQMC5883PCalibration() ja aplica offset e escala dentro do");
    Serial.println("// driver (lib/utils/qmc5883p.cpp) - nao ha nada a editar la.");
    Serial.println();
    
    // Verifica qualidade da calibração
    Serial.println("📋 VERIFICAÇÃO DA QUALIDADE:");
    float range_ratio_xy = (float)(x_max - x_min) / (y_max - y_min);
    float range_ratio_xz = (float)(x_max - x_min) / (z_max - z_min);
    float range_ratio_yz = (float)(y_max - y_min) / (z_max - z_min);
    
    bool good_cal = true;
    if (range_ratio_xy < 0.7 || range_ratio_xy > 1.4) {
        Serial.println("   ⚠️  Proporção X/Y fora do ideal. Tente girar mais no eixo Z.");
        good_cal = false;
    }
    if (range_ratio_xz < 0.7 || range_ratio_xz > 1.4) {
        Serial.println("   ⚠️  Proporção X/Z fora do ideal. Tente girar mais no eixo Y.");
        good_cal = false;
    }
    if (range_ratio_yz < 0.7 || range_ratio_yz > 1.4) {
        Serial.println("   ⚠️  Proporção Y/Z fora do ideal. Tente girar mais no eixo X.");
        good_cal = false;
    }
    if ((x_max - x_min) < 500 || (y_max - y_min) < 500 || (z_max - z_min) < 500) {
        Serial.println("   ⚠️  Range muito pequeno. Gire o sensor mais amplamente.");
        good_cal = false;
    }
    
    if (good_cal) {
        Serial.println("   ✅ Calibração parece boa! Os ranges estão proporcionais.");
    }
    Serial.println();
    
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println("  Agora o programa mostrará leituras calibradas em tempo real  ");
    Serial.println("  Verifique se o Azimute aponta corretamente para o Norte      ");
    Serial.println("═══════════════════════════════════════════════════════════════");
    Serial.println();
}

void loop() {
    int16_t x_raw, y_raw, z_raw;
    
    if (read_QMC5883P_raw(x_raw, y_raw, z_raw)) {
        // Aplica calibração
        float x_cal = (x_raw - offset_x) * scale_x;
        float y_cal = (y_raw - offset_y) * scale_y;
        float z_cal = (z_raw - offset_z) * scale_z;
        
        // Converte para µT
        const float SCALE_FACTOR = 1.0f / 37.5f;
        float mx = x_cal * SCALE_FACTOR;
        float my = y_cal * SCALE_FACTOR;
        float mz = z_cal * SCALE_FACTOR;
        
        // Calcula azimute (heading)
        float heading = atan2(my, mx) * RAD_TO_DEG;
        if (heading < 0) heading += 360.0f;
        
        // Intensidade total
        float intensity = sqrt(mx*mx + my*my + mz*mz);
        
        // Exibe valores
        Serial.printf("🧭 Mag: X=%+7.2f Y=%+7.2f Z=%+7.2f µT | Azimute: %6.1f° | Intensidade: %5.1f µT\n",
                     mx, my, mz, heading, intensity);
    }
    
    delay(200);
}
