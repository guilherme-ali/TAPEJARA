// INSTRUÇÕES:
// 1. Comente todo o conteúdo de main.cpp (ou renomeie temporariamente)
// 2. Compile e execute ESTE arquivo
// 3. Deixe o drone TOTALMENTE PARADO em superfície plana
// 4. Aguarde a calibração completa
// 5. Copie os valores exibidos no Serial
// 6. Cole na main.cpp conforme indicado

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Pinos I2C: única fonte de verdade em include/board_config.h
#include "board_config.h"
#define I2C_SDA PIN_I2C_SDA
#define I2C_SCL PIN_I2C_SCL

Adafruit_MPU6050 mpu;

// Parâmetros de calibração
const int CALIBRATION_SAMPLES = 200000;  // Número de amostras (quanto mais, melhor)
const int WARMUP_SAMPLES = 2000;        // Amostras descartadas no início
const int SAMPLE_DELAY_MS = 5;         // Delay entre amostras (5ms = 200Hz)

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║     CALIBRAÇÃO MPU6050 - Modo de Alta Precisão        ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    
    // Inicializa I2C usando as definições de board_config.h
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz para comunicação rápida
    
    Serial.printf("Configuração I2C:\n");
    Serial.printf("   SDA: GPIO %d\n", I2C_SDA);
    Serial.printf("   SCL: GPIO %d\n", I2C_SCL);
    Serial.println();
    
    // Inicializa MPU6050
    Serial.println("🔍 Procurando MPU6050...");
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("❌ ERRO: MPU6050 não encontrado!");
        Serial.println("   Verifique as conexões:");
        Serial.printf("   - SDA -> GPIO %d\n", I2C_SDA);
        Serial.printf("   - SCL -> GPIO %d\n", I2C_SCL);
        Serial.println("   - VCC -> 3.3V");
        Serial.println("   - GND -> GND");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("✅ MPU6050 encontrado!");
    Serial.println();
    
    // Configura o MPU6050 para máxima precisão
    Serial.println("⚙️  Configurando MPU6050...");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);      // ±2g (maior precisão)
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);           // ±250°/s (maior precisão)
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);        // Filtro passa-baixa 21Hz
    
    Serial.println("   Acelerômetro: ±2g");
    Serial.println("   Giroscópio: ±250°/s");
    Serial.println("   Filtro: 21Hz");
    Serial.println();
    
    // Aguarda estabilização
    Serial.println("⏳ Aguardando estabilização do sensor...");
    delay(2000);
    
    // Aviso importante
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║                    ⚠️  ATENÇÃO ⚠️                        ║");
    Serial.println("║                                                        ║");
    Serial.println("║  1. DEIXE O DRONE TOTALMENTE PARADO                    ║");
    Serial.println("║  2. Coloque em superfície PLANA e NIVELADA             ║");
    Serial.println("║  3. NÃO TOQUE no drone durante a calibração            ║");
    Serial.println("║  4. Evite vibrações na mesa/bancada                    ║");
    Serial.println("║                                                        ║");
    Serial.println("║  A calibração iniciará em 5 segundos...                ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    
    for (int i = 5; i > 0; i--) {
        Serial.printf("   Iniciando em %d...\n", i);
        delay(1000);
    }
    
    Serial.println("\n🚀 INICIANDO CALIBRAÇÃO!\n");
    
    // ========== FASE 1: WARM-UP ==========
    Serial.println("📊 Fase 1/3: Aquecimento do sensor");
    Serial.printf("   Descartando %d amostras iniciais...\n", WARMUP_SAMPLES);
    
    for (int i = 0; i < WARMUP_SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        delay(SAMPLE_DELAY_MS);
        
        if (i % 20 == 0) {
            Serial.print(".");
        }
    }
    Serial.println(" ✓");
    Serial.println();
    
    // ========== FASE 2: COLETA DE DADOS ==========
    Serial.println("📊 Fase 2/3: Coletando dados de calibração");
    Serial.printf("   Processando %d amostras (isso pode levar ~%d segundos)...\n", 
                  CALIBRATION_SAMPLES, (CALIBRATION_SAMPLES * SAMPLE_DELAY_MS) / 1000);
    
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    
    int progress_step = CALIBRATION_SAMPLES / 20; // Mostra progresso a cada 5%
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sum_ax += a.acceleration.x;
        sum_ay += a.acceleration.y;
        sum_az += a.acceleration.z;
        sum_gx += g.gyro.x;
        sum_gy += g.gyro.y;
        sum_gz += g.gyro.z;
        
        delay(SAMPLE_DELAY_MS);
        
        if (i % progress_step == 0) {
            int percent = (i * 100) / CALIBRATION_SAMPLES;
            Serial.printf("   Progresso: %3d%% ", percent);
            for (int j = 0; j < percent / 5; j++) Serial.print("█");
            Serial.println();
        }
    }
    Serial.println("   Progresso: 100% ████████████████████");
    Serial.println(" ✓");
    Serial.println();
    
    // ========== FASE 3: CÁLCULO DOS OFFSETS ==========
    Serial.println("📊 Fase 3/3: Calculando offsets");
    
    // Médias
    float accel_offset_x = sum_ax / CALIBRATION_SAMPLES;
    float accel_offset_y = sum_ay / CALIBRATION_SAMPLES;
    float accel_offset_z = (sum_az / CALIBRATION_SAMPLES) - 9.81; // Subtrai gravidade
    float gyro_offset_x = sum_gx / CALIBRATION_SAMPLES;
    float gyro_offset_y = sum_gy / CALIBRATION_SAMPLES;
    float gyro_offset_z = sum_gz / CALIBRATION_SAMPLES;
    
    // ========== FASE 4: VALIDAÇÃO ==========
    Serial.println();
    Serial.println("🔍 Validando calibração...");
    delay(1000);
    
    // Testa com algumas amostras
    float test_ax = 0, test_ay = 0, test_az = 0;
    float test_gx = 0, test_gy = 0, test_gz = 0;
    const int TEST_SAMPLES = 100;
    
    for (int i = 0; i < TEST_SAMPLES; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        test_ax += a.acceleration.x - accel_offset_x;
        test_ay += a.acceleration.y - accel_offset_y;
        test_az += (a.acceleration.z - accel_offset_z) - 9.81;
        test_gx += g.gyro.x - gyro_offset_x;
        test_gy += g.gyro.y - gyro_offset_y;
        test_gz += g.gyro.z - gyro_offset_z;
        
        delay(SAMPLE_DELAY_MS);
    }
    
    test_ax /= TEST_SAMPLES;
    test_ay /= TEST_SAMPLES;
    test_az /= TEST_SAMPLES;
    test_gx /= TEST_SAMPLES;
    test_gy /= TEST_SAMPLES;
    test_gz /= TEST_SAMPLES;
    
    // ========== RESULTADOS ==========
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║              ✅ CALIBRAÇÃO CONCLUÍDA! ✅                ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    
    Serial.println("📋 OFFSETS CALCULADOS:");
    Serial.println("   ┌─────────────────────────────────────────┐");
    Serial.printf("   │ Accel X: %+.6f m/s²       │\n", accel_offset_x);
    Serial.printf("   │ Accel Y: %+.6f m/s²       │\n", accel_offset_y);
    Serial.printf("   │ Accel Z: %+.6f m/s²       │\n", accel_offset_z);
    Serial.printf("   │ Gyro  X: %+.6f rad/s      │\n", gyro_offset_x);
    Serial.printf("   │ Gyro  Y: %+.6f rad/s      │\n", gyro_offset_y);
    Serial.printf("   │ Gyro  Z: %+.6f rad/s      │\n", gyro_offset_z);
    Serial.println("   └─────────────────────────────────────────┘");
    Serial.println();
    
    Serial.println("🔍 VALIDAÇÃO (valores após calibração):");
    Serial.println("   ┌─────────────────────────────────────────┐");
    Serial.printf("   │ Accel X: %+.6f m/s² (deve ≈0) │\n", test_ax);
    Serial.printf("   │ Accel Y: %+.6f m/s² (deve ≈0) │\n", test_ay);
    Serial.printf("   │ Accel Z: %+.6f m/s² (deve ≈0) │\n", test_az);
    Serial.printf("   │ Gyro  X: %+.6f rad/s (deve ≈0) │\n", test_gx);
    Serial.printf("   │ Gyro  Y: %+.6f rad/s (deve ≈0) │\n", test_gy);
    Serial.printf("   │ Gyro  Z: %+.6f rad/s (deve ≈0) │\n", test_gz);
    Serial.println("   └─────────────────────────────────────────┘");
    Serial.println();
    
    // Verifica qualidade da calibração
    float max_accel_error = max(abs(test_ax), max(abs(test_ay), abs(test_az)));
    float max_gyro_error = max(abs(test_gx), max(abs(test_gy), abs(test_gz)));
    
    Serial.println("📊 QUALIDADE DA CALIBRAÇÃO:");
    Serial.printf("   Erro máximo acelerômetro: %.6f m/s²\n", max_accel_error);
    Serial.printf("   Erro máximo giroscópio:   %.6f rad/s\n", max_gyro_error);
    
    if (max_accel_error < 0.1 && max_gyro_error < 0.01) {
        Serial.println("   Status: ✅ EXCELENTE!");
    } else if (max_accel_error < 0.5 && max_gyro_error < 0.05) {
        Serial.println("   Status: ✅ BOM");
    } else {
        Serial.println("   Status: ⚠️  ACEITÁVEL (considere recalibrar)");
    }
    Serial.println();
    
    // ========== CÓDIGO PARA COPIAR ==========
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║         📋 COPIE ESTE CÓDIGO PARA main.cpp            ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("// Cole estas linhas no início de main.cpp");
    Serial.println();
    Serial.println("┌────────────────────────────────────────────────────────┐");
    Serial.println("│ CÓDIGO PARA COPIAR (início)                           │");
    Serial.println("└────────────────────────────────────────────────────────┘");
    Serial.println();
    Serial.printf("float accel_offset_x = %.6ff;\n", accel_offset_x);
    Serial.printf("float accel_offset_y = %.6ff;\n", accel_offset_y);
    Serial.printf("float accel_offset_z = %.6ff;\n", accel_offset_z);
    Serial.printf("float gyro_offset_x = %.6ff;\n", gyro_offset_x);
    Serial.printf("float gyro_offset_y = %.6ff;\n", gyro_offset_y);
    Serial.printf("float gyro_offset_z = %.6ff;\n", gyro_offset_z);
    Serial.println();
    Serial.println("┌────────────────────────────────────────────────────────┐");
    Serial.println("│ CÓDIGO PARA COPIAR (fim)                              │");
    Serial.println("└────────────────────────────────────────────────────────┘");
    Serial.println();
    
    Serial.println("\n✅ Calibração salva! Agora:");
    Serial.println("   1. Copie o código acima");
    Serial.println("   2. Cole em main.cpp (substitua os valores existentes)");
    Serial.println("   3. Recompile e faça upload da main.cpp");
    Serial.println();
    Serial.println("════════════════════════════════════════════════════════");
}

void loop() {
    // Nada no loop - calibração executada apenas uma vez
    delay(1000);
}