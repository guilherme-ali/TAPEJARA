#include "bmp280.h"
#include <math.h>

// ============= REGISTRADORES (datasheet BMP280, rev 1.23) =============
#define BMP280_REG_CALIB   0x88  // dig_T1..dig_P9, 24 bytes consecutivos
#define BMP280_REG_ID      0xD0
#define BMP280_REG_RESET   0xE0
#define BMP280_REG_STATUS  0xF3
#define BMP280_REG_CTRL    0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_DATA    0xF7  // press_msb/lsb/xlsb + temp_msb/lsb/xlsb

#define BMP280_CHIP_ID     0x58
#define BMP280_RESET_WORD  0xB6

// osrs_t = x1 (001), osrs_p = x4 (011), mode = normal (11).
// A temperatura so' entra na compensacao da pressao, entao x1 basta.
#define BMP280_CTRL_MEAS   0x2F
// t_sb = 0,5 ms (000), IIR filter = x4 (010), spi3w_en = 0.
// ODR resultante ~125 Hz — folgado para a leitura decimada a ~24 Hz do loop.
#define BMP280_CONFIG      0x08

// ============= ESTADO DO DRIVER =============
static TwoWire* bmp_wire = nullptr;

// Coeficientes de calibracao, lidos uma unica vez em start_BMP280
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Ponte entre a compensacao de temperatura e a de pressao (datasheet)
static int32_t t_fine = 0;

// Ultimos valores validos — read_BMP280 os mantem se a transacao I2C falhar
static float last_pressure_pa   = 0.0f;
static float last_temperature_c = 0.0f;

// ============= ACESSO I2C =============

static bool bmp_write8(uint8_t reg, uint8_t value) {
    bmp_wire->beginTransmission(ADDR_BMP280);
    bmp_wire->write(reg);
    bmp_wire->write(value);
    return bmp_wire->endTransmission() == 0;
}

// Le len bytes a partir de reg. Retorna false se o sensor nao responder.
static bool bmp_read(uint8_t reg, uint8_t* buffer, uint8_t len) {
    bmp_wire->beginTransmission(ADDR_BMP280);
    bmp_wire->write(reg);
    if (bmp_wire->endTransmission(false) != 0) return false;

    if (bmp_wire->requestFrom((uint8_t)ADDR_BMP280, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buffer[i] = bmp_wire->read();
    return true;
}

// ============= COMPENSACAO (datasheet, versao em ponto flutuante) =============

// Converte a leitura crua de temperatura em °C e atualiza t_fine.
static float compensate_temperature(int32_t adc_T) {
    float var1 = (((float)adc_T) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
    float var2 = ((((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f) *
                  (((float)adc_T) / 131072.0f - ((float)dig_T1) / 8192.0f)) * ((float)dig_T3);

    t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

// Converte a leitura crua de pressao em Pa. Depende de t_fine, entao
// compensate_temperature() precisa ter rodado antes com a mesma amostra.
static float compensate_pressure(int32_t adc_P) {
    float var1 = ((float)t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)dig_P6) / 32768.0f;
    var2 = var2 + var1 * ((float)dig_P5) * 2.0f;
    var2 = (var2 / 4.0f) + (((float)dig_P4) * 65536.0f);
    var1 = (((float)dig_P3) * var1 * var1 / 524288.0f + ((float)dig_P2) * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * ((float)dig_P1);

    if (var1 == 0.0f) return 0.0f; // evita divisao por zero (datasheet)

    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = ((float)dig_P9) * p * p / 2147483648.0f;
    var2 = p * ((float)dig_P8) / 32768.0f;

    return p + (var1 + var2 + ((float)dig_P7)) / 16.0f;
}

// ============= API PUBLICA =============

bool start_BMP280(TwoWire& wireInstance) {
    bmp_wire = &wireInstance;

    uint8_t chip_id = 0;
    if (!bmp_read(BMP280_REG_ID, &chip_id, 1)) {
        Serial.println("BMP280: sem resposta no endereco 0x76.");
        return false;
    }
    if (chip_id != BMP280_CHIP_ID) {
        Serial.printf("BMP280: chip id inesperado 0x%02X (esperado 0x%02X).\n",
                      chip_id, BMP280_CHIP_ID);
        return false;
    }

    // Soft reset para partir de um estado conhecido (NVM recarrega em ~2 ms)
    bmp_write8(BMP280_REG_RESET, BMP280_RESET_WORD);
    delay(5);

    // Coeficientes de calibracao: 24 bytes little-endian a partir de 0x88
    uint8_t calib[24];
    if (!bmp_read(BMP280_REG_CALIB, calib, 24)) {
        Serial.println("BMP280: falha ao ler coeficientes de calibracao.");
        return false;
    }

    dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dig_T2 = (int16_t) (calib[3] << 8 | calib[2]);
    dig_T3 = (int16_t) (calib[5] << 8 | calib[4]);
    dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dig_P2 = (int16_t) (calib[9] << 8 | calib[8]);
    dig_P3 = (int16_t) (calib[11] << 8 | calib[10]);
    dig_P4 = (int16_t) (calib[13] << 8 | calib[12]);
    dig_P5 = (int16_t) (calib[15] << 8 | calib[14]);
    dig_P6 = (int16_t) (calib[17] << 8 | calib[16]);
    dig_P7 = (int16_t) (calib[19] << 8 | calib[18]);
    dig_P8 = (int16_t) (calib[21] << 8 | calib[20]);
    dig_P9 = (int16_t) (calib[23] << 8 | calib[22]);

    // dig_T1/dig_P1 zerados indicam NVM nao carregada ou sensor errado
    if (dig_T1 == 0 || dig_P1 == 0) {
        Serial.println("BMP280: coeficientes de calibracao invalidos.");
        return false;
    }

    // config antes de ctrl_meas: em modo normal, escritas em 0xF5 sao ignoradas
    if (!bmp_write8(BMP280_REG_CONFIG, BMP280_CONFIG)) return false;
    if (!bmp_write8(BMP280_REG_CTRL, BMP280_CTRL_MEAS)) return false;

    delay(10); // primeira conversao (osrs_p x4 -> ~7,5 ms)

    Serial.printf("BMP280 inicializado (chip id 0x%02X, 0x%02X).\n", chip_id, ADDR_BMP280);
    return true;
}

void read_BMP280(float& pressure_pa, float& temperature_c) {
    uint8_t data[6];
    if (bmp_wire == nullptr || !bmp_read(BMP280_REG_DATA, data, 6)) {
        pressure_pa   = last_pressure_pa;
        temperature_c = last_temperature_c;
        return;
    }

    // 20 bits por grandeza: msb[7:0] lsb[7:0] xlsb[7:4]
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    // Ordem importa: compensate_pressure depende do t_fine desta amostra
    temperature_c = compensate_temperature(adc_T);
    pressure_pa   = compensate_pressure(adc_P);

    last_pressure_pa   = pressure_pa;
    last_temperature_c = temperature_c;
}

float bmp280_altitude(float pressure_pa, float p0_pa) {
    if (p0_pa <= 0.0f || pressure_pa <= 0.0f) return 0.0f;

    // h = 44330 * [1 - (p/p0)^(1/5.255)]
    return 44330.0f * (1.0f - powf(pressure_pa / p0_pa, 1.0f / 5.255f));
}

float bmp280_reference_pressure(int n_samples) {
    if (n_samples < 1) n_samples = 1;

    int count = 0;

    // Espera ate o sensor estabilizar e responder com pressao valida.
    while (count < 1000) {
        float p, t;
        read_BMP280(p, t);
        if (p > 0.0f) count++;
        delay(10); // > tempo de conversao, garante amostras independentes
    }

    float sum = 0.0f;
    float p, t;
    for (int i = 0; i < n_samples; i++) {
        read_BMP280(p, t);
        sum += p;
        delay(10); // > tempo de conversao, garante amostras independentes
    }
    return sum / (float)n_samples;
}
