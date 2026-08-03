#include "qmc5883p.h"

// ============= REGISTRADORES (datasheet QST 13-52-19 rev A, tabela 14) =============
#define QMC5883P_REG_CHIPID  0x00  // read only, valor esperado 0x80
#define QMC5883P_REG_DATA    0x01  // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
#define QMC5883P_REG_STATUS  0x09  // bit1 OVFL, bit0 DRDY
#define QMC5883P_REG_CTRL1   0x0A  // OSR2<7:6> OSR1<5:4> ODR<3:2> MODE<1:0>
#define QMC5883P_REG_CTRL2   0x0B  // SOFT_RST<7> SELF_TEST<6> - - RNG<3:2> SET/RESET<1:0>
#define QMC5883P_REG_SIGN    0x29  // define o sinal dos eixos X/Y/Z (ver nota abaixo)

#define QMC5883P_CHIP_ID     0x80

// Sequência do exemplo 7.1 "Normal Mode Setup" do datasheet.
#define QMC5883P_SIGN_VALUE  0x06  // valor exigido pelo datasheet em 0x29
#define QMC5883P_CTRL2_INIT  0x08  // RNG = ±8 G (10), SET/RESET MODE = set and reset on (00)
#define QMC5883P_CTRL1_INIT  0xCD  // OSR2 = 8, OSR1 = 8, ODR = 200 Hz (11), MODE = normal (01)

// Sensibilidade em ±8 G (tabela 2 do datasheet): 3750 LSB/Gauss.
// 1 Gauss = 100 µT  ->  µT = LSB / 3750 * 100 = LSB / 37.5
#define QMC5883P_LSB_PER_UT  37.5f

// Alinhamento com o frame do MPU6050: a tríade do QMC5883P sai girada 180° em
// torno de X, o que invertia o sentido do yaw fundido em relação ao integrado
// pelo giroscópio. (x, -y, -z) é rotação própria — negar só Y também corrigiria
// o azimute, mas como reflexão deixaria o eixo vertical errado e o heading
// passaria a derivar com o tilt. Ajuste só estes três sinais se a montagem mudar.
#define QMC5883P_AXIS_SIGN_X  (+1.0f)
#define QMC5883P_AXIS_SIGN_Y  (-1.0f)
#define QMC5883P_AXIS_SIGN_Z  (-1.0f)

// ============= ESTADO DO DRIVER =============
static TwoWire* _qmc_wire = nullptr;

static float _mag_offset_x = 0.0f;
static float _mag_offset_y = 0.0f;
static float _mag_offset_z = 0.0f;
static float _mag_scale_x = 1.0f;
static float _mag_scale_y = 1.0f;
static float _mag_scale_z = 1.0f;

// Últimas leituras válidas, devolvidas em caso de falha de I2C
static float _last_mx = 0.0f;
static float _last_my = 0.0f;
static float _last_mz = 0.0f;

// ============= ACESSO I2C =============

static bool _qmc_write8(uint8_t reg, uint8_t value) {
    _qmc_wire->beginTransmission(ADDR_QMC5883P);
    _qmc_wire->write(reg);
    _qmc_wire->write(value);
    return _qmc_wire->endTransmission() == 0;
}

static bool _qmc_read(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _qmc_wire->beginTransmission(ADDR_QMC5883P);
    _qmc_wire->write(reg);
    if (_qmc_wire->endTransmission(false) != 0) return false;

    if (_qmc_wire->requestFrom((uint8_t)ADDR_QMC5883P, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buffer[i] = _qmc_wire->read();
    return true;
}

// ============= API PUBLICA =============

void setQMC5883PCalibration(float offset_x, float offset_y, float offset_z,
                            float scale_x, float scale_y, float scale_z) {
    _mag_offset_x = offset_x;
    _mag_offset_y = offset_y;
    _mag_offset_z = offset_z;
    _mag_scale_x = scale_x;
    _mag_scale_y = scale_y;
    _mag_scale_z = scale_z;

    Serial.println("✅ Calibração do QMC5883P configurada:");
    Serial.printf("   Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", offset_x, offset_y, offset_z);
    Serial.printf("   Escalas: X=%.4f, Y=%.4f, Z=%.4f\n", scale_x, scale_y, scale_z);
}

bool start_QMC5883P(TwoWire& wireInstance) {
    _qmc_wire = &wireInstance;

    // O barramento já foi aberto por start_IMU_MPU6050().
    uint8_t chip_id = 0;
    if (!_qmc_read(QMC5883P_REG_CHIPID, &chip_id, 1)) {
        Serial.printf("❌ QMC5883P: sem resposta no endereço 0x%02X.\n", ADDR_QMC5883P);
        Serial.println("   Rode test/i2c_scan.cpp para ver quem responde no barramento.");
        _qmc_wire = nullptr;
        return false;
    }
    if (chip_id != QMC5883P_CHIP_ID) {
        Serial.printf("❌ QMC5883P: chip id 0x%02X (esperado 0x%02X).\n",
                      chip_id, QMC5883P_CHIP_ID);
        _qmc_wire = nullptr;
        return false;
    }

    // Soft reset — volta todos os registradores ao default (Suspend Mode)
    _qmc_write8(QMC5883P_REG_CTRL2, 0x80);
    delay(10);

    // Sequência exata do exemplo 7.1 do datasheet, nesta ordem.
    // O registrador 0x29 não aparece na tabela de registradores, mas todos os
    // exemplos de inicialização exigem escrever 0x06 nele ("Define the sign for
    // X Y and Z axis"). Sem isso os eixos saem com sinal errado.
    if (!_qmc_write8(QMC5883P_REG_SIGN, QMC5883P_SIGN_VALUE)) return false;
    if (!_qmc_write8(QMC5883P_REG_CTRL2, QMC5883P_CTRL2_INIT)) return false;
    if (!_qmc_write8(QMC5883P_REG_CTRL1, QMC5883P_CTRL1_INIT)) return false;

    delay(20); // primeira conversão a 200 Hz

    Serial.printf("✅ QMC5883P inicializado (0x%02X, chip id 0x%02X, ±8 G, 200 Hz).\n",
                  ADDR_QMC5883P, chip_id);
    return true;
}

bool read_QMC5883P_raw(int16_t& x, int16_t& y, int16_t& z) {
    if (_qmc_wire == nullptr) return false;

    uint8_t data[6];
    if (!_qmc_read(QMC5883P_REG_DATA, data, 6)) return false;

    // 16 bits em complemento de dois, LSB primeiro (tabela 15).
    x = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    y = (int16_t)((uint16_t)data[3] << 8 | data[2]);
    z = (int16_t)((uint16_t)data[5] << 8 | data[4]);
    return true;
}

void read_QMC5883P(float& mx, float& my, float& mz) {
    if (_qmc_wire == nullptr) {
        mx = my = mz = 0.0f;
        return;
    }

    int16_t x_raw, y_raw, z_raw;
    if (!read_QMC5883P_raw(x_raw, y_raw, z_raw)) {
        mx = _last_mx;
        my = _last_my;
        mz = _last_mz;
        return;
    }

    // Hard-iron (offset) e soft-iron (escala), ambos em LSB. Antes do
    // remapeamento de eixos: os coeficientes vêm de read_QMC5883P_raw(), no
    // frame cru, e inverter o sinal antes deslocaria o centro do elipsoide.
    float x_cal = (x_raw - _mag_offset_x) * _mag_scale_x;
    float y_cal = (y_raw - _mag_offset_y) * _mag_scale_y;
    float z_cal = (z_raw - _mag_offset_z) * _mag_scale_z;

    // Frame cru do chip -> frame do MPU6050 (ver nota em QMC5883P_AXIS_SIGN_*).
    mx = QMC5883P_AXIS_SIGN_X * x_cal / QMC5883P_LSB_PER_UT;
    my = QMC5883P_AXIS_SIGN_Y * y_cal / QMC5883P_LSB_PER_UT;
    mz = QMC5883P_AXIS_SIGN_Z * z_cal / QMC5883P_LSB_PER_UT;

    _last_mx = mx;
    _last_my = my;
    _last_mz = mz;
}
