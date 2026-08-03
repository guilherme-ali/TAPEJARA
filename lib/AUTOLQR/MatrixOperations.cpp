#include "MatrixOperations.h"
#include <math.h>
#include <string.h>
#include <new>

// Otimizações específicas para ESP32
#if defined(ESP32)
    #include "esp_attr.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    // Usar DSP library se disponível
    #if __has_include("dsps_mulc.h")
        #include "dsps_mulc.h"
        #include "dsps_add.h"
        #include "dsps_dotprod.h"
        #define USE_ESP_DSP 1
    #endif
#endif

// Macro para alinhamento de memória otimizado (cache line de 32 bytes)
#define ALIGN_32 __attribute__((aligned(32)))

// Constantes otimizadas
static const float EPSILON_INV = 1e6f;  // Para comparações rápidas
static const float REG_FACTOR = 1e-8f;   // Regularização

// ============================================================================
// FUNÇÕES INLINE OTIMIZADAS PARA CASOS ESPECÍFICOS
// ============================================================================

// Multiplicação 2x2 ultra-otimizada
IRAM_ATTR static inline void multiply_2x2(const float* __restrict__ a, 
                                          const float* __restrict__ b, 
                                          float* __restrict__ c) {
    const float a00 = a[0], a01 = a[1], a10 = a[2], a11 = a[3];
    const float b00 = b[0], b01 = b[1], b10 = b[2], b11 = b[3];
    
    c[0] = a00 * b00 + a01 * b10;
    c[1] = a00 * b01 + a01 * b11;
    c[2] = a10 * b00 + a11 * b10;
    c[3] = a10 * b01 + a11 * b11;
}

// Multiplicação 3x3 ultra-otimizada
IRAM_ATTR static inline void multiply_3x3(const float* __restrict__ a, 
                                          const float* __restrict__ b, 
                                          float* __restrict__ c) {
    const float a00 = a[0], a01 = a[1], a02 = a[2];
    const float a10 = a[3], a11 = a[4], a12 = a[5];
    const float a20 = a[6], a21 = a[7], a22 = a[8];
    
    const float b00 = b[0], b10 = b[3], b20 = b[6];
    const float b01 = b[1], b11 = b[4], b21 = b[7];
    const float b02 = b[2], b12 = b[5], b22 = b[8];
    
    c[0] = a00 * b00 + a01 * b10 + a02 * b20;
    c[1] = a00 * b01 + a01 * b11 + a02 * b21;
    c[2] = a00 * b02 + a01 * b12 + a02 * b22;
    c[3] = a10 * b00 + a11 * b10 + a12 * b20;
    c[4] = a10 * b01 + a11 * b11 + a12 * b21;
    c[5] = a10 * b02 + a11 * b12 + a12 * b22;
    c[6] = a20 * b00 + a21 * b10 + a22 * b20;
    c[7] = a20 * b01 + a21 * b11 + a22 * b21;
    c[8] = a20 * b02 + a21 * b12 + a22 * b22;
}

// Multiplicação 4x4 otimizada (comum em SDRE 4 estados)
IRAM_ATTR static inline void multiply_4x4(const float* __restrict__ a, 
                                          const float* __restrict__ b, 
                                          float* __restrict__ c) {
    for (int i = 0; i < 4; i++) {
        const float* a_row = &a[i * 4];
        float* c_row = &c[i * 4];
        
        const float a_i0 = a_row[0], a_i1 = a_row[1], a_i2 = a_row[2], a_i3 = a_row[3];
        
        c_row[0] = a_i0 * b[0] + a_i1 * b[4] + a_i2 * b[8]  + a_i3 * b[12];
        c_row[1] = a_i0 * b[1] + a_i1 * b[5] + a_i2 * b[9]  + a_i3 * b[13];
        c_row[2] = a_i0 * b[2] + a_i1 * b[6] + a_i2 * b[10] + a_i3 * b[14];
        c_row[3] = a_i0 * b[3] + a_i1 * b[7] + a_i2 * b[11] + a_i3 * b[15];
    }
}

// Multiplicação 6x6 otimizada (comum em SDRE 6 estados)
IRAM_ATTR static void multiply_6x6(const float* __restrict__ a, 
                                   const float* __restrict__ b, 
                                   float* __restrict__ c) {
    for (int i = 0; i < 6; i++) {
        const float* a_row = &a[i * 6];
        float* c_row = &c[i * 6];
        
        const float a0 = a_row[0], a1 = a_row[1], a2 = a_row[2];
        const float a3 = a_row[3], a4 = a_row[4], a5 = a_row[5];
        
        c_row[0] = a0*b[0]  + a1*b[6]  + a2*b[12] + a3*b[18] + a4*b[24] + a5*b[30];
        c_row[1] = a0*b[1]  + a1*b[7]  + a2*b[13] + a3*b[19] + a4*b[25] + a5*b[31];
        c_row[2] = a0*b[2]  + a1*b[8]  + a2*b[14] + a3*b[20] + a4*b[26] + a5*b[32];
        c_row[3] = a0*b[3]  + a1*b[9]  + a2*b[15] + a3*b[21] + a4*b[27] + a5*b[33];
        c_row[4] = a0*b[4]  + a1*b[10] + a2*b[16] + a3*b[22] + a4*b[28] + a5*b[34];
        c_row[5] = a0*b[5]  + a1*b[11] + a2*b[17] + a3*b[23] + a4*b[29] + a5*b[35];
    }
}

// ============================================================================
// MULTIPLICAÇÃO DE MATRIZES PRINCIPAL
// ============================================================================

IRAM_ATTR void MatrixOperations::matrixMultiply(const float* __restrict__ m1, 
                                                const float* __restrict__ m2, 
                                                float* __restrict__ result, 
                                                int rows1, int cols1, int cols2)
{
    if (!m1 || !m2 || !result)
        return;

    // Casos especiais otimizados
    if (rows1 == 2 && cols1 == 2 && cols2 == 2) {
        multiply_2x2(m1, m2, result);
        return;
    }
    if (rows1 == 3 && cols1 == 3 && cols2 == 3) {
        multiply_3x3(m1, m2, result);
        return;
    }
    if (rows1 == 4 && cols1 == 4 && cols2 == 4) {
        multiply_4x4(m1, m2, result);
        return;
    }
    if (rows1 == 6 && cols1 == 6 && cols2 == 6) {
        multiply_6x6(m1, m2, result);
        return;
    }

    // Caso geral otimizado com loop reordering e unrolling
    const int total = rows1 * cols2;
    memset(result, 0, total * sizeof(float));

    // Loop otimizado para localidade de cache (ikj order)
    for (int i = 0; i < rows1; i++) {
        const int i_cols1 = i * cols1;
        const int i_cols2 = i * cols2;
        
        for (int k = 0; k < cols1; k++) {
            const float a_ik = m1[i_cols1 + k];
            
            // Skip zero multiplications
            if (a_ik == 0.0f) continue;
            
            const int k_cols2 = k * cols2;
            float* __restrict__ c_row = &result[i_cols2];
            const float* __restrict__ b_row = &m2[k_cols2];
            
            // Unroll by 4
            int j = 0;
            const int cols2_4 = cols2 & ~3;
            for (; j < cols2_4; j += 4) {
                c_row[j]   += a_ik * b_row[j];
                c_row[j+1] += a_ik * b_row[j+1];
                c_row[j+2] += a_ik * b_row[j+2];
                c_row[j+3] += a_ik * b_row[j+3];
            }
            for (; j < cols2; j++) {
                c_row[j] += a_ik * b_row[j];
            }
        }
    }
}

// Multiplicação 6x6 com OUTPUT simétrico: calcula só triângulo superior (21
// produtos) e espelha. Válido apenas quando a*b é matematicamente simétrico.
IRAM_ATTR static void multiply_6x6_symout(const float* __restrict__ a,
                                          const float* __restrict__ b,
                                          float* __restrict__ c) {
    for (int i = 0; i < 6; i++) {
        const float* a_row = &a[i * 6];
        const float a0 = a_row[0], a1 = a_row[1], a2 = a_row[2];
        const float a3 = a_row[3], a4 = a_row[4], a5 = a_row[5];

        for (int j = i; j < 6; j++) {
            const float v = a0 * b[j]      + a1 * b[6 + j]  + a2 * b[12 + j]
                          + a3 * b[18 + j] + a4 * b[24 + j] + a5 * b[30 + j];
            c[i * 6 + j] = v;
            c[j * 6 + i] = v;  // espelha
        }
    }
}

IRAM_ATTR void MatrixOperations::matrixMultiplySymOutput(const float* __restrict__ m1,
                                                         const float* __restrict__ m2,
                                                         float* __restrict__ result,
                                                         int n)
{
    if (!m1 || !m2 || !result) return;

    if (n == 6) {
        multiply_6x6_symout(m1, m2, result);
        return;
    }

    // Caso geral: triângulo superior + espelho
    for (int i = 0; i < n; i++) {
        const float* a_row = &m1[i * n];
        for (int j = i; j < n; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += a_row[k] * m2[k * n + j];
            }
            result[i * n + j] = sum;
            result[j * n + i] = sum;
        }
    }
}

// ============================================================================
// OPERAÇÕES BÁSICAS OTIMIZADAS
// ============================================================================

IRAM_ATTR void MatrixOperations::matrixAdd(const float* __restrict__ m1, 
                                           const float* __restrict__ m2, 
                                           float* __restrict__ result, 
                                           int rows, int cols)
{
    if (!m1 || !m2 || !result) return;

    const int total = rows * cols;
    
#ifdef USE_ESP_DSP
    dsps_add_f32(m1, m2, result, total, 1, 1, 1);
#else
    int i = 0;
    const int total_8 = total & ~7;
    
    // Unroll by 8
    for (; i < total_8; i += 8) {
        result[i]   = m1[i]   + m2[i];
        result[i+1] = m1[i+1] + m2[i+1];
        result[i+2] = m1[i+2] + m2[i+2];
        result[i+3] = m1[i+3] + m2[i+3];
        result[i+4] = m1[i+4] + m2[i+4];
        result[i+5] = m1[i+5] + m2[i+5];
        result[i+6] = m1[i+6] + m2[i+6];
        result[i+7] = m1[i+7] + m2[i+7];
    }
    for (; i < total; i++) {
        result[i] = m1[i] + m2[i];
    }
#endif
}

IRAM_ATTR void MatrixOperations::matrixSubtract(const float* __restrict__ m1, 
                                                const float* __restrict__ m2, 
                                                float* __restrict__ result, 
                                                int rows, int cols)
{
    if (!m1 || !m2 || !result) return;

    const int total = rows * cols;
    int i = 0;
    const int total_8 = total & ~7;
    
    for (; i < total_8; i += 8) {
        result[i]   = m1[i]   - m2[i];
        result[i+1] = m1[i+1] - m2[i+1];
        result[i+2] = m1[i+2] - m2[i+2];
        result[i+3] = m1[i+3] - m2[i+3];
        result[i+4] = m1[i+4] - m2[i+4];
        result[i+5] = m1[i+5] - m2[i+5];
        result[i+6] = m1[i+6] - m2[i+6];
        result[i+7] = m1[i+7] - m2[i+7];
    }
    for (; i < total; i++) {
        result[i] = m1[i] - m2[i];
    }
}

// ============================================================================
// TRANSPOSIÇÃO OTIMIZADA
// ============================================================================

IRAM_ATTR void MatrixOperations::transposeMatrix(const float* __restrict__ matrix, 
                                                 float* __restrict__ result, 
                                                 int rows, int cols)
{
    if (!matrix || !result) return;

    // Casos especiais inline
    if (rows == 2 && cols == 2) {
        result[0] = matrix[0]; result[1] = matrix[2];
        result[2] = matrix[1]; result[3] = matrix[3];
        return;
    }
    if (rows == 3 && cols == 3) {
        result[0] = matrix[0]; result[1] = matrix[3]; result[2] = matrix[6];
        result[3] = matrix[1]; result[4] = matrix[4]; result[5] = matrix[7];
        result[6] = matrix[2]; result[7] = matrix[5]; result[8] = matrix[8];
        return;
    }
    if (rows == 4 && cols == 4) {
        for (int i = 0; i < 4; i++) {
            result[i]      = matrix[i * 4];
            result[i + 4]  = matrix[i * 4 + 1];
            result[i + 8]  = matrix[i * 4 + 2];
            result[i + 12] = matrix[i * 4 + 3];
        }
        return;
    }
    if (rows == 6 && cols == 6) {
        for (int i = 0; i < 6; i++) {
            const int i6 = i * 6;
            result[i]      = matrix[i6];
            result[i + 6]  = matrix[i6 + 1];
            result[i + 12] = matrix[i6 + 2];
            result[i + 18] = matrix[i6 + 3];
            result[i + 24] = matrix[i6 + 4];
            result[i + 30] = matrix[i6 + 5];
        }
        return;
    }

    // Caso geral com blocking para cache
    const int BLOCK = 4;
    for (int i = 0; i < rows; i += BLOCK) {
        const int max_i = (i + BLOCK < rows) ? i + BLOCK : rows;
        for (int j = 0; j < cols; j += BLOCK) {
            const int max_j = (j + BLOCK < cols) ? j + BLOCK : cols;
            for (int ii = i; ii < max_i; ii++) {
                for (int jj = j; jj < max_j; jj++) {
                    result[jj * rows + ii] = matrix[ii * cols + jj];
                }
            }
        }
    }
}

// ============================================================================
// INVERSÃO DE MATRIZES OTIMIZADA
// ============================================================================

// Inversão 2x2 inline
IRAM_ATTR static inline bool invert_2x2(const float* __restrict__ m, float* __restrict__ r) {
    const float det = m[0] * m[3] - m[1] * m[2];
    if (fabsf(det) < 1e-10f) return false;
    
    const float inv_det = 1.0f / det;
    r[0] =  m[3] * inv_det;
    r[1] = -m[1] * inv_det;
    r[2] = -m[2] * inv_det;
    r[3] =  m[0] * inv_det;
    return true;
}

// Inversão 3x3 otimizada
IRAM_ATTR static inline bool invert_3x3(const float* __restrict__ m, float* __restrict__ r) {
    const float a = m[0], b = m[1], c = m[2];
    const float d = m[3], e = m[4], f = m[5];
    const float g = m[6], h = m[7], i = m[8];

    const float ei_fh = e * i - f * h;
    const float fg_di = f * g - d * i;
    const float dh_eg = d * h - e * g;
    
    const float det = a * ei_fh + b * fg_di + c * dh_eg;
    if (fabsf(det) < 1e-10f) return false;

    const float inv_det = 1.0f / det;

    r[0] = ei_fh * inv_det;
    r[1] = (c * h - b * i) * inv_det;
    r[2] = (b * f - c * e) * inv_det;
    r[3] = fg_di * inv_det;
    r[4] = (a * i - c * g) * inv_det;
    r[5] = (c * d - a * f) * inv_det;
    r[6] = dh_eg * inv_det;
    r[7] = (b * g - a * h) * inv_det;
    r[8] = (a * e - b * d) * inv_det;
    return true;
}

// Inversão 4x4 otimizada usando cofatores
IRAM_ATTR static bool invert_4x4_cofactor(const float* __restrict__ m, float* __restrict__ inv) {
    float s0 = m[0] * m[5] - m[4] * m[1];
    float s1 = m[0] * m[6] - m[4] * m[2];
    float s2 = m[0] * m[7] - m[4] * m[3];
    float s3 = m[1] * m[6] - m[5] * m[2];
    float s4 = m[1] * m[7] - m[5] * m[3];
    float s5 = m[2] * m[7] - m[6] * m[3];

    float c5 = m[10] * m[15] - m[14] * m[11];
    float c4 = m[9]  * m[15] - m[13] * m[11];
    float c3 = m[9]  * m[14] - m[13] * m[10];
    float c2 = m[8]  * m[15] - m[12] * m[11];
    float c1 = m[8]  * m[14] - m[12] * m[10];
    float c0 = m[8]  * m[13] - m[12] * m[9];

    float det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    if (fabsf(det) < 1e-10f) return false;

    float invdet = 1.0f / det;

    inv[0]  = ( m[5] * c5 - m[6] * c4 + m[7] * c3) * invdet;
    inv[1]  = (-m[1] * c5 + m[2] * c4 - m[3] * c3) * invdet;
    inv[2]  = ( m[13] * s5 - m[14] * s4 + m[15] * s3) * invdet;
    inv[3]  = (-m[9] * s5 + m[10] * s4 - m[11] * s3) * invdet;

    inv[4]  = (-m[4] * c5 + m[6] * c2 - m[7] * c1) * invdet;
    inv[5]  = ( m[0] * c5 - m[2] * c2 + m[3] * c1) * invdet;
    inv[6]  = (-m[12] * s5 + m[14] * s2 - m[15] * s1) * invdet;
    inv[7]  = ( m[8] * s5 - m[10] * s2 + m[11] * s1) * invdet;

    inv[8]  = ( m[4] * c4 - m[5] * c2 + m[7] * c0) * invdet;
    inv[9]  = (-m[0] * c4 + m[1] * c2 - m[3] * c0) * invdet;
    inv[10] = ( m[12] * s4 - m[13] * s2 + m[15] * s0) * invdet;
    inv[11] = (-m[8] * s4 + m[9] * s2 - m[11] * s0) * invdet;

    inv[12] = (-m[4] * c3 + m[5] * c1 - m[6] * c0) * invdet;
    inv[13] = ( m[0] * c3 - m[1] * c1 + m[2] * c0) * invdet;
    inv[14] = (-m[12] * s3 + m[13] * s1 - m[14] * s0) * invdet;
    inv[15] = ( m[8] * s3 - m[9] * s1 + m[10] * s0) * invdet;

    return true;
}

// Gauss-Jordan otimizado para tamanhos genéricos
/// Ordem maxima resolvida no buffer da pilha. n=6 (SDA da DARE 6x6) cabe folgado:
/// 6*12 floats = 288 B. Acima disso cai no heap, como antes.
static const int GJ_STACK_MAX_N = 8;

IRAM_ATTR static bool invert_gauss_jordan(const float* matrix, float* result, int n) {
    const int n2 = n * 2;

    // O SDA chama isto uma vez por iteracao; um new/delete por chamada custava
    // tempo e fragmentava o heap no caminho quente. Para n <= 8 fica na pilha.
    float stackAug[GJ_STACK_MAX_N * GJ_STACK_MAX_N * 2];
    float* heapAug = nullptr;
    float* aug;
    if (n <= GJ_STACK_MAX_N) {
        aug = stackAug;
    } else {
        heapAug = new(std::nothrow) float[n * n2];
        if (!heapAug) return false;
        aug = heapAug;
    }

    // Inicializar [A|I]
    memset(aug, 0, n * n2 * sizeof(float));
    for (int i = 0; i < n; i++) {
        memcpy(&aug[i * n2], &matrix[i * n], n * sizeof(float));
        aug[i * n2 + n + i] = 1.0f;
    }
    
    // Eliminação com pivotamento parcial
    for (int i = 0; i < n; i++) {
        // Encontrar pivô
        float max_val = fabsf(aug[i * n2 + i]);
        int max_row = i;
        for (int k = i + 1; k < n; k++) {
            const float val = fabsf(aug[k * n2 + i]);
            if (val > max_val) {
                max_val = val;
                max_row = k;
            }
        }
        
        if (max_val < 1e-10f) {
            delete[] heapAug;
            return false;
        }
        
        // Trocar linhas
        if (max_row != i) {
            float* row_i = &aug[i * n2];
            float* row_max = &aug[max_row * n2];
            for (int j = 0; j < n2; j++) {
                const float tmp = row_i[j];
                row_i[j] = row_max[j];
                row_max[j] = tmp;
            }
        }
        
        // Normalizar linha
        const float inv_pivot = 1.0f / aug[i * n2 + i];
        float* row_i = &aug[i * n2];
        for (int j = i; j < n2; j++) {
            row_i[j] *= inv_pivot;
        }
        
        // Eliminar outras linhas
        for (int k = 0; k < n; k++) {
            if (k != i) {
                const float factor = aug[k * n2 + i];
                if (factor != 0.0f) {
                    float* row_k = &aug[k * n2];
                    for (int j = i; j < n2; j++) {
                        row_k[j] -= factor * row_i[j];
                    }
                }
            }
        }
    }
    
    // Extrair resultado
    for (int i = 0; i < n; i++) {
        memcpy(&result[i * n], &aug[i * n2 + n], n * sizeof(float));
    }
    
    delete[] heapAug;
    return true;
}

// IRAM: o SDA chama isto uma vez por iteracao. Os demais kernels de matriz ja
// estavam em IRAM; so este cruzava a fronteira para a flash a cada passo.
IRAM_ATTR bool MatrixOperations::invertMatrix(const float* matrix, float* result, int n)
{
    if (!matrix || !result) return false;

    // Os kernels invert_* marcam entrada e saida como __restrict__, mas o SDA
    // inverte in-place (invertMatrix(X, X, n)). Sob -Os isso passava batido; com
    // -O3 o compilador pode assumir que nao ha alias e reordenar os stores.
    // Copiamos para a pilha quando os ponteiros coincidem.
    if (matrix == result && n <= GJ_STACK_MAX_N) {
        float tmp[GJ_STACK_MAX_N * GJ_STACK_MAX_N];
        memcpy(tmp, matrix, n * n * sizeof(float));
        return MatrixOperations::invertMatrix(tmp, result, n);
    }

    switch (n) {
        case 1:
            if (fabsf(matrix[0]) < 1e-10f) return false;
            result[0] = 1.0f / matrix[0];
            return true;
        case 2:
            return invert_2x2(matrix, result);
        case 3:
            return invert_3x3(matrix, result);
        case 4:
            return invert_4x4_cofactor(matrix, result);
        default:
            return invert_gauss_jordan(matrix, result, n);
    }
}

// ============================================================================
// FUNÇÕES AUXILIARES OTIMIZADAS
// ============================================================================

IRAM_ATTR void MatrixOperations::matrixCopy(const float* source, float* dest, int size)
{
    if (!source || !dest) return;
    memcpy(dest, source, size * sizeof(float));
}

IRAM_ATTR void MatrixOperations::matrixClear(float* matrix, int size)
{
    if (!matrix) return;
    memset(matrix, 0, size * sizeof(float));
}

IRAM_ATTR void MatrixOperations::matrixIdentity(float* matrix, int n)
{
    if (!matrix) return;
    
    const int total = n * n;
    memset(matrix, 0, total * sizeof(float));
    
    // Unroll para tamanhos comuns
    switch (n) {
        case 2: matrix[0] = matrix[3] = 1.0f; break;
        case 3: matrix[0] = matrix[4] = matrix[8] = 1.0f; break;
        case 4: matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1.0f; break;
        case 6: matrix[0] = matrix[7] = matrix[14] = matrix[21] = matrix[28] = matrix[35] = 1.0f; break;
        default:
            for (int i = 0; i < n; i++) {
                matrix[i * n + i] = 1.0f;
            }
    }
}

IRAM_ATTR float MatrixOperations::dotProduct(const float* v1, const float* v2, int size)
{
    if (!v1 || !v2) return 0.0f;
    
#ifdef USE_ESP_DSP
    float result;
    dsps_dotprod_f32(v1, v2, &result, size);
    return result;
#else
    float result = 0.0f;
    int i = 0;
    const int size_4 = size & ~3;
    
    // Unroll by 4 with accumulator
    float acc0 = 0, acc1 = 0, acc2 = 0, acc3 = 0;
    for (; i < size_4; i += 4) {
        acc0 += v1[i]   * v2[i];
        acc1 += v1[i+1] * v2[i+1];
        acc2 += v1[i+2] * v2[i+2];
        acc3 += v1[i+3] * v2[i+3];
    }
    result = acc0 + acc1 + acc2 + acc3;
    
    for (; i < size; i++) {
        result += v1[i] * v2[i];
    }
    return result;
#endif
}

IRAM_ATTR bool MatrixOperations::crossProduct(const float* __restrict__ v1,
                                              const float* __restrict__ v2,
                                              float* __restrict__ result,
                                              int size)
{
    if (!v1 || !v2 || !result || size < 2) return false;

    if (size == 2) {
        result[0] = v1[0] * v2[1] - v1[1] * v2[0];
        result[1] = 0.0f;
        return true;
    }

    if (size == 3) {
        const float x = v1[1] * v2[2] - v1[2] * v2[1];
        const float y = v1[2] * v2[0] - v1[0] * v2[2];
        const float z = v1[0] * v2[1] - v1[1] * v2[0];

        result[0] = x;
        result[1] = y;
        result[2] = z;
        return true;
    }

    return false;
}

IRAM_ATTR void MatrixOperations::matrixVectorMultiply(const float* __restrict__ matrix, 
                                                      const float* __restrict__ vector, 
                                                      float* __restrict__ result, 
                                                      int rows, int cols)
{
    if (!matrix || !vector || !result) return;
    
    for (int i = 0; i < rows; i++) {
        const float* row = &matrix[i * cols];
        
        // Unroll by 4
        float sum = 0.0f;
        int j = 0;
        const int cols_4 = cols & ~3;
        
        float acc0 = 0, acc1 = 0, acc2 = 0, acc3 = 0;
        for (; j < cols_4; j += 4) {
            acc0 += row[j]   * vector[j];
            acc1 += row[j+1] * vector[j+1];
            acc2 += row[j+2] * vector[j+2];
            acc3 += row[j+3] * vector[j+3];
        }
        sum = acc0 + acc1 + acc2 + acc3;
        
        for (; j < cols; j++) {
            sum += row[j] * vector[j];
        }
        result[i] = sum;
    }
}

IRAM_ATTR float MatrixOperations::det2x2(const float* m)
{
    if (!m) return 0.0f;
    return m[0] * m[3] - m[1] * m[2];
}

IRAM_ATTR float MatrixOperations::det3x3(const float* m)
{
    if (!m) return 0.0f;
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

void MatrixOperations::printMatrix(const float* matrix, int rows, int cols, const char* name)
{
    if (!matrix) return;
        
    if (name) {
        Serial.print("Matrix ");
        Serial.print(name);
        Serial.println(":");
    }
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            Serial.print(matrix[i * cols + j], 4);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

// ============================================================================
// DECOMPOSIÇÕES OTIMIZADAS
// ============================================================================

IRAM_ATTR void MatrixOperations::givensRotation(float* matrix, int n, int i, int j, float c, float s)
{
    if (!matrix || i >= n || j >= n) return;
    
    // Aplica rotação nas linhas i e j
    for (int k = 0; k < n; k++) {
        const float temp_i = matrix[i * n + k];
        const float temp_j = matrix[j * n + k];
        matrix[i * n + k] = c * temp_i - s * temp_j;
        matrix[j * n + k] = s * temp_i + c * temp_j;
    }
}

IRAM_ATTR bool MatrixOperations::hessenbergReduction(float* matrix, float* Q, int n)
{
    if (!matrix || !Q || n <= 2) return false;
    
    matrixIdentity(Q, n);
    
    for (int k = 0; k < n - 2; k++) {
        // Encontrar elemento máximo
        float max_val = fabsf(matrix[(k + 1) * n + k]);
        int max_idx = k + 1;
        
        for (int i = k + 2; i < n; i++) {
            const float val = fabsf(matrix[i * n + k]);
            if (val > max_val) {
                max_val = val;
                max_idx = i;
            }
        }
        
        if (max_val < EPSILON) continue;
        
        const float a = matrix[(k + 1) * n + k];
        const float b = matrix[max_idx * n + k];
        const float r = sqrtf(a * a + b * b);
        const float c = a / r;
        const float s = -b / r;
        
        givensRotation(matrix, n, k + 1, max_idx, c, s);
        givensRotation(Q, n, k + 1, max_idx, c, s);
        
        // Aplicar rotação transposta nas colunas
        for (int i = 0; i < n; i++) {
            const float temp_k1 = matrix[i * n + (k + 1)];
            const float temp_max = matrix[i * n + max_idx];
            matrix[i * n + (k + 1)] = c * temp_k1 - s * temp_max;
            matrix[i * n + max_idx] = s * temp_k1 + c * temp_max;
        }
    }
    
    return true;
}

IRAM_ATTR void MatrixOperations::qrDecomposition(float* matrix, float* Q, float* R, int n)
{
    if (!matrix || !Q || !R) return;
    
    matrixCopy(matrix, R, n * n);
    matrixIdentity(Q, n);
    
    for (int j = 0; j < n - 1; j++) {
        for (int i = n - 1; i > j; i--) {
            const float a = R[(i - 1) * n + j];
            const float b = R[i * n + j];
            
            if (fabsf(b) < EPSILON) continue;
            
            const float r = sqrtf(a * a + b * b);
            const float c = a / r;
            const float s = -b / r;
            
            // Aplicar rotação em R
            for (int k = j; k < n; k++) {
                const float temp_im1 = R[(i - 1) * n + k];
                const float temp_i = R[i * n + k];
                R[(i - 1) * n + k] = c * temp_im1 - s * temp_i;
                R[i * n + k] = s * temp_im1 + c * temp_i;
            }
            
            // Aplicar rotação transposta em Q
            for (int k = 0; k < n; k++) {
                const float temp_im1 = Q[k * n + (i - 1)];
                const float temp_i = Q[k * n + i];
                Q[k * n + (i - 1)] = c * temp_im1 - s * temp_i;
                Q[k * n + i] = s * temp_im1 + c * temp_i;
            }
        }
    }
}

IRAM_ATTR bool MatrixOperations::schurDecomposition(float* matrix, float* Q, int n, int max_iterations)
{
    if (!matrix || !Q || n <= 0) return false;
    
    matrixIdentity(Q, n);
    
    // Reduzir para Hessenberg
    float* Q_hess = new(std::nothrow) float[n * n];
    if (!Q_hess) return false;
    
    if (!hessenbergReduction(matrix, Q_hess, n)) {
        delete[] Q_hess;
        return false;
    }
    
    float* temp_Q = new(std::nothrow) float[n * n];
    float* Q_iter = new(std::nothrow) float[n * n];
    float* R_iter = new(std::nothrow) float[n * n];
    
    if (!temp_Q || !Q_iter || !R_iter) {
        delete[] Q_hess;
        delete[] temp_Q;
        delete[] Q_iter;
        delete[] R_iter;
        return false;
    }
    
    matrixMultiply(Q, Q_hess, temp_Q, n, n, n);
    matrixCopy(temp_Q, Q, n * n);
    
    // Iteração QR
    for (int iter = 0; iter < max_iterations; iter++) {
        // Verificar convergência
        bool converged = true;
        for (int i = 0; i < n - 1; i++) {
            if (fabsf(matrix[(i + 1) * n + i]) > EPSILON) {
                converged = false;
                break;
            }
        }
        if (converged) break;
        
        // QR decomposition
        qrDecomposition(matrix, Q_iter, R_iter, n);
        
        // matrix = R * Q
        matrixMultiply(R_iter, Q_iter, matrix, n, n, n);
        
        // Atualizar Q
        matrixMultiply(Q, Q_iter, temp_Q, n, n, n);
        matrixCopy(temp_Q, Q, n * n);
    }
    
    delete[] Q_hess;
    delete[] temp_Q;
    delete[] Q_iter;
    delete[] R_iter;
    
    return true;
}

IRAM_ATTR bool MatrixOperations::computeEigenvalues(const float* matrix, float* eigenvalues_real,
                                                    float* eigenvalues_imag, int n, int max_iterations)
{
    if (!matrix || !eigenvalues_real || !eigenvalues_imag || n <= 0) return false;
    
    float* A = new(std::nothrow) float[n * n];
    float* Q = new(std::nothrow) float[n * n];
    
    if (!A || !Q) {
        delete[] A;
        delete[] Q;
        return false;
    }
    
    matrixCopy(matrix, A, n * n);
    
    if (!schurDecomposition(A, Q, n, max_iterations)) {
        delete[] A;
        delete[] Q;
        return false;
    }
    
    // Extrair autovalores
    int i = 0;
    while (i < n) {
        if (i == n - 1 || fabsf(A[(i + 1) * n + i]) < EPSILON) {
            // Autovalor real
            eigenvalues_real[i] = A[i * n + i];
            eigenvalues_imag[i] = 0.0f;
            i++;
        } else {
            // Bloco 2x2: autovalores complexos
            const float a = A[i * n + i];
            const float b = A[i * n + (i + 1)];
            const float c = A[(i + 1) * n + i];
            const float d = A[(i + 1) * n + (i + 1)];
            
            const float trace = a + d;
            const float det = a * d - b * c;
            const float disc = trace * trace - 4.0f * det;
            
            if (disc < 0) {
                eigenvalues_real[i] = eigenvalues_real[i + 1] = trace * 0.5f;
                eigenvalues_imag[i] = sqrtf(-disc) * 0.5f;
                eigenvalues_imag[i + 1] = -eigenvalues_imag[i];
            } else {
                const float sqrt_disc = sqrtf(disc);
                eigenvalues_real[i] = (trace + sqrt_disc) * 0.5f;
                eigenvalues_real[i + 1] = (trace - sqrt_disc) * 0.5f;
                eigenvalues_imag[i] = eigenvalues_imag[i + 1] = 0.0f;
            }
            i += 2;
        }
    }
    
    delete[] A;
    delete[] Q;
    return true;
}

IRAM_ATTR bool MatrixOperations::solveGeneralizedEigenproblemDARE(const float* H, const float* J,
                                                                  float* stable_eigenvectors, int n)
{
    // Implementação simplificada - retorna false por enquanto
    // Para implementação completa, usar biblioteca Eigen ou implementar QZ algorithm
    (void)H; (void)J; (void)stable_eigenvectors; (void)n;
    return false;
}
