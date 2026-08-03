#include "AutoLQR.h"
#include <math.h>
#include <ArduinoEigen.h>
#include <string.h>  // Para strcmp
#include <algorithm> // Para std::sort
#include <stdint.h>

// Tamanho do scratch do SDA: 11 blocos n*n (Ak,Gk,Hk,*_next,AT,W,Temp1..3),
// 3 blocos m*m (R_inv,BT_P_B,R_plus_BTPB) e 4 blocos m*n (BT,B_Rinv,BT_P,BT_P_A).
// n=6,m=3 -> 495 floats (1980 B).
static constexpr int SDA_SCRATCH_FLOATS(int n, int m) {
    return 11 * n * n + 3 * m * m + 4 * m * n;
}

// Teto de iteracoes do SDA. Caracterizado no host sobre 6 pontos do envelope
// (hover, roll/pitch ate 50 graus, taxas 350 deg/s, acoplamento giroscopico):
// converge em 10-11 iteracoes com residuo relativo da DARE ~4e-8. 40 da folga
// de ~3,6x e serve so como rede de seguranca; era 100.
#ifndef SDA_MAX_ITERATIONS
#define SDA_MAX_ITERATIONS 40
#endif

// ============================================================================
// SDA em fixed-point Q13.18 (int32) — caminho rápido para ESP32-S2 sem FPU.
// Validado end-to-end: erro do K < 1% vs float, ~2.7× mais rápido, erro é
// puramente quantização (não amplifica com κ). Range ±8192 dá margem 2.7×
// sobre o pico observado (maxGk≈2980, estado-dependente no SDRE).
// A flag g_q_ovf sinaliza saturação → o chamador faz fallback para o SDA float.
// ============================================================================
namespace {
typedef int32_t q16_t;
static const int Q_SHIFT = 18;     // Q13.18: 13 bits inteiros (±8192), 18 fracionários (res 3.8e-6)
static bool g_q_ovf = false;       // overflow/saturação detectado no run atual
static int  g_q_iters = 0;         // nº de iterações do último sda_q (telemetria)

// Conversões parametrizadas pelo nº de bits fracionários (sh)
static inline q16_t f2q(float x, int sh) {
    double v = (double)x * (double)(1u << sh);
    if (v >  2147483647.0) { g_q_ovf = true; return INT32_MAX; }
    if (v < -2147483648.0) { g_q_ovf = true; return INT32_MIN; }
    return (q16_t)llround(v);
}
static inline float q2f(q16_t x, int sh) { return (float)x / (float)(1u << sh); }

static inline q16_t qmul(q16_t a, q16_t b, int sh) {
    int64_t r = ((int64_t)a * (int64_t)b) >> sh;
    if (r > INT32_MAX) { g_q_ovf = true; return INT32_MAX; }
    if (r < INT32_MIN) { g_q_ovf = true; return INT32_MIN; }
    return (q16_t)r;
}
static inline q16_t qdiv(q16_t a, q16_t b, int sh) {
    if (b == 0) { g_q_ovf = true; return (a >= 0) ? INT32_MAX : INT32_MIN; }
    int64_t r = ((int64_t)a << sh) / b;     // estoura se b minúsculo → clamp + flag
    if (r > INT32_MAX) { g_q_ovf = true; return INT32_MAX; }
    if (r < INT32_MIN) { g_q_ovf = true; return INT32_MIN; }
    return (q16_t)r;
}
static void matmul_q(const q16_t* a, const q16_t* b, q16_t* c, int r1, int c1, int c2, int sh) {
    for (int i = 0; i < r1; i++)
        for (int j = 0; j < c2; j++) {
            int64_t acc = 0;
            for (int k = 0; k < c1; k++) acc += (int64_t)a[i * c1 + k] * (int64_t)b[k * c2 + j];
            int64_t r = acc >> sh;
            if (r > INT32_MAX)      { g_q_ovf = true; r = INT32_MAX; }
            else if (r < INT32_MIN) { g_q_ovf = true; r = INT32_MIN; }
            c[i * c2 + j] = (q16_t)r;
        }
}
static void transpose_q(const q16_t* a, q16_t* at, int r, int c) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++) at[j * r + i] = a[i * c + j];
}
static void add_q(const q16_t* a, const q16_t* b, q16_t* c, int n) {
    for (int i = 0; i < n; i++) c[i] = a[i] + b[i];
}
// Inversão Gauss-Jordan fixed-point parametrizada pelo shift
static bool invert_q(const q16_t* src, q16_t* dst, int n, int sh) {
    static q16_t aug[6 * 12];
    const int n2 = 2 * n;
    const q16_t one = f2q(1.0f, sh);
    const q16_t pivot_floor = f2q(1e-4f, sh);   // piso > resolução fixed-point
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) aug[i * n2 + j] = src[i * n + j];
        for (int j = 0; j < n; j++) aug[i * n2 + n + j] = (i == j) ? one : 0;
    }
    for (int i = 0; i < n; i++) {
        q16_t maxv = aug[i * n2 + i]; if (maxv < 0) maxv = -maxv;
        int mr = i;
        for (int k = i + 1; k < n; k++) {
            q16_t v = aug[k * n2 + i]; if (v < 0) v = -v;
            if (v > maxv) { maxv = v; mr = k; }
        }
        if (maxv < pivot_floor) return false;   // ~singular no domínio fixed-point
        if (mr != i)
            for (int j = 0; j < n2; j++) { q16_t t = aug[i * n2 + j]; aug[i * n2 + j] = aug[mr * n2 + j]; aug[mr * n2 + j] = t; }
        q16_t piv = aug[i * n2 + i];
        for (int j = 0; j < n2; j++) aug[i * n2 + j] = qdiv(aug[i * n2 + j], piv, sh);
        for (int k = 0; k < n; k++)
            if (k != i) {
                q16_t f = aug[k * n2 + i];
                if (f != 0)
                    for (int j = 0; j < n2; j++) aug[k * n2 + j] -= qmul(f, aug[i * n2 + j], sh);
            }
    }
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) dst[i * n + j] = aug[i * n2 + n + j];
    return true;
}
// SDA completo em fixed-point. Converge internamente (critério análogo ao float,
// tolerância relaxada p/ o piso de quantização). Retorna false em singular/overflow.
// Pout/Kout recebem P e K na representação fixed-point do shift sh.
static bool sda_q(const q16_t* A, const q16_t* B, const q16_t* Q, const q16_t* R,
                  q16_t* Kout, q16_t* Pout, int n, int m, int sh) {
    static q16_t Ak[36], Gk[36], Hk[36], Akn[36], Gkn[36], Hkn[36];
    static q16_t Rinv[9], BT[18], AT[36], W[36], T1[36], T2[36], T3[36], BRi[18];
    static q16_t BTP[18], BTPB[9], Rp[9], BTPA[18];
    const int nn = n * n;
    const q16_t one = f2q(1.0f, sh);
    const int maxIter = 25;
    g_q_iters = maxIter;

    memcpy(Ak, A, nn * sizeof(q16_t));
    transpose_q(B, BT, n, m);
    if (!invert_q(R, Rinv, m, sh)) return false;
    matmul_q(B, Rinv, BRi, n, m, m, sh);
    matmul_q(BRi, BT, Gk, n, m, n, sh);
    memcpy(Hk, Q, nn * sizeof(q16_t));

    for (int it = 0; it < maxIter; it++) {
        matmul_q(Gk, Hk, T1, n, n, n, sh);
        for (int i = 0; i < n; i++) T1[i * n + i] += one;
        if (!invert_q(T1, W, n, sh)) return false;
        matmul_q(Ak, W, T1, n, n, n, sh);     // T1 = Ak·W
        matmul_q(T1, Ak, Akn, n, n, n, sh);
        transpose_q(Ak, AT, n, n);
        matmul_q(Gk, AT, T2, n, n, n, sh);
        matmul_q(T1, T2, T3, n, n, n, sh);
        add_q(Gk, T3, Gkn, nn);
        matmul_q(W, Ak, T2, n, n, n, sh);
        matmul_q(Hk, T2, T3, n, n, n, sh);
        matmul_q(AT, T3, T2, n, n, n, sh);
        add_q(Hk, T2, Hkn, nn);

        // Convergência: max|ΔHk| / max|Hk| < 1e-3 (piso fixed-point ~3.8e-6)
        q16_t dmax = 0, hmax = 0;
        for (int i = 0; i < nn; i++) {
            q16_t d = Hkn[i] - Hk[i]; if (d < 0) d = -d;
            q16_t h = Hk[i];          if (h < 0) h = -h;
            if (d > dmax) dmax = d;
            if (h > hmax) hmax = h;
        }
        memcpy(Ak, Akn, nn * sizeof(q16_t));
        memcpy(Gk, Gkn, nn * sizeof(q16_t));
        memcpy(Hk, Hkn, nn * sizeof(q16_t));
        if ((int64_t)dmax * 1000 < (int64_t)hmax) { g_q_iters = it + 1; break; }   // convergiu
    }
    if (Pout) memcpy(Pout, Hk, nn * sizeof(q16_t));   // P = Hk

    // K = (R + B'·P·B)^-1 · B'·P·A
    matmul_q(BT, Hk, BTP, m, n, n, sh);
    matmul_q(BTP, B, BTPB, m, n, m, sh);
    add_q(R, BTPB, Rp, m * m);
    if (!invert_q(Rp, Rp, m, sh)) return false;
    matmul_q(BTP, A, BTPA, m, n, n, sh);
    matmul_q(Rp, BTPA, Kout, m, m, n, sh);
    return true;
}
} // namespace
// ============================================================================

AutoLQR::AutoLQR(int stateSize, int controlSize)
    : stateSize(stateSize)
    , controlSize(controlSize)
    , A(nullptr)
    , B(nullptr)
    , Q(nullptr)
    , R(nullptr)
    , K(nullptr)
    , state(nullptr)
    , P(nullptr)
    , Kr(nullptr)
    , reference(nullptr)
    , lastIterations(-1)
    , lastResidual(-1.0f)
    , residualHistoryCount(0)
    , sdaScratch(nullptr)
{
    // Inicializar histórico de resíduos
    for (int i = 0; i < 10; i++) {
        residualHistory[i] = 0.0f;
    }
    
    if (stateSize > 0 && controlSize > 0) {
        A = new float[stateSize * stateSize]();
        B = new float[stateSize * controlSize]();
        Q = new float[stateSize * stateSize]();
        R = new float[controlSize * controlSize]();
        K = new float[controlSize * stateSize]();
        state = new float[stateSize]();
        P = new float[stateSize * stateSize]();
        Kr = new float[controlSize * controlSize]();
        reference = new float[controlSize]();
        sdaScratch = new float[SDA_SCRATCH_FLOATS(stateSize, controlSize)]();
    }
}

AutoLQR::~AutoLQR()
{
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] K;
    delete[] state;
    delete[] P;
    delete[] Kr;
    delete[] reference;
    delete[] sdaScratch;
}

bool AutoLQR::setStateMatrix(const float* inputA)
{
    if (!inputA || !A)
        return false;
    matrixCopy(inputA, A, stateSize * stateSize);
    return true;
}

bool AutoLQR::setInputMatrix(const float* inputB)
{
    if (!inputB || !B)
        return false;
    matrixCopy(inputB, B, stateSize * controlSize);
    return true;
}

bool AutoLQR::setCostMatrices(const float* inputQ, const float* inputR)
{
    if (!inputQ || !inputR || !Q || !R)
        return false;
    matrixCopy(inputQ, Q, stateSize * stateSize);
    matrixCopy(inputR, R, controlSize * controlSize);
    return true;
}

void AutoLQR::setGains(const float* inputK)
{
    if (!inputK || !K)
        return;
    matrixCopy(inputK, K, controlSize * stateSize);
}

bool AutoLQR::computeGains(const char* method)
{
    bool K_flag = false;
    
    if (strcmp(method, "SDA") == 0) {
        K_flag = computeGainMatrixSDA(); 
    } else if (strcmp(method, "SDA_FIXED") == 0) {
        K_flag = computeGainMatrixSDA_Fixed(); // caminho rápido fixed-point Q13.18
    } else if (strcmp(method, "SDA_SS") == 0) {
        K_flag = computeGainMatrixSDA_SS();
    } else if (strcmp(method, "ASDA") == 0) {
        K_flag = computeGainMatrixASDA();
    } else if (strcmp(method, "SDA_SCALED") == 0) {
        K_flag = computeGainMatrixSDA_Scaled();
    } else if (strcmp(method, "SCHUR") == 0) {
        K_flag = computeGainMatrixSchur();
    } else if (strcmp(method, "VAN_DOOREN") == 0) {
        K_flag = computeGainMatrixVanDooren();
    } else if (strcmp(method, "ITERATIVE") == 0) {
        K_flag = computeGainMatrixIterative();
    } else if (strcmp(method, "ADDA") == 0) {
        K_flag = computeGainMatrixADDA();
    } else {
        // Método padrão: SDA
        Serial.print(F("Método desconhecido: "));
        Serial.print(method);
        Serial.println(F(". Usando SDA."));
        K_flag = computeGainMatrixSDA();
    }
    
    if (!K_flag)
        return false;

    bool Kr_flag = computeGainMatrixKr();
    return Kr_flag;
}

void AutoLQR::updateState(const float* currentState)
{
    if (!currentState || !state)
        return;
    matrixCopy(currentState, state, stateSize);
}

void AutoLQR::updateReference(const float* newReference)
{
    if (!newReference || !reference)
        return;
    matrixCopy(newReference, reference, controlSize);
}

void AutoLQR::calculateControl(float* controlOutput)
{
    if (!controlOutput || !K || !state || !Kr || !reference)
        return;

    // Initialize control outputs to zero
    matrixClear(controlOutput, controlSize);

    // u = -K·x + Kr·r
    for (int i = 0; i < controlSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            controlOutput[i] -= K[i * stateSize + j] * state[j];
            if(j < controlSize) {
                controlOutput[i] += Kr[i * controlSize + j] * reference[j];
            }
        }
    }
}

bool AutoLQR::isSystemControllable()
{
    // Basic controllability check for 2x2 systems
    if (stateSize == 2 && controlSize == 1) {
        float det = B[0] * A[1] - B[1] * A[0];
        return fabs(det) > 1e-6;
    }

    // For larger systems, implement a more sophisticated controllability check
    // or return true and let the DARE solver determine feasibility
    return true;
}

const float* AutoLQR::getRicattiSolution() const
{
    return P;
}

bool AutoLQR::computeGainMatrixSDA_Fixed()
{
    // Caminho rápido do SDA em fixed-point Q13.18 (ESP32-S2 sem FPU).
    // Retorna false → chamador faz fallback p/ o SDA float quando há
    // overflow/saturação (g_q_ovf) ou matriz singular no domínio fixed-point.
    const int n = stateSize, m = controlSize;

    if (!A || !B || !Q || !R || !K)
        return false;
    if (n != 6 || m != 3)          // buffers e validação dimensionados p/ o caso 6x3
        return false;
    if (!isSystemControllable())
        return false;

    q16_t Aq[36], Bq[18], Qq[36], Rq[9], Kq[18], Pq[36];   // stack (one-shot por chamada)

    g_q_ovf = false;
    for (int i = 0; i < n * n; i++) { Aq[i] = f2q(A[i], Q_SHIFT); Qq[i] = f2q(Q[i], Q_SHIFT); }
    for (int i = 0; i < n * m; i++)  Bq[i] = f2q(B[i], Q_SHIFT);
    for (int i = 0; i < m * m; i++)  Rq[i] = f2q(R[i], Q_SHIFT);
    if (g_q_ovf) return false;     // entrada já fora do range ±8192 → fallback

    if (!sda_q(Aq, Bq, Qq, Rq, Kq, Pq, n, m, Q_SHIFT))
        return false;              // singular no domínio fixed-point → fallback
    if (g_q_ovf) return false;     // saturou durante o SDA → fallback

    for (int i = 0; i < m * n; i++) K[i] = q2f(Kq[i], Q_SHIFT);
    if (P) for (int i = 0; i < n * n; i++) P[i] = q2f(Pq[i], Q_SHIFT);

    lastIterations = g_q_iters;
    lastResidual   = 0.0f;         // direto (limitado por quantização, não iterativo)
    return true;
}

// MATRIX_FAST_ATTR = IRAM_ATTR no ESP32, vazio no host (ver MatrixOperations.h).
// O loop do SDA rodava da flash via cache de instrucoes de 16 KB, competindo com
// o resto do firmware; os kernels de matriz que ele chama ja estavam em IRAM.
MATRIX_FAST_ATTR bool AutoLQR::computeGainMatrixSDA()
{
    // Implementação do Structure-preserving Doubling Algorithm (SDA)
    // Para DARE: A'·P·A - P - A'·P·B·(R + B'·P·B)^(-1)·B'·P·A + Q = 0

    if (!A || !B || !Q || !R || !K || !P || !sdaScratch)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    // Scratch pré-alocado no construtor (ver SDA_SCRATCH_FLOATS). Nenhuma
    // alocação dinâmica no caminho quente.
    const int nn = stateSize * stateSize;
    const int mm = controlSize * controlSize;
    const int mn = controlSize * stateSize;

    float* p = sdaScratch;
    float* Ak      = p; p += nn;
    float* Gk      = p; p += nn;
    float* Hk      = p; p += nn;
    float* Ak_next = p; p += nn;
    float* Gk_next = p; p += nn;
    float* Hk_next = p; p += nn;
    float* AT      = p; p += nn;
    float* W       = p; p += nn;
    float* Temp1   = p; p += nn;
    float* Temp2   = p; p += nn;
    float* Temp3   = p; p += nn;
    float* R_inv   = p; p += mm;
    float* BT      = p; p += mn;
    float* B_Rinv  = p; p += mn;

    // ========================================================================
    // INICIALIZAÇÃO CORRETA DO SDA PARA DARE
    // ========================================================================

    // 1. Ak = A (correto)
    matrixCopy(A, Ak, nn);

    // 2. Calcular transpostas
    transposeMatrix(A, AT, stateSize, stateSize);
    transposeMatrix(B, BT, stateSize, controlSize);

    // 3. Calcular R_inv
    matrixCopy(R, R_inv, mm);
    if (!invertMatrix(R_inv, R_inv, controlSize)) {
        return false;
    }

    // 4. Gk = B * R^(-1) * B' (correto)
    matrixMultiply(B, R_inv, B_Rinv, stateSize, controlSize, controlSize);
    matrixMultiply(B_Rinv, BT, Gk, stateSize, controlSize, stateSize);

    // 5. CORREÇÃO CRÍTICA: Hk = Q (não A'·Q·A + Q)
    matrixCopy(Q, Hk, nn);

    // ========================================================================
    // ESCALONAMENTO INICIAL (removido para DARE pois altera a solução)
    // ========================================================================
    // O escalonamento beta é aplicável em CARE (Equação de Riccati Contínua),
    // mas em DARE (Discreta) ele modifica a resposta final do horizonte infinito.
    // Portanto, nenhuma alteração em Gk (ou Hk) deve ser feita.

    // ========================================================================
    // LOOP SDA
    // ========================================================================
    // Cada iteracao do SDA dobra o horizonte coberto (A_k -> A_k(I+G_kH_k)^-1 A_k).
    // Com dt = 5,2 ms sao ~10 iteracoes so para alcancar o regime permanente:
    // nessa fase H_k ainda cresce e o residuo relativo fica perto de 1, e a queda
    // quadratica so aparece no fim. Nao tente cortar cedo por "estagnacao".
    const int maxIterations = SDA_MAX_ITERATIONS;
    const float tolerance = 1e-6f;
    const float tolerance2 = tolerance * tolerance;
    bool converged = false;

    // Trabalhamos com normas AO QUADRADO no teste de parada: rel < tol  <=>
    // diff2 < tol^2 * norm2. Evita 2 sqrtf por iteracao — a libgcc do
    // xtensa-esp32s3 emula div e sqrt em software (a FPU do LX7 so tem
    // add/sub/mul/madd), entao cada uma custa ~70 ciclos.
    float last_rel2 = 0.0f;

    // Inicializar histórico de resíduos
    residualHistoryCount = 0;
    for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

    int iter = 0;
    for (; iter < maxIterations; iter++) {
        // W = (I + Gk·Hk)^(-1)
        matrixMultiply(Gk, Hk, Temp1, stateSize, stateSize, stateSize);

        for (int i = 0; i < stateSize; i++) {
            Temp1[i * stateSize + i] += 1.0f;
        }

        matrixCopy(Temp1, W, nn);
        if (!invertMatrix(W, W, stateSize)) {
            break;
        }

        // Temp1 = Ak·W
        matrixMultiply(Ak, W, Temp1, stateSize, stateSize, stateSize);

        // Ak_next = (Ak·W)·Ak
        matrixMultiply(Temp1, Ak, Ak_next, stateSize, stateSize, stateSize);

        // Gk_next = Gk + (Ak·W)·Gk·Ak'  (output do produto duplo é simétrico)
        transposeMatrix(Ak, AT, stateSize, stateSize);
        matrixMultiply(Gk, AT, Temp2, stateSize, stateSize, stateSize);
        matrixMultiplySymOutput(Temp1, Temp2, Temp3, stateSize);
        matrixAdd(Gk, Temp3, Gk_next, stateSize, stateSize);

        // Hk_next = Hk + Ak'·Hk·W·Ak  (output do produto duplo é simétrico)
        matrixMultiply(W, Ak, Temp2, stateSize, stateSize, stateSize);
        matrixMultiply(Hk, Temp2, Temp3, stateSize, stateSize, stateSize);
        matrixMultiplySymOutput(AT, Temp3, Temp2, stateSize);
        matrixAdd(Hk, Temp2, Hk_next, stateSize, stateSize);

        // Convergência: norma de Frobenius relativa, ao quadrado (sem sqrt)
        float diff2 = 0.0f;
        float norm2 = 0.0f;
        for (int i = 0; i < nn; i++) {
            const float d = Hk_next[i] - Hk[i];
            diff2 += d * d;
            norm2 += Hk[i] * Hk[i];
        }

        // rel^2 = diff2 / norm2 (1 divisao por iteracao, contra 2 sqrt + 1 div antes)
        const float rel2 = (norm2 > 1e-20f) ? (diff2 / norm2) : diff2;
        last_rel2 = rel2;

        // Armazenar resíduo no histórico (primeiras 10 iterações).
        // Aqui o sqrtf vale a pena: o historico e' diagnostico e so roda 10x.
        if (iter < 10) {
            residualHistory[iter] = sqrtf(rel2);
            residualHistoryCount = iter + 1;
        }

        // Atualizar
        matrixCopy(Ak_next, Ak, nn);
        matrixCopy(Gk_next, Gk, nn);
        matrixCopy(Hk_next, Hk, nn);

        if (rel2 < tolerance2) {
            converged = true;
            break;
        }
    }

    lastIterations = (iter < maxIterations) ? (iter + 1) : maxIterations;
    lastResidual   = sqrtf(last_rel2);

    // P = Hk (solução final)
    matrixCopy(Hk, P, nn);

    // ========================================================================
    // CÁLCULO DO GANHO K
    // ========================================================================
    // K = (R + B'·P·B)^(-1) · B'·P·A

    float* BT_P        = p; p += mn;
    float* BT_P_B      = p; p += mm;
    float* BT_P_A      = p; p += mn;
    float* R_plus_BTPB = p;

    // BT_P = B'·P
    matrixMultiply(BT, P, BT_P, controlSize, stateSize, stateSize);

    // BT_P_B = (B'·P)·B
    matrixMultiply(BT_P, B, BT_P_B, controlSize, stateSize, controlSize);

    // R_plus_BTPB = R + B'·P·B
    matrixAdd(R, BT_P_B, R_plus_BTPB, controlSize, controlSize);

    // Inverter
    if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, controlSize)) {
        converged = false;
    } else {
        // BT_P_A = (B'·P)·A
        matrixMultiply(BT_P, A, BT_P_A, controlSize, stateSize, stateSize);

        // K = (R + B'·P·B)^(-1) · (B'·P·A)
        matrixMultiply(R_plus_BTPB, BT_P_A, K, controlSize, controlSize, stateSize);
    }

    return converged;
}

bool AutoLQR::computeGainMatrixSchur()
{
    // Método de Schur para DARE usando formulação do pencil simplético
    // Resolve: A'PA - P - A'PB(R+B'PB)^{-1}B'PA + Q = 0
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    
    // ========================================================================
    // PASSO 1: Alocar memória
    // ========================================================================
    float* R_inv = new float[m * m];
    float* BT = new float[m * n];
    float* AT = new float[n * n];
    float* A_inv = new float[n * n];
    float* G = new float[n * n];
    float* temp_nm = new float[n * m];
    float* temp_nn = new float[n * n];
    float* temp_nn2 = new float[n * n];
    
    // ========================================================================
    // PASSO 2: Calcular matrizes auxiliares
    // ========================================================================
    // R_inv = inv(R)
    matrixCopy(R, R_inv, m * m);
    if (!invertMatrix(R_inv, R_inv, m)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        return false;
    }
    
    // BT = B'
    transposeMatrix(B, BT, n, m);
    
    // AT = A'
    transposeMatrix(A, AT, n, n);
    
    // G = B * R^{-1} * B'
    matrixMultiply(B, R_inv, temp_nm, n, m, m);
    matrixMultiply(temp_nm, BT, G, n, m, n);
    
    // A_inv = inv(A)
    matrixCopy(A, A_inv, n * n);
    if (!invertMatrix(A_inv, A_inv, n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        return false;
    }
    
    // ========================================================================
    // PASSO 3: Construir matriz Hamiltoniana simplética Z (2n x 2n)
    // ========================================================================
    // Para DARE, usamos a matriz simplética:
    // Z = [A + G*A'^{-1}*Q,  -G*A'^{-1}    ]
    //     [-A'^{-1}*Q,        A'^{-1}      ]
    // Os autovalores estáveis de Z dão a solução
    
    Eigen::MatrixXf Z(2*n, 2*n);
    
    // Calcular A'^{-1}
    float* AT_inv = new float[n * n];
    matrixCopy(AT, AT_inv, n * n);
    if (!invertMatrix(AT_inv, AT_inv, n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    // Bloco (0,0): A + G*A'^{-1}*Q
    // temp_nn = A'^{-1} * Q
    matrixMultiply(AT_inv, Q, temp_nn, n, n, n);
    // temp_nn2 = G * (A'^{-1} * Q)
    matrixMultiply(G, temp_nn, temp_nn2, n, n, n);
    // Z(0:n, 0:n) = A + G*A'^{-1}*Q
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(i, j) = A[i * n + j] + temp_nn2[i * n + j];
        }
    }
    
    // Bloco (0,n): -G*A'^{-1}
    matrixMultiply(G, AT_inv, temp_nn, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(i, n + j) = -temp_nn[i * n + j];
        }
    }
    
    // Bloco (n,0): -A'^{-1}*Q
    matrixMultiply(AT_inv, Q, temp_nn, n, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(n + i, j) = -temp_nn[i * n + j];
        }
    }
    
    // Bloco (n,n): A'^{-1}
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Z(n + i, n + j) = AT_inv[i * n + j];
        }
    }
    
    // ========================================================================
    // PASSO 4: Decomposição de Schur complexa
    // ========================================================================
    Eigen::ComplexSchur<Eigen::MatrixXf> schur(Z);
    
    if (schur.info() != Eigen::Success) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    Eigen::MatrixXcf T = schur.matrixT();
    Eigen::MatrixXcf U = schur.matrixU();
    
    // ========================================================================
    // PASSO 5: Identificar autovalores estáveis (|λ| < 1)
    // ========================================================================
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    for (int i = 0; i < 2*n; i++) {
        std::complex<float> eigenvalue = T(i, i);
        float magnitude = std::abs(eigenvalue);
        
        if (magnitude < 1.0f && magnitude > 1e-10f) {
            stable_indices.push_back(i);
        }
    }
    
    // Se não encontrou n autovalores estáveis, tentar com threshold mais relaxado
    if (stable_indices.size() < static_cast<size_t>(n)) {
        stable_indices.clear();
        for (int i = 0; i < 2*n; i++) {
            std::complex<float> eigenvalue = T(i, i);
            float magnitude = std::abs(eigenvalue);
            
            if (magnitude < 1.0f + 1e-6f) {
                stable_indices.push_back(i);
                if (stable_indices.size() == static_cast<size_t>(n)) break;
            }
        }
    }
    
    if (stable_indices.size() != static_cast<size_t>(n)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        return false;
    }
    
    // ========================================================================
    // PASSO 6: Extrair subespaço invariante estável
    // ========================================================================
    Eigen::MatrixXcf U11(n, n);
    Eigen::MatrixXcf U21(n, n);
    
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        for (int i = 0; i < n; i++) {
            U11(i, j) = U(i, idx);
            U21(i, j) = U(n + i, idx);
        }
    }
    
    // ========================================================================
    // PASSO 7: Calcular P = U21 * inv(U11)
    // ========================================================================
    Eigen::MatrixXcf P_complex = U21 * U11.inverse();
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i * n + j] = P_complex(i, j).real();
        }
    }
    
    // Forçar simetria de P
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }
    
    // ========================================================================
    // PASSO 8: Calcular K = (R + B'*P*B)^{-1} * B'*P*A
    // ========================================================================
    float* BT_P = new float[m * n];
    float* BT_P_B = new float[m * m];
    float* BT_P_A = new float[m * n];
    float* term = new float[m * m];
    
    matrixMultiply(BT, P, BT_P, m, n, n);
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    matrixAdd(R, BT_P_B, term, m, m);
    
    if (!invertMatrix(term, term, m)) {
        delete[] R_inv; delete[] BT; delete[] AT; delete[] A_inv;
        delete[] G; delete[] temp_nm; delete[] temp_nn; delete[] temp_nn2;
        delete[] AT_inv;
        delete[] BT_P; delete[] BT_P_B; delete[] BT_P_A; delete[] term;
        return false;
    }
    
    matrixMultiply(BT_P, A, BT_P_A, m, n, n);
    matrixMultiply(term, BT_P_A, K, m, m, n);

    // ========================================================================
    // Limpeza de memória
    // ========================================================================
    delete[] R_inv;
    delete[] BT;
    delete[] AT;
    delete[] A_inv;
    delete[] G;
    delete[] temp_nm;
    delete[] temp_nn;
    delete[] temp_nn2;
    delete[] AT_inv;
    delete[] BT_P;
    delete[] BT_P_B;
    delete[] BT_P_A;
    delete[] term;
    
    return true;
}

bool AutoLQR::computeGainMatrixVanDooren()
{
    // ========================================================================
    // Método de Van Dooren para DARE
    // Baseado em: P. van Dooren, "A Generalized Eigenvalue Approach For Solving
    // Riccati Equations", SIAM J. Sci. Stat. Comput., Vol.2(2), 1981.
    //
    // Usa o pencil simplétictico estendido (2n+m)×(2n+m) com deflação QR
    // para obter um pencil (2n)×(2n) antes da decomposição QZ.
    // ========================================================================
    
    // Van Dooren é método direto - inicializar histórico de resíduos com zeros
    residualHistoryCount = 1;
    for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int pencil_size = 2*n + m;

    // ========================================================================
    // Pré-alocar memória
    // ========================================================================
    float* BT = new float[m * n];
    float* AT = new float[n * n];
    float* BT_P = new float[m * n];
    float* BT_P_B = new float[m * m];
    float* term = new float[m * m];
    float* BT_P_A = new float[m * n];

    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);

    // ========================================================================
    // PASSO 1: Construir pencil estendido H - λJ  [(2n+m) × (2n+m)]
    // ========================================================================
    // H = [ A    0    B ]       J = [ I   0   0 ]
    //     [-Q    I    0 ]           [ 0  A^T  0 ]
    //     [ 0    0    R ]           [ 0 -B^T  0 ]
    //
    // Nota: Esta formulação é equivalente mas mais estável numericamente
    // ========================================================================
    
    Eigen::MatrixXf H(pencil_size, pencil_size);
    Eigen::MatrixXf J(pencil_size, pencil_size);
    
    H.setZero();
    J.setZero();

    // H[:n, :n] = A
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(i, j) = A[i * n + j];
        }
    }
    
    // H[:n, 2n:] = B
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            H(i, 2*n + j) = B[i * m + j];
        }
    }
    
    // H[n:2n, :n] = -Q
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            H(n + i, j) = -Q[i * n + j];
        }
    }
    
    // H[n:2n, n:2n] = I
    for (int i = 0; i < n; i++) {
        H(n + i, n + i) = 1.0f;
    }
    
    // H[2n:, 2n:] = R
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            H(2*n + i, 2*n + j) = R[i * m + j];
        }
    }
    
    // J[:n, :n] = I
    for (int i = 0; i < n; i++) {
        J(i, i) = 1.0f;
    }
    
    // J[n:2n, n:2n] = A^T
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            J(n + i, n + j) = AT[i * n + j];
        }
    }
    
    // J[2n:, n:2n] = -B^T
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            J(2*n + i, n + j) = -BT[i * n + j];
        }
    }

    // ========================================================================
    // PASSO 2: Deflação QR para reduzir de (2n+m)×(2n+m) para (2n)×(2n)
    // ========================================================================
    // Extrai as últimas m colunas de H e faz QR
    // Depois projeta no espaço ortogonal
    
    Eigen::MatrixXf H_last_cols = H.rightCols(m);
    
    // Decomposição QR: H_last_cols = Q_qr * R_qr
    Eigen::HouseholderQR<Eigen::MatrixXf> qr(H_last_cols);
    Eigen::MatrixXf Q_qr = qr.householderQ();
    
    // Q_deflate = últimas (2n) colunas de Q_qr (descarta as primeiras m)
    Eigen::MatrixXf Q_deflate = Q_qr.rightCols(2*n);
    
    // Aplica deflação: projeta H e J nas primeiras 2n colunas
    Eigen::MatrixXf H_deflated = Q_deflate.transpose() * H.leftCols(2*n);
    Eigen::MatrixXf J_deflated = Q_deflate.transpose() * J.leftCols(2*n);

    // ========================================================================
    // PASSO 3: Decomposição QZ Generalizada no pencil deflacionado (2n × 2n)
    // ========================================================================
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXf> ges;
    ges.compute(H_deflated, J_deflated, true);
    
    if (ges.info() != Eigen::Success) {
        delete[] BT; delete[] AT;
        delete[] BT_P; delete[] BT_P_B; delete[] term; delete[] BT_P_A;
        return false;
    }
    
    Eigen::VectorXcf alpha = ges.alphas();
    Eigen::VectorXcf beta = ges.betas();
    const Eigen::MatrixXcf& Z = ges.eigenvectors();

    // ========================================================================
    // PASSO 4: Seleção de autovalores estáveis (|λ| < 1 para DARE)
    // ========================================================================
    // Armazena pares (magnitude, índice) para ordenar
    std::vector<std::pair<float, int>> eig_pairs;
    eig_pairs.reserve(2*n);
    
    const float beta_min = 1e-10f;
    
    for (int i = 0; i < 2*n; i++) {
        float beta_abs = std::abs(beta(i));
        float magnitude;
        
        if (beta_abs > beta_min) {
            std::complex<float> eigenvalue = alpha(i) / beta(i);
            magnitude = std::abs(eigenvalue);
        } else {
            // Autovalor no infinito - não é estável
            magnitude = 1e10f;
        }
        
        eig_pairs.push_back({magnitude, i});
    }
    
    // Ordena por magnitude (menor primeiro)
    std::sort(eig_pairs.begin(), eig_pairs.end());
    
    // Seleciona os n menores (dentro do círculo unitário)
    std::vector<int> stable_indices;
    stable_indices.reserve(n);
    
    for (int i = 0; i < n && i < static_cast<int>(eig_pairs.size()); i++) {
        if (eig_pairs[i].first < 1.0f) {
            stable_indices.push_back(eig_pairs[i].second);
        }
    }
    
    // Se não encontrou n autovalores estáveis, relaxa threshold
    if (stable_indices.size() < static_cast<size_t>(n)) {
        stable_indices.clear();
        for (int i = 0; i < n && i < static_cast<int>(eig_pairs.size()); i++) {
            if (eig_pairs[i].first < 1.1f) {
                stable_indices.push_back(eig_pairs[i].second);
            }
        }
    }
    
    if (stable_indices.size() != static_cast<size_t>(n)) {
        delete[] BT; delete[] AT;
        delete[] BT_P; delete[] BT_P_B; delete[] term; delete[] BT_P_A;
        return false;
    }

    // ========================================================================
    // PASSO 5: Extrair subespaço invariante estável
    // ========================================================================
    // Z está particionado como [U1; U2] onde cada bloco é n × n
    
    Eigen::MatrixXcf U1(n, n);
    Eigen::MatrixXcf U2(n, n);
    
    for (int j = 0; j < n; j++) {
        int idx = stable_indices[j];
        U1.col(j) = Z.col(idx).head(n);
        U2.col(j) = Z.col(idx).segment(n, n);
    }

    // ========================================================================
    // PASSO 6: Calcular P = U2 * inv(U1)
    // ========================================================================
    // Usa decomposição LU para maior estabilidade
    Eigen::FullPivLU<Eigen::MatrixXcf> lu(U1);
    
    if (!lu.isInvertible()) {
        delete[] BT; delete[] AT;
        delete[] BT_P; delete[] BT_P_B; delete[] term; delete[] BT_P_A;
        return false;
    }
    
    Eigen::MatrixXcf P_complex = U2 * lu.inverse();
    
    // Extrai parte real e copia para P
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            P[i * n + j] = P_complex(i, j).real();
        }
    }
    
    // Forçar simetria: P = (P + P') / 2
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }

    // ========================================================================
    // PASSO 7: Calcular K = (R + B'PB)^(-1) * B'PA
    // ========================================================================
    matrixMultiply(BT, P, BT_P, m, n, n);
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    matrixAdd(R, BT_P_B, term, m, m);
    
    if (!invertMatrix(term, term, m)) {
        delete[] BT; delete[] AT;
        delete[] BT_P; delete[] BT_P_B; delete[] term; delete[] BT_P_A;
        return false;
    }
    
    matrixMultiply(BT_P, A, BT_P_A, m, n, n);
    matrixMultiply(term, BT_P_A, K, m, m, n);

    // ========================================================================
    // Van Dooren é método direto (não iterativo)
    // ========================================================================
    lastIterations = 1;  // Método direto = 1 "iteração"
    lastResidual = 0.0f; // Solução direta, sem resíduo iterativo

    // ========================================================================
    // Limpeza
    // ========================================================================
    delete[] BT; delete[] AT;
    delete[] BT_P; delete[] BT_P_B; delete[] term; delete[] BT_P_A;
    
    return true;
}

bool AutoLQR::computeGainMatrixKr()
{
    // Primeiro, certifique-se de que os ganhos K foram calculados
    if (!K) {
        return false;
    }

    for (int i = 0; i < controlSize; ++i) { 
        for (int j = 0; j < controlSize; ++j) { 
            if (j < stateSize) {
                Kr[i * controlSize + j] = -K[i * stateSize + j];
            } else {
                Kr[i * controlSize + j] = 0.0f;
            }
        }
    }
    return true; 
}

bool AutoLQR::exportKr(float* exportedKr)
{
    if (!Kr || !exportedKr)
        return false;
    
    matrixCopy(Kr, exportedKr, controlSize * controlSize);
    return true;
}

void AutoLQR::estimateFeedforwardGain(float* ffGain, const float* desiredState)
{
    if (!ffGain || !desiredState || !A || !B || !K)
        return;

    // For steady-state tracking: u_ff = -(inv(B'B) * B' * (A-I)) * x_desired
    // This is simplified for common cases

    if (stateSize == 2 && controlSize == 1) {
        // Special case for position-velocity systems
        float Bsq = B[0] * B[0] + B[1] * B[1];
        if (Bsq < 1e-6)
            return;

        float invBsq = 1.0f / Bsq;

        // Compute (A-I) * x_desired
        float dx[2];
        dx[0] = (A[0] - 1.0f) * desiredState[0] + A[1] * desiredState[1];
        dx[1] = A[2] * desiredState[0] + (A[3] - 1.0f) * desiredState[1];

        // Compute feedforward gain
        ffGain[0] = -invBsq * (B[0] * dx[0] + B[1] * dx[1]);
    } else {
        // For other systems, initialize to zero
        matrixClear(ffGain, controlSize);
    }
}

float AutoLQR::estimateConvergenceTime(float convergenceThreshold)
{
    // Estimate convergence time based on eigenvalues
    // This is a simplified estimate for 2x2 systems

    if (stateSize == 2) {
        // Compute closed-loop dynamics: A - B*K
        float CL[4];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                CL[i * 2 + j] = A[i * 2 + j];
                for (int k = 0; k < controlSize; k++) {
                    CL[i * 2 + j] -= B[i * controlSize + k] * K[k * stateSize + j];
                }
            }
        }

        // Approximate dominant eigenvalue using trace and determinant
        float trace = CL[0] + CL[3];
        float det = CL[0] * CL[3] - CL[1] * CL[2];

        // Characteristic equation: λ² - trace·λ + det = 0
        float discriminant = trace * trace - 4 * det;

        if (discriminant >= 0) {
            // Real eigenvalues
            float lambda1 = (trace + sqrt(discriminant)) / 2;
            float lambda2 = (trace - sqrt(discriminant)) / 2;

            // Dominant eigenvalue (larger magnitude)
            float domEigenvalue = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;

            if (fabs(domEigenvalue) < 1.0f && fabs(domEigenvalue) > 0.0f) {
                // Estimate time constant
                float timeConstant = -1.0f / log(fabs(domEigenvalue));

                // Time to reach convergenceThreshold
                return timeConstant * log(1.0f / convergenceThreshold);
            }
        }
    }

    // Default value if calculation fails
    return -1.0f;
}

bool AutoLQR::exportGains(float* exportedK)
{
    if (!K || !exportedK)
        return false;

    matrixCopy(K, exportedK, controlSize * stateSize);
    return true;
}

float AutoLQR::calculateExpectedCost()
{
    if (!P || !state)
        return -1.0f;

    // Cost = x'Px
    float cost = 0;
    for (int i = 0; i < stateSize; i++) {
        for (int j = 0; j < stateSize; j++) {
            cost += state[i] * P[i * stateSize + j] * state[j];
        }
    }

    return cost;
}

bool AutoLQR::computeGainMatrixIterative()
{
    // Método Iterativo de Riccati com Warm Start para DARE
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    // Verificar warm start
    bool has_warm_start = false;
    float P_norm = 0.0f;
    for (int i = 0; i < stateSize * stateSize; i++) {
        P_norm += fabsf(P[i]);
    }
    has_warm_start = (P_norm > 1e-6f);

    // Se não tem warm start, inicializa P = Q
    if (!has_warm_start) {
        matrixCopy(Q, P, stateSize * stateSize);
    }

    // Pré-alocação de memória
    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int nm = n * m;
    const int mm = m * m;
    
    float* P_new = new float[nn]();
    float* AT = new float[nn]();
    float* BT = new float[nm]();
    float* PA = new float[nn]();
    float* PB = new float[nm]();
    float* ATPA = new float[nn]();
    float* BTPB = new float[mm]();
    float* BTPA = new float[m * n]();
    float* S = new float[mm]();
    float* S_inv = new float[mm]();
    float* K_temp = new float[m * n]();
    float* ATPB = new float[nm]();
    float* correction = new float[nn]();

    // Calcular transpostas (constantes)
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);

    const int maxIterations = 100;
    const float tolerance = 1e-6f;
    bool converged = false;
    
    // Inicializar histórico de resíduos
    residualHistoryCount = 0;
    for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

    for (int iter = 0; iter < maxIterations; iter++) {
        // ================================================================
        // ITERAÇÃO DARE
        // P_new = Q + A'PA - A'PB(R + B'PB)^{-1}B'PA
        // ================================================================

        matrixMultiply(P, A, PA, n, n, n);
        matrixMultiply(P, B, PB, n, n, m);
        matrixMultiply(AT, PA, ATPA, n, n, n);
        matrixMultiply(BT, PB, BTPB, m, n, m);
        matrixMultiply(BT, PA, BTPA, m, n, n);
        
        matrixAdd(R, BTPB, S, m, m);
        
        // Regularização
        for (int i = 0; i < m; i++) {
            S[i * m + i] += 1e-8f;
        }
        
        matrixCopy(S, S_inv, mm);
        if (!invertMatrix(S_inv, S_inv, m)) {
            break;
        }
        
        matrixMultiply(S_inv, BTPA, K_temp, m, m, n);
        matrixMultiply(AT, PB, ATPB, n, n, m);
        matrixMultiply(ATPB, K_temp, correction, n, m, n);
        
        float diff = 0.0f;
        float P_norm = 0.0f;
        
        for (int i = 0; i < nn; i++) {
            P_new[i] = Q[i] + ATPA[i] - correction[i];
            float d = P_new[i] - P[i];
            diff += d * d;
            P_norm += P[i] * P[i];
        }
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P_new[i * n + j] + P_new[j * n + i]) * 0.5f;
                P_new[i * n + j] = avg;
                P_new[j * n + i] = avg;
            }
        }
        
        matrixCopy(P_new, P, nn);

        diff = sqrtf(diff);
        P_norm = sqrtf(P_norm);
        float rel_diff = (P_norm > 1e-10f) ? (diff / P_norm) : diff;
        
        // Armazenar resíduo no histórico (primeiras 10 iterações)
        if (iter < 10) {
            residualHistory[iter] = rel_diff;
            residualHistoryCount = iter + 1;
        }
        
        if (rel_diff < tolerance) {
            converged = true;
            lastIterations = iter + 1;
            lastResidual = rel_diff;
            break;
        }
    }
    
    if (!converged) {
        lastIterations = maxIterations;
        // Calculate final residual
        float diff = 0.0f;
        float P_norm = 0.0f;
        for (int i = 0; i < nn; i++) {
            float d = P_new[i] - P[i];
            diff += d * d;
            P_norm += P[i] * P[i];
        }
        diff = sqrtf(diff);
        P_norm = sqrtf(P_norm);
        lastResidual = (P_norm > 1e-10f) ? (diff / P_norm) : diff;
    }

    // ================================================================
    // CÁLCULO FINAL DO GANHO K
    // ================================================================
    matrixMultiply(P, A, PA, n, n, n);
    matrixMultiply(P, B, PB, n, n, m);
    matrixMultiply(BT, PB, BTPB, m, n, m);
    matrixMultiply(BT, PA, BTPA, m, n, n);
    
    matrixAdd(R, BTPB, S, m, m);
    for (int i = 0; i < m; i++) {
        S[i * m + i] += 1e-8f;
    }
    
    matrixCopy(S, S_inv, mm);
    if (invertMatrix(S_inv, S_inv, m)) {
        matrixMultiply(S_inv, BTPA, K, m, m, n);
    } else {
        converged = false;
    }

    // Limpeza
    delete[] P_new;
    delete[] AT;
    delete[] BT;
    delete[] PA;
    delete[] PB;
    delete[] ATPA;
    delete[] BTPB;
    delete[] BTPA;
    delete[] S;
    delete[] S_inv;
    delete[] K_temp;
    delete[] ATPB;
    delete[] correction;

    return converged;
}

// ============================================================================
// SDA COM SINGLE SHIFT (SDA-ss)
// Versão melhorada do SDA com parâmetro de shift para convergência aprimorada
// quando autovalores estão próximos de 1
// ============================================================================
bool AutoLQR::computeGainMatrixSDA_SS()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-6f;
    
    // ========================================================================
    // SDA com Single Shift: Transforma o problema para acelerar convergência
    // O shift γ transforma autovalores λ → (λ-γ)/(1-γλ)
    // Escolha ótima: γ próximo ao maior autovalor instável do Hamiltoniano
    // Para DARE típico, γ = 0.5 a 0.9 funciona bem
    // ========================================================================
    float gamma = 0.3f;  // Shift real (não 1.0!)
    bool converged = false;
    float prev_diff = 1e10f;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    float* I_gamma = new float[nn]();  // Matriz para shift
    
    // Inicialização com shift aplicado ao sistema
    // A_shifted = (A - γI)/(1 - γ) para transformar o problema
    matrixCopy(A, Ak, nn);
    
    // Aplicar shift na matriz A: Ak = A - γ·I (primeira parte da transformação)
    for (int i = 0; i < n; i++) {
        Ak[i * n + i] -= gamma;
    }
    // Normalizar: Ak = Ak / (1 - γ)
    float inv_1_minus_gamma = 1.0f / (1.0f - gamma);
    for (int i = 0; i < nn; i++) {
        Ak[i] *= inv_1_minus_gamma;
    }
    
    transposeMatrix(Ak, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    // Calcular R_inv
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        // Gk = B * R^(-1) * B' / (1-γ)²
        float* B_Rinv = new float[n * m];
        matrixMultiply(B, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT, Gk, n, m, n);
        delete[] B_Rinv;
        
        // Escalar Gk pelo fator do shift
        float scale_G = inv_1_minus_gamma * inv_1_minus_gamma;
        for (int i = 0; i < nn; i++) {
            Gk[i] *= scale_G;
        }
        
        // Hk = Q (sem alteração)
        matrixCopy(Q, Hk, nn);
        
        // Inicializar histórico de resíduos
        residualHistoryCount = 0;
        for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

        // Loop SDA com shift (as iterações são as mesmas, mas sobre sistema transformado)
        for (int iter = 0; iter < maxIterations; iter++) {
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_Hk = 0.0f;
            for (int i = 0; i < nn; i++) {
                diff += (Hk_next[i] - Hk[i]) * (Hk_next[i] - Hk[i]);
                norm_Hk += Hk[i] * Hk[i];
            }
            diff = sqrtf(diff);
            norm_Hk = sqrtf(norm_Hk);
            
            float rel_diff = (norm_Hk > 1e-10f) ? (diff / norm_Hk) : diff;
            
            // Armazenar resíduo no histórico (primeiras 10 iterações)
            if (iter < 10) {
                residualHistory[iter] = rel_diff;
                residualHistoryCount = iter + 1;
            }
            
            prev_diff = rel_diff;
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                lastIterations = iter + 1;
                lastResidual = rel_diff;
                break;
            }
        }
        
        if (!converged) {
            lastIterations = maxIterations;
            float diff_final = 0.0f;
            float norm_Hk_final = 0.0f;
            for (int i = 0; i < nn; i++) {
                diff_final += (Hk_next[i] - Hk[i]) * (Hk_next[i] - Hk[i]);
                norm_Hk_final += Hk[i] * Hk[i];
            }
            diff_final = sqrtf(diff_final);
            norm_Hk_final = sqrtf(norm_Hk_final);
            lastResidual = (norm_Hk_final > 1e-10f) ? (diff_final / norm_Hk_final) : diff_final;
        }

        // P = Hk
        matrixCopy(Hk, P, nn);
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K usando A ORIGINAL (não a transformada pelo shift)
        // K = (R + B'·P·B)^(-1) · B'·P·A
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            // Usa A original, não Ak (que foi transformada pelo shift)
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] I_gamma;

    return converged;
}

// ============================================================================
// SDA ADAPTATIVO (ASDA)
// Usa escalonamento adaptativo durante iterações para melhor estabilidade
// ============================================================================
bool AutoLQR::computeGainMatrixASDA()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-6f;
    bool converged = false;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // Fatores de escalonamento adaptativo
    float alpha_k = 1.0f;
    float beta_k = 1.0f;
    
    // Inicialização ASDA
    matrixCopy(A, Ak, nn);
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        float* B_Rinv = new float[n * m];
        matrixMultiply(B, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT, Gk, n, m, n);
        delete[] B_Rinv;
        
        matrixCopy(Q, Hk, nn);
        
        // Cálculo do escalonamento ótimo inicial
        float norm_A = 0.0f, norm_G = 0.0f, norm_H = 0.0f;
        for (int i = 0; i < nn; i++) {
            norm_A += Ak[i] * Ak[i];
            norm_G += Gk[i] * Gk[i];
            norm_H += Hk[i] * Hk[i];
        }
        norm_A = sqrtf(norm_A);
        norm_G = sqrtf(norm_G);
        norm_H = sqrtf(norm_H);
        
        if (norm_A > 1e-10f) {
            alpha_k = 1.0f / norm_A;
        }
        if (norm_H > 1e-10f && norm_G > 1e-10f) {
            beta_k = sqrtf(norm_H / norm_G);
        }
        
        // Aplicar escalonamento inicial
        for (int i = 0; i < nn; i++) {
            Gk[i] *= (beta_k * beta_k);
        }
        
        // Inicializar histórico de resíduos
        residualHistoryCount = 0;
        for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

        // Loop ASDA
        for (int iter = 0; iter < maxIterations; iter++) {
            // Passo 1: Calcular escalonamento adaptativo
            float norm_Gk = 0.0f, norm_Hk = 0.0f;
            for (int i = 0; i < nn; i++) {
                norm_Gk += Gk[i] * Gk[i];
                norm_Hk += Hk[i] * Hk[i];
            }
            norm_Gk = sqrtf(norm_Gk);
            norm_Hk = sqrtf(norm_Hk);
            
            float scale_factor = 1.0f;
            if (norm_Gk > 1e-10f && norm_Hk > 1e-10f) {
                scale_factor = sqrtf(norm_Hk / norm_Gk);
                scale_factor = fminf(fmaxf(scale_factor, 0.1f), 10.0f);
            }
            
            for (int i = 0; i < nn; i++) {
                Gk[i] *= scale_factor;
                Hk[i] /= scale_factor;
            }
            
            // Passo 2: Iteração SDA padrão
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_H_new = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff += d * d;
                norm_H_new += Hk_next[i] * Hk_next[i];
            }
            diff = sqrtf(diff);
            norm_H_new = sqrtf(norm_H_new);
            
            float rel_diff = (norm_H_new > 1e-10f) ? (diff / norm_H_new) : diff;
            
            // Armazenar resíduo no histórico (primeiras 10 iterações)
            if (iter < 10) {
                residualHistory[iter] = rel_diff;
                residualHistoryCount = iter + 1;
            }
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                lastIterations = iter + 1;
                lastResidual = rel_diff;
                break;
            }
        }
        
        if (!converged) {
            lastIterations = maxIterations;
            float diff_final = 0.0f;
            float norm_H_final = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff_final += d * d;
                norm_H_final += Hk_next[i] * Hk_next[i];
            }
            diff_final = sqrtf(diff_final);
            norm_H_final = sqrtf(norm_H_final);
            lastResidual = (norm_H_final > 1e-10f) ? (diff_final / norm_H_final) : diff_final;
        }

        // P = Hk
        matrixCopy(Hk, P, nn);
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;

    return converged;
}

// ============================================================================
// SDA COM ESCALONAMENTO ÓTIMO (Scaled SDA)
// Usa escalonamento ótimo do pencil Hamiltoniano para melhor condicionamento
// ============================================================================
bool AutoLQR::computeGainMatrixSDA_Scaled()
{
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;
    const int maxIterations = 100;
    const float tolerance = 1e-6f;
    bool converged = false;
    
    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* W = new float[nn]();
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // Matrizes de escalonamento
    float* D = new float[n]();
    float* Dinv = new float[n]();
    
    // Calcular escalonamento diagonal baseado nas normas das linhas de A
    for (int i = 0; i < n; i++) {
        float row_norm = 0.0f;
        for (int j = 0; j < n; j++) {
            row_norm += A[i * n + j] * A[i * n + j];
        }
        row_norm = sqrtf(row_norm);
        D[i] = (row_norm > 1e-10f) ? (1.0f / sqrtf(row_norm)) : 1.0f;
        Dinv[i] = 1.0f / D[i];
    }
    
    // Aplicar escalonamento a A: A_scaled = D * A * D^(-1)
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Ak[i * n + j] = D[i] * A[i * n + j] * Dinv[j];
        }
    }
    
    transposeMatrix(Ak, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    // Calcular R_inv
    matrixCopy(R, R_inv, mm);
    bool init_ok = invertMatrix(R_inv, R_inv, m);
    
    if (init_ok) {
        // Calcular G com escalonamento
        float* B_scaled = new float[n * m];
        float* B_Rinv = new float[n * m];
        float* BT_scaled = new float[m * n];
        
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                B_scaled[i * m + j] = D[i] * B[i * m + j];
            }
        }
        
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                BT_scaled[i * n + j] = BT[i * n + j] * D[j];
            }
        }
        
        matrixMultiply(B_scaled, R_inv, B_Rinv, n, m, m);
        matrixMultiply(B_Rinv, BT_scaled, Gk, n, m, n);
        
        delete[] B_scaled;
        delete[] B_Rinv;
        delete[] BT_scaled;
        
        // Q_scaled = D * Q * D
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Hk[i * n + j] = D[i] * Q[i * n + j] * D[j];
            }
        }
        
        // Inicializar histórico de resíduos
        residualHistoryCount = 0;
        for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

        // Loop SDA escalonado
        for (int iter = 0; iter < maxIterations; iter++) {
            matrixMultiply(Gk, Hk, Temp1, n, n, n);
            for (int i = 0; i < n; i++) {
                Temp1[i * n + i] += 1.0f;
            }
            
            matrixCopy(Temp1, W, nn);
            if (!invertMatrix(W, W, n)) {
                break;
            }
            
            matrixMultiply(Ak, W, Temp1, n, n, n);
            matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
            
            transposeMatrix(Ak, AT, n, n);
            matrixMultiply(Gk, AT, Temp2, n, n, n);
            matrixMultiply(Temp1, Temp2, Temp3, n, n, n);
            matrixAdd(Gk, Temp3, Gk_next, n, n);
            
            matrixMultiply(W, Ak, Temp2, n, n, n);
            matrixMultiply(Hk, Temp2, Temp3, n, n, n);
            matrixMultiply(AT, Temp3, Temp2, n, n, n);
            matrixAdd(Hk, Temp2, Hk_next, n, n);
            
            float diff = 0.0f;
            float norm_H = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff += d * d;
                norm_H += Hk_next[i] * Hk_next[i];
            }
            diff = sqrtf(diff);
            norm_H = sqrtf(norm_H);
            
            float rel_diff = (norm_H > 1e-10f) ? (diff / norm_H) : diff;
            
            // Armazenar resíduo no histórico (primeiras 10 iterações)
            if (iter < 10) {
                residualHistory[iter] = rel_diff;
                residualHistoryCount = iter + 1;
            }
            
            matrixCopy(Ak_next, Ak, nn);
            matrixCopy(Gk_next, Gk, nn);
            matrixCopy(Hk_next, Hk, nn);
            
            if (rel_diff < tolerance) {
                converged = true;
                lastIterations = iter + 1;
                lastResidual = rel_diff;
                break;
            }
        }
        
        if (!converged) {
            lastIterations = maxIterations;
            float diff_final = 0.0f;
            float norm_H_final = 0.0f;
            for (int i = 0; i < nn; i++) {
                float d = Hk_next[i] - Hk[i];
                diff_final += d * d;
                norm_H_final += Hk_next[i] * Hk_next[i];
            }
            diff_final = sqrtf(diff_final);
            norm_H_final = sqrtf(norm_H_final);
            lastResidual = (norm_H_final > 1e-10f) ? (diff_final / norm_H_final) : diff_final;
        }

        // Recuperar P original: P = D^(-1) * P_scaled * D^(-1)
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                P[i * n + j] = Dinv[i] * Hk[i * n + j] * Dinv[j];
            }
        }
        
        // Forçar simetria
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
                P[i * n + j] = avg;
                P[j * n + i] = avg;
            }
        }

        // Cálculo do ganho K
        float* BT_P = new float[m * n];
        float* BT_P_B = new float[mm];
        float* BT_P_A = new float[m * n];
        float* R_plus_BTPB = new float[mm];
        
        matrixMultiply(BT, P, BT_P, m, n, n);
        matrixMultiply(BT_P, B, BT_P_B, m, n, m);
        matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
        
        if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
            converged = false;
        } else {
            matrixMultiply(BT_P, A, BT_P_A, m, n, n);
            matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
        }
        
        delete[] BT_P;
        delete[] BT_P_B;
        delete[] BT_P_A;
        delete[] R_plus_BTPB;
    }

    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] D; delete[] Dinv;

    return converged;
}

bool AutoLQR::computeGainMatrixADDA()
{
    // Implementação do Alternating-Directional Doubling Algorithm (ADDA)
    // Baseado em: Lin, Xu - "A structure-preserving doubling algorithm for 
    // nonsymmetric algebraic Riccati equation" (2006)
    // 
    // Diferença do SDA: O ADDA usa DUAS matrizes inversas em cada iteração:
    //   V = (I + Gk·Hk)^(-1)
    //   W = (I + Hk·Gk)^(-1)
    // 
    // Isso explora a simetria do problema para melhor estabilidade numérica.
    // 
    // Iterações ADDA:
    //   Ak+1 = Ak·V·Ak (usando V para atualização de A)
    //   Gk+1 = Gk + Ak·V·Gk·Ak'  (usando V)
    //   Hk+1 = Hk + Ak'·W·Hk·Ak  (usando W - alternância direcional!)
    
    if (!A || !B || !Q || !R || !K || !P)
        return false;

    if (!isSystemControllable()) {
        return false;
    }

    const int n = stateSize;
    const int m = controlSize;
    const int nn = n * n;
    const int mm = m * m;

    // Alocação de memória
    float* Ak = new float[nn]();
    float* Gk = new float[nn]();
    float* Hk = new float[nn]();
    
    float* Ak_next = new float[nn]();
    float* Gk_next = new float[nn]();
    float* Hk_next = new float[nn]();
    
    float* R_inv = new float[mm]();
    float* BT = new float[m * n]();
    float* AT = new float[nn]();
    float* V = new float[nn]();      // (I + Gk·Hk)^(-1)
    float* W = new float[nn]();      // (I + Hk·Gk)^(-1) - ADDA específico
    float* Temp1 = new float[nn]();
    float* Temp2 = new float[nn]();
    float* Temp3 = new float[nn]();
    
    // ========================================================================
    // INICIALIZAÇÃO DO ADDA
    // ========================================================================
    
    // 1. Ak = A
    matrixCopy(A, Ak, nn);
    
    // 2. Calcular transpostas
    transposeMatrix(A, AT, n, n);
    transposeMatrix(B, BT, n, m);
    
    // 3. Calcular R_inv
    matrixCopy(R, R_inv, mm);
    if (!invertMatrix(R_inv, R_inv, m)) {
        delete[] Ak; delete[] Gk; delete[] Hk;
        delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
        delete[] R_inv; delete[] BT; delete[] AT;
        delete[] V; delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
        return false;
    }
    
    // 4. Gk = B * R^(-1) * B'
    float* B_Rinv = new float[n * m];
    matrixMultiply(B, R_inv, B_Rinv, n, m, m);
    matrixMultiply(B_Rinv, BT, Gk, n, m, n);
    delete[] B_Rinv;

    // 5. Hk = Q
    matrixCopy(Q, Hk, nn);

    // ========================================================================
    // LOOP ADDA - Iterações de dobramento com alternância direcional
    // ========================================================================
    const int maxIterations = 100;
    const float tolerance = 1e-6f;
    bool converged = false;
    
    // Inicializar histórico de resíduos
    residualHistoryCount = 0;
    for (int i = 0; i < 10; i++) residualHistory[i] = 0.0f;

    for (int iter = 0; iter < maxIterations; iter++) {
        // ================================================================
        // ADDA: Calcular AMBAS as inversas V e W
        // ================================================================
        
        // V = (I + Gk·Hk)^(-1)
        matrixMultiply(Gk, Hk, Temp1, n, n, n);
        for (int i = 0; i < n; i++) {
            Temp1[i * n + i] += 1.0f;
        }
        matrixCopy(Temp1, V, nn);
        if (!invertMatrix(V, V, n)) {
            break;
        }
        
        // W = (I + Hk·Gk)^(-1) - ADDA específico (alternância)
        matrixMultiply(Hk, Gk, Temp1, n, n, n);
        for (int i = 0; i < n; i++) {
            Temp1[i * n + i] += 1.0f;
        }
        matrixCopy(Temp1, W, nn);
        if (!invertMatrix(W, W, n)) {
            break;
        }
        
        // ================================================================
        // Iterações ADDA com alternância direcional
        // ================================================================
        
        // Ak_next = Ak·V·Ak
        matrixMultiply(Ak, V, Temp1, n, n, n);
        matrixMultiply(Temp1, Ak, Ak_next, n, n, n);
        
        // Gk_next = Gk + Ak·V·Gk·Ak'  (usa V)
        transposeMatrix(Ak, AT, n, n);
        matrixMultiply(Gk, AT, Temp2, n, n, n);
        matrixMultiply(Temp1, Temp2, Temp3, n, n, n);  // Temp1 ainda é Ak·V
        matrixAdd(Gk, Temp3, Gk_next, n, n);
        
        // Hk_next = Hk + Ak'·W·Hk·Ak  (usa W - ALTERNÂNCIA!)
        // Nota: Esta é a diferença chave do ADDA vs SDA
        matrixMultiply(W, Hk, Temp2, n, n, n);
        matrixMultiply(Temp2, Ak, Temp3, n, n, n);
        matrixMultiply(AT, Temp3, Temp2, n, n, n);
        matrixAdd(Hk, Temp2, Hk_next, n, n);
        
        // Verificar convergência usando norma de Frobenius relativa
        float diff = 0.0f;
        float norm_H = 0.0f;
        for (int i = 0; i < nn; i++) {
            float d = Hk_next[i] - Hk[i];
            diff += d * d;
            norm_H += Hk[i] * Hk[i];
        }
        diff = sqrtf(diff);
        norm_H = sqrtf(norm_H);

        float rel_diff = (norm_H > 1e-10f) ? (diff / norm_H) : diff;
        
        // Armazenar resíduo no histórico (primeiras 10 iterações)
        if (iter < 10) {
            residualHistory[iter] = rel_diff;
            residualHistoryCount = iter + 1;
        }
        
        // Atualizar
        matrixCopy(Ak_next, Ak, nn);
        matrixCopy(Gk_next, Gk, nn);
        matrixCopy(Hk_next, Hk, nn);
        
        if (rel_diff < tolerance) {
            converged = true;
            lastIterations = iter + 1;
            lastResidual = rel_diff;
            break;
        }
    }
    
    if (!converged) {
        lastIterations = maxIterations;
        float diff_final = 0.0f;
        float norm_H_final = 0.0f;
        for (int i = 0; i < nn; i++) {
            float d = Hk_next[i] - Hk[i];
            diff_final += d * d;
            norm_H_final += Hk[i] * Hk[i];
        }
        diff_final = sqrtf(diff_final);
        norm_H_final = sqrtf(norm_H_final);
        lastResidual = (norm_H_final > 1e-10f) ? (diff_final / norm_H_final) : diff_final;
    }

    // P = Hk (solução final)
    matrixCopy(Hk, P, nn);
    
    // Forçar simetria de P
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float avg = (P[i * n + j] + P[j * n + i]) * 0.5f;
            P[i * n + j] = avg;
            P[j * n + i] = avg;
        }
    }

    // ========================================================================
    // CÁLCULO DO GANHO K
    // ========================================================================
    // K = (R + B'·P·B)^(-1) · B'·P·A
    
    float* BT_P = new float[m * n];
    float* BT_P_B = new float[mm];
    float* BT_P_A = new float[m * n];
    float* R_plus_BTPB = new float[mm];
    
    // BT_P = B'·P
    matrixMultiply(BT, P, BT_P, m, n, n);
    
    // BT_P_B = (B'·P)·B
    matrixMultiply(BT_P, B, BT_P_B, m, n, m);
    
    // R_plus_BTPB = R + B'·P·B
    matrixAdd(R, BT_P_B, R_plus_BTPB, m, m);
    
    // Inverter
    if (!invertMatrix(R_plus_BTPB, R_plus_BTPB, m)) {
        converged = false;
    } else {
        // BT_P_A = (B'·P)·A
        matrixMultiply(BT_P, A, BT_P_A, m, n, n);
        
        // K = (R + B'·P·B)^(-1) · (B'·P·A)
        matrixMultiply(R_plus_BTPB, BT_P_A, K, m, m, n);
    }

    // Limpeza
    delete[] Ak; delete[] Gk; delete[] Hk;
    delete[] Ak_next; delete[] Gk_next; delete[] Hk_next;
    delete[] R_inv; delete[] BT; delete[] AT;
    delete[] V; delete[] W; delete[] Temp1; delete[] Temp2; delete[] Temp3;
    delete[] BT_P; delete[] BT_P_B; delete[] BT_P_A; delete[] R_plus_BTPB;

    return converged;
}

int AutoLQR::getLastIterations() const {
    return lastIterations;
}

float AutoLQR::getLastResidual() const {
    return lastResidual;
}

int AutoLQR::getResidualHistory(float* residuals) const {
    for (int i = 0; i < 10; i++) {
        residuals[i] = residualHistory[i];
    }
    return residualHistoryCount;
}
