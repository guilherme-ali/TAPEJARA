#ifndef AUTO_LQR_H
#define AUTO_LQR_H

#include <Arduino.h>
#include "MatrixOperations.h"

class AutoLQR : public MatrixOperations {
public:
    /**
     * @brief Construct a new AutoLQR controller
     * @param stateSize Number of state variables
     * @param controlSize Number of control inputs
     */
    AutoLQR(int stateSize, int controlSize);

    /**
     * @brief Destroy the AutoLQR controller and free memory
     */
    ~AutoLQR();

    /**
     * @brief Set the system state matrix A
     * @param A Pointer to state matrix (stateSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool setStateMatrix(const float* A);

    /**
     * @brief Set the input matrix B
     * @param B Pointer to input matrix (stateSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setInputMatrix(const float* B);

    /**
     * @brief Set the cost matrices Q and R
     * @param Q Pointer to state cost matrix (stateSize x stateSize)
     * @param R Pointer to control cost matrix (controlSize x controlSize)
     * @return true if successful, false otherwise
     */
    bool setCostMatrices(const float* Q, const float* R);

    /**
     * @brief Compute optimal feedback gains
     * @return true if successful, false if computation fails
     */
    bool computeGains(const char* method = "SDA");

    /**
     * @brief Update the controller with current state
     * @param currentState Pointer to current state vector (stateSize)
     */
    void updateState(const float* currentState);

    /**
     * @brief Calculate control inputs based on current state
     * @param controlOutput Pointer to control output vector (controlSize)
     */
    void calculateControl(float* controlOutput);

    /**
     * @brief Set pre-computed gain values
     * @param K Pointer to gain matrix (controlSize x stateSize)
     */
    void setGains(const float* K);

    /**
     * @brief Check if the system is controllable
     * @return true if controllable, false otherwise
     */
    bool isSystemControllable();

    /**
     * @brief Get the solution of the Riccati equation
     * @return Pointer to the P matrix (stateSize x stateSize)
     */
    const float* getRicattiSolution() const;

    /**
     * @brief Estimate feedforward gain for steady-state tracking
     * @param ffGain Pointer to feedforward gain vector (controlSize)
     * @param desiredState Pointer to desired state vector (stateSize)
     */
    void estimateFeedforwardGain(float* ffGain, const float* desiredState);

    /**
     * @brief Estimate time to convergence
     * @param convergenceThreshold Threshold for considering system converged (default: 0.05)
     * @return Estimated time in seconds, or -1 if estimation fails
     */
    float estimateConvergenceTime(float convergenceThreshold = 0.05f);

    /**
     * @brief Export computed gains to external array
     * @param exportedK Pointer to destination array (controlSize x stateSize)
     * @return true if successful, false otherwise
     */
    bool exportGains(float* exportedK);

    /**
     * @brief Calculate expected cost from current state
     * @return Expected cost value, or -1 if calculation fails
     */
    float calculateExpectedCost();

    /**
     * @brief Compute Kr gains
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixKr();

    /**
     * @brief Export Kr gains to external array
     * @param exportedKr Pointer to destination array
     * @return true if successful, false otherwise
     */
    bool exportKr(float* exportedKr);

    /**
     * @brief Update the reference values
     * @param newReference Pointer to new reference vector
     */
    void updateReference(const float* newReference);

    /**
     * @brief Get the number of iterations used in last computation
     * @return Number of iterations, or -1 if not available
     */
    int getLastIterations() const;

    /**
     * @brief Get the final residual from last computation
     * @return Final residual norm, or -1 if not available
     */
    float getLastResidual() const;

    /**
     * @brief Get residuals history for first iterations
     * @param residuals Output array to store residuals (size MAX_RESIDUAL_HISTORY)
     * @return Number of valid residuals stored
     */
    int getResidualHistory(float* residuals) const;

    static const int MAX_RESIDUAL_HISTORY = 10; ///< Maximum number of residuals to store

private:
    int stateSize; ///< Number of state variables
    int controlSize; ///< Number of control inputs

    float* A; ///< State matrix
    float* B; ///< Input matrix
    float* Q; ///< State cost matrix
    float* R; ///< Control cost matrix
    float* K; ///< Control gain matrix
    float* state; ///< Current state
    float* P; ///< Riccati equation solution
    float* Kr; ///< Kr gain matrix
    float* reference; ///< To store reference values
    
    int lastIterations; ///< Number of iterations in last computation
    float lastResidual; ///< Final residual norm from last computation
    float residualHistory[10]; ///< Residuals for first 10 iterations
    int residualHistoryCount; ///< Number of valid entries in residualHistory

    /// Scratch do SDA, alocado uma unica vez no construtor. O SDA roda a cada
    /// ciclo do SDRE (~200 Hz); fazer 13 new/delete por chamada custava tempo e
    /// fragmentava o heap. Layout: 9 blocos n*n + 1 de m*m + 3 de m*n.
    float* sdaScratch;

    /**
     * @brief Compute the optimal gain matrix by solving DARE (iterative method)
     * @return true if successful, false otherwise
     */
    bool computeGainMatrix();
    
    /**
     * @brief Compute the optimal gain matrix using Schur method (QZ decomposition)
     * Solves DARE using generalized Schur decomposition for direct solution
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSchur();

    /**
     * @brief Compute the optimal gain matrix using Van Dooren's method
     * Solves DARE using Van Dooren's algorithm for numerical stability
     * @return true if successful, false otherwise
     */    
    bool computeGainMatrixVanDooren();

    /**
     * @brief Compute the optimal gain matrix using SDA method
     * Solves DARE using State-dependent Riccati Equation for adaptive control
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSDA();

    /**
     * @brief Caminho rápido do SDA em fixed-point Q13.18 (ESP32-S2 sem FPU)
     * Resolve a DARE em int32 (~2.7× mais rápido que float, erro do K < 1%).
     * Validado só para o caso 6 estados / 3 controles. Retorna false em
     * overflow/saturação ou matriz singular → o chamador faz fallback p/ o SDA float.
     * @return true se sucesso, false se deve cair no fallback float
     */
    bool computeGainMatrixSDA_Fixed();

    /**
     * @brief Compute the optimal gain matrix using Iterative method
     * Solves DARE using basic iterative approach for simplicity
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixIterative();
    
    /**
     * @brief Compute the optimal gain matrix using SDA with Single Shift (SDA-ss)
     * Enhanced SDA with shift parameter for better convergence when eigenvalues near 1
     * Reference: "Structure-preserving algorithms for periodic discrete-time algebraic Riccati equations"
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSDA_SS();
    
    /**
     * @brief Compute the optimal gain matrix using Adaptive SDA (ASDA)
     * Adaptive scaling during iterations for improved numerical stability
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixASDA();
    
    /**
     * @brief Compute the optimal gain matrix using Scaled SDA
     * Uses optimal scaling for better conditioning of Hamiltonian pencil
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixSDA_Scaled();
    
    /**
     * @brief Compute the optimal gain matrix using ADDA (Alternating-Directional Doubling Algorithm)
     * Variant of SDA that alternates multiplication order between iterations
     * for improved numerical stability on certain problems
     * Reference: Lin, Xu, "On the Doubling Algorithm for a (Shifted) Nonsymmetric
     *            Algebraic Riccati Equation", 2007
     * @return true if successful, false otherwise
     */
    bool computeGainMatrixADDA();
};

#endif
