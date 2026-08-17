#include <math.h>
#include "Kalman.h"
//https://github.com/leech001/MPU6050
#define RAD_TO_DEG 57.295779513082320876798154814105
uint32_t timer;
extern float GyroLP[];
extern Kalman_t KalmanX = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f
};

Kalman_t KalmanY = {
        .Q_angle = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.03f,
};



//------------- Old 2019 Kalman------------------------------
float  KalmanFillerOld(Kalman_f *Kalman, float Input,float dt) {

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = Input - Kalman->Out;
    Kalman->Out += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->Out;
};

//------------- 11-2025 Kalman------------------------------
float KalmanFiller(Kalman_f *Kalman, float Input, float dt) {

    // ===============================================
    // 1. BU?C D? ÐOÁN (PREDICTION STEP)
    // ===============================================

    // C?p nh?t TR?NG THÁI (State Prediction)
    // Tr?ng thái (Kalman->Out) du?c d? doán b?ng cách tr? di ?nh hu?ng c?a Bias tích luy (dt * bias).
    // Tr?ng thái Bias (Kalman->bias) du?c coi là h?ng s? trong kho?ng dt (bias += 0).
    Kalman->Out -= Kalman->bias * dt; 

    // C?p nh?t MA TR?N HI?P PHUONG SAI (Covariance Prediction)
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // ===============================================
    // 2. BU?C C?P NH?T (CORRECTION STEP)
    // ===============================================
    
    // Tính Gain Kalman (K)
    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S; // Gain cho Out
    K[1] = Kalman->P[1][0] / S; // Gain cho Bias

    // Tính sai s? do lu?ng (Innovation)
    float y = Input - Kalman->Out;

    // C?p nh?t TR?NG THÁI U?C TÍNH (State Update)
    Kalman->Out += K[0] * y;      // U?c tính tr?ng thái chính
    Kalman->bias += K[1] * y;     // U?c tính và hi?u ch?nh Bias

    // C?p nh?t MA TR?N HI?P PHUONG SAI (Covariance Update)
    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->Out;
}
