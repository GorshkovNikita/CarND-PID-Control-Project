#include "PID.h"
#include "helpers.h"
#include "math.h"
#include "float.h"
#include <vector>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    this->twiddling = false;
}

void PID::InitTwiddle() {
    this->twiddling = true;
    this->twiddleTolerance = 0.001;
    this->iterations = 0;
    //           p    i    d
    this->dp = { 1.0, 1.0, 1.0};
    this->totalError = 0;
    this->bestError = DBL_MAX;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    iterations++;
}

void PID::Twiddle() {
    if (twiddling) {
        totalError += pow(TotalError(), 2);
        double currErr = totalError / iterations;
        double totalChange = dp[0] + dp[1] + dp[2];
        if (totalChange > twiddleTolerance) {
            std::cout << "Twiddling ..." << std::endl;
            int idx = iterations % dp.size();
            
        } else {
            std::cout << "Final coefficients Kp = " << this->Kp <<
                         " Ki = " << this->Ki <<
                         " Kd = " << this->Kd << std::endl;
            twiddling = false;
        }
    }
}

double PID::TotalError() {
    return - Kp * p_error - Ki * i_error - Kd * d_error;
}
