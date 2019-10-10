#include "PID.h"
#include "helpers.h"
#include "math.h"
#include "float.h"
#include <vector>
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->p = { Kp, Ki, Kd };
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    this->twiddling = false;
}

void PID::InitTwiddle() {
    this->twiddling = true;
    this->twiddleTolerance = 0.001;
    this->iterations = 0;
    this->iterationsToIgnore = 250;
    //           p    i    d
    this->dp = { 0.05, 0.0005, 0.1 };
    this->totalError = 0;
    this->bestError = DBL_MAX;
    this->curIdx = 0;
    this->state = STARTED;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    iterations++;
    if (iterations < iterationsToIgnore)
        totalError += pow(cte, 2);
}

double PID::TotalError() {
    double err = -(p[0] * p_error + p[1] * i_error + p[2] * d_error);
    if (err < -1) return -1;
    if (err > 1) return 1;
    return err;
}

void PID::Twiddle() {
    if (twiddling && iterations > iterationsToIgnore) {
        double currErr = totalError; // / (iterationsToIgnore);
        std::cout << "currErr: " << currErr << std::endl;
        double totalChange = dp[0] + dp[1] + dp[2];
        if (totalChange > twiddleTolerance) {
            switch (state) {
                case STARTED: {
                    p[curIdx] += dp[curIdx];
                    state = INCREMENT;
                    break;
                }
                case INCREMENT: {
                    if (currErr < bestError) {
                        bestError = currErr;
                        dp[curIdx] *= 1.2;
                        curIdx = (curIdx + 1) % p.size();
                        state = STARTED;
                    } else {
                        p[curIdx] -= 2 * dp[curIdx];
                        state = DECREMENT;
                    }
                    break;
                }
                case DECREMENT: {
                    if (currErr < bestError) {
                        bestError = currErr;
                        dp[curIdx] *= 1.2;
                    } else {
                        p[curIdx] += dp[curIdx];
                        dp[curIdx] *= 0.8;
                    }
                    curIdx = (curIdx + 1) % p.size();
                    state = STARTED;
                    break;
                }
            }
            std::cout << "Coefficients dKp = " << this->dp[0] <<
                      " dKi = " << this->dp[1] <<
                      " dKd = " << this->dp[2] << std::endl;
            std::cout << "Coefficients Kp = " << this->p[0] <<
                      " Ki = " << this->p[1] <<
                      " Kd = " << this->p[2] << std::endl;
            iterations = 0;
            totalError = 0.0;
        } else {
            std::cout << "Final coefficients Kp = " << this->p[0] <<
                         " Ki = " << this->p[1] <<
                         " Kd = " << this->p[2] << std::endl;
            twiddling = false;
        }
    }
}

int PID::GetIterations() {
    return iterations;
}
